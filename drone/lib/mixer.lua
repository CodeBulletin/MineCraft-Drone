local utils = require("lib.utils")
local mixer = {}

-- Load config once when the module is required so we have the calibration data
local file = fs.open("config.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

function mixer.apply(ctx, throttle, invert, multiplyer, pitchCorr, rollCorr, yawCorr)
    local rawSpeeds = {}
    local minSpeed = math.huge
    local maxSpeed = -math.huge
    local sumOffset = 0

    -- 1. Apply Calibrations from config.txt
    local pCorr = pitchCorr * (config.pitchInvert or 1)
    local rCorr = rollCorr * (config.rollInvert or 1)
    local yCorr = yawCorr * (config.yawInvert or 1)
    
    -- If the physical X and Z axes are rotated 90 degrees, swap the outputs
    if config.swapPitchRoll then
        pCorr, rCorr = rCorr, pCorr
    end

    for i, motor in ipairs(ctx.motors) do
        -- 2. Use the calibrated corrections instead of the raw ones
        local offset = - pCorr * motor.z
                       - rCorr * motor.x
                       - yCorr * motor.spin
        
        rawSpeeds[i] = offset
        sumOffset = sumOffset + offset
    end

    local avgOffset = sumOffset / #ctx.motors
    
    for i=1, #rawSpeeds do
        rawSpeeds[i] = throttle + rawSpeeds[i] - avgOffset
        
        if rawSpeeds[i] < minSpeed then minSpeed = rawSpeeds[i] end
        if rawSpeeds[i] > maxSpeed then maxSpeed = rawSpeeds[i] end
    end

    for i, motor in ipairs(ctx.motors) do
        local finalSpeed = utils.clampInt(invert * rawSpeeds[i], -255, 255)
        
        rednet.send(
            motor.id,
            { speed = finalSpeed, tilt = motor.spin * 3 },
            ctx.network
        )
    end
end

return mixer