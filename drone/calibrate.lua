local sensors = require("lib.sensors")
local utils = require("lib.utils")

term.clear()
term.setCursorPos(1,1)
print("=== EXTENDED DRONE CALIBRATION ===")
print("Keep clear! Drone will perform sustained tests.")
print("Preparing sensors...")
sleep(2)

if not fs.exists("config.txt") then error("Run master_setup.lua first") end
local file = fs.open("config.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

-- Open modem
for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        break
    end
end

-- Function to apply a sustained force over time and accumulate readings
local function sustainedTest(axisName, p, r, y)
    -- baseThrottle should be just enough to let the drone tilt/slip on the ground
    local baseThrottle = 85 
    local runDuration = 2.0     -- Duration of motor pulse in seconds
    local sampleInterval = 0.05 -- Sample every 50ms
    local steps = math.floor(runDuration / sampleInterval)
    
    local totalAng = { x = 0, y = 0, z = 0 }
    
    print(string.format("Testing %s axis for %.1fs...", axisName, runDuration))
    
    -- Continuous drive and sample loop
    for i = 1, steps do
        -- Keep sending motor signals every frame to prevent motor timeouts
        for _, m in ipairs(config.motors) do
            local offset = - p * m.z - r * m.x - y * m.spin
            rednet.send(m.id, { speed = baseThrottle + offset, tilt = m.spin * 3 }, config.network)
        end
        
        sleep(sampleInterval)
        
        -- Read and accumulate angular rates over time
        local s = sensors.read()
        totalAng.x = totalAng.x + s.ang.x
        totalAng.y = totalAng.y + s.ang.y
        totalAng.z = totalAng.z + s.ang.z
    end
    
    -- Immediately cut all motors
    for _, m in ipairs(config.motors) do
        rednet.send(m.id, { speed = 0, tilt = 0 }, config.network)
    end
    
    print("-> Axis complete. Waiting for drone to settle...")
    sleep(2.5) -- Extended cooldown to ensure drone comes to a complete rest
    
    return totalAng
end

-- Execute the extended tests
local pRes = sustainedTest("PITCH", 35, 0, 0)
local rRes = sustainedTest("ROLL",  0, 35, 0)
local yRes = sustainedTest("YAW",   0, 0, 35)

print("\n=== ANALYZING ACCUMULATED DATA ===")

local swapPitchRoll = false
local pitchInvert = 1
local rollInvert = 1
local yawInvert = 1

-- Check whether a Pitch command registered more strongly on Roll (Z) than Pitch (X)
if math.abs(pRes.z) > math.abs(pRes.x) then
    swapPitchRoll = true
    print("- Pitch and Roll are SWAPPED")
    
    -- Detect inversions when axes are swapped
    pitchInvert = pRes.z > 0 and 1 or -1
    rollInvert = rRes.x > 0 and 1 or -1
else
    print("- Axes are correctly mapped")
    
    -- Detect inversions when axes are normal
    pitchInvert = pRes.x > 0 and 1 or -1
    rollInvert = rRes.z > 0 and 1 or -1
end

-- Yaw always targets the Y axis
yawInvert = yRes.y > 0 and 1 or -1

-- Print out final status reports
print(string.format("  Pitch Multiplier: %d", pitchInvert))
print(string.format("  Roll Multiplier:  %d", rollInvert))
print(string.format("  Yaw Multiplier:   %d", yawInvert))

-- Merge calibration into config table
config.swapPitchRoll = swapPitchRoll
config.pitchInvert = pitchInvert
config.rollInvert = rollInvert
config.yawInvert = yawInvert

-- Save back out to file
local fw = fs.open("config.txt","w")
fw.write(textutils.serialize(config))
fw.close()

print("\nCalibration successfully saved to config.txt!")