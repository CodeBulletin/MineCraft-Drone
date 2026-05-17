term.clear()
term.setCursorPos(1,1)

local pid = require("lib.pid")
local utils = require("lib.utils")
local sensors = require("lib.sensors")
local mixer = require("lib.mixer")
local flightState = require("lib.flight.state")
local flightYaw = require("lib.flight.yaw")
local flightRecovery = require("lib.flight.recovery")

--------------------------------------------------
-- Setup & Config
--------------------------------------------------
for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on " .. side)
        break
    end
end

if not fs.exists("config.txt") then error("Run master_setup.lua first") end
local file = fs.open("config.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

local network = config.network
local controllerId = config.controller

print("Master ready")

--------------------------------------------------
-- State Context Array (ctx)
--------------------------------------------------
local ctx = {
    -- Config / Globals
    dt = 0.001,
    motors = config.motors,
    network = network,
    baseThrottle = 110,
    power = 8,
    yawPower = 6,
    ALIGN_VEL_TOLERANCE = 0.2,
    ALIGN_ANG_TOLERANCE = 0.1,
    
    -- Telemetry & Targets
    targetY = 80, targetX = 0.5, targetZ = 0.5,
    targetPitch = 0, targetRoll = 0, targetYaw = 0, targetYawRate = 0,
    yCorr = 0,
    
    -- Dynamic State
    firstTick = true, wasManual = false, wasYawing = false, wasRecovery = false,
    alignState = false, activePathKey = "",
    recoveryTimer = 0, jumpPulseTimer = 0,
    tiltLimit = 0.4, velLimit = 20.0,
    
    control = { fb = 0, rl = 0, yc = 0, th = 0, manual = false },

    -- PID Controllers
    pitchPID     = pid.create(6.0, 0.00, 0.0),
    rollPID      = pid.create(6.0, 0.00, 0.0),
    yawPID       = pid.create(8.0, 0.00, 1.5),
    pitchRatePID = pid.create(1.2, 0.02, 0.6),
    rollRatePID  = pid.create(1.2, 0.02, 0.6),
    yawRatePID   = pid.create(1.2, 0.02, 0.6),
    altPID       = pid.create(3.0, 0.0, 0.5),
    velPID       = pid.create(12.0, 0.8, 0.0),
    posXPID      = pid.create(0.5, 0.0, 0.0),
    posZPID      = pid.create(0.5, 0.0, 0.0),
    velXPID      = pid.create(0.05, 0.0, 0.03),
    velZPID      = pid.create(0.05, 0.0, 0.03),
    pathCrossPID = pid.create(0.6, 0.0, 0.3),
    pathAlongPID = pid.create(0.6, 0.0, 0.3)
}

--------------------------------------------------
-- Threads
--------------------------------------------------
local function networkThread()
    local s = sensors.read()
    local bootPing = {
        type = "telemetry", x = s.x, y = s.alt, z = s.z,
        vx = s.vx, vy = s.vy, vz = s.vz, yaw = s.yaw, pitch = s.pitch, roll = s.roll
    }

    if controllerId then rednet.send(controllerId, bootPing, network)
    else rednet.broadcast(bootPing, network) end

    while true do
        local id, msg = rednet.receive(network)
        if not controllerId then controllerId = id end

        if msg and msg.type == "control" then
            ctx.control.fb = msg.fb or 0
            ctx.control.rl = msg.rl or 0
            ctx.control.yc = msg.yc or 0
            ctx.control.th = msg.th or 0
            ctx.control.manual = msg.manual or false
            ctx.control.hasTarget = msg.hasTarget or false
            if ctx.control.hasTarget then
                ctx.control.targetX = msg.targetX
                ctx.control.targetZ = msg.targetZ
                ctx.control.targetY = msg.targetY
                ctx.control.prevX = msg.prevX
                ctx.control.prevZ = msg.prevZ
                ctx.control.targetYaw = msg.targetYaw
                ctx.control.useTargetYaw = msg.useTargetYaw
            end
        end
    end
end

local function flightThread()
    local lastTime = os.clock()

    while true do
        local now = os.clock()
        ctx.dt = math.max(now - lastTime, 0.001)
        lastTime = now

        -- 1. Read Sensors & Update Context vars
        local s = sensors.read()
        for k, v in pairs(s) do ctx[k] = v end

        if ctx.firstTick then   
            ctx.targetY = (ctx.control.hasTarget and ctx.control.targetY) and ctx.control.targetY or ctx.alt
            ctx.targetYaw = ctx.yaw
            if ctx.control.hasTarget then
                ctx.targetX = ctx.control.targetX; ctx.targetZ = ctx.control.targetZ
            else
                ctx.targetX = ctx.x; ctx.targetZ = ctx.z
            end
            ctx.firstTick = false
        end

        -- 2. Altitude Setup
        local climbRate = ctx.control.th * 6.0 
        if not (ctx.control.hasTarget and ctx.control.targetY) then
            ctx.targetY = ctx.targetY + climbRate * ctx.dt
        else 
            ctx.targetY = ctx.control.targetY
        end
        local targetVY = utils.clamp(pid.update(ctx.altPID, ctx.targetY, ctx.alt, ctx.dt), -10.0, 10.0)   
        ctx.yCorr = pid.update(ctx.velPID, targetVY, ctx.vy, ctx.dt)

        -- 3. Drone-Local Velocity Matrix
        ctx.cosY = math.cos(ctx.yaw)
        ctx.sinY = math.sin(ctx.yaw)
        ctx.localVX = ctx.cosY * ctx.vx - ctx.sinY * ctx.vz
        ctx.localVZ = ctx.sinY * ctx.vx + ctx.cosY * ctx.vz

        -- 4. Dispatch Navigation State
        flightState.dispatch(ctx)
        
        -- 5. Yaw & Recovery Logic
        flightYaw.update(ctx)
        local throttle, invert, multiplyer = flightRecovery.update(ctx)

        -- 6. Rate PID Corrections
        local tpaFactor = utils.getTPA(throttle, ctx.baseThrottle)
        
        local targetPitchRate = pid.update(ctx.pitchPID, utils.angleDiff(ctx.targetPitch, ctx.pitch), 0, ctx.dt)
        local targetRollRate  = pid.update(ctx.rollPID, utils.angleDiff(ctx.targetRoll, ctx.roll), 0, ctx.dt)
        
        local yawCorr   = pid.update(ctx.yawRatePID, ctx.targetYawRate, ctx.yawRate, ctx.dt) * ctx.yawPower * multiplyer
        local pitchCorr = pid.update(ctx.pitchRatePID, targetPitchRate, ctx.pitchRate, ctx.dt) * ctx.power * tpaFactor * multiplyer
        local rollCorr  = pid.update(ctx.rollRatePID, targetRollRate, ctx.rollRate, ctx.dt) * ctx.power * tpaFactor * multiplyer

        -- 7. Motor Mixer
        mixer.apply(ctx, throttle, invert, multiplyer, pitchCorr, rollCorr, yawCorr)

        -- 8. CLI Print
        term.clear()
        term.setCursorPos(1,1)
        print("=== DRONE STATE ===")
        print(string.format("dt: %.4f freq: %0.4f ver: 1.5", ctx.dt, 1.0/ctx.dt))
        print(string.format("\nPitch: %.3f | %.3f\nRoll : %.3f | %.3f\nYaw  : %.3f | %.3f", ctx.pitch, ctx.targetPitch, ctx.roll, ctx.targetRoll, ctx.yaw, ctx.targetYaw))
        print(string.format("\nAlt: X Y Z : %.3f %.3f %.3f", ctx.x, ctx.alt, ctx.z))
        print(string.format("Target     : %.3f %.3f %.3f", ctx.targetX, ctx.targetY, ctx.targetZ))
        print(string.format("Manual: %s | Aligning: %s", tostring(ctx.control.manual), tostring(ctx.alignState)))

        sleep(0.02)
    end
end

local function telemetryThread()
    while true do
        if controllerId then
            local s = sensors.read()
            rednet.send(controllerId, {
                type = "telemetry",
                x = s.x, y = s.alt, z = s.z,
                vx = s.vx, vy = s.vy, vz = s.vz,
                yaw = s.yaw, pitch = s.pitch, roll = s.roll,
            }, network)
        end
        sleep(0.2)
    end
end

parallel.waitForAny( networkThread, flightThread, telemetryThread )