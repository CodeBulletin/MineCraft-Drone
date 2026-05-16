term.clear()
term.setCursorPos(1,1)

-- os.exit(1)
--------------------------------------------------
-- Network Setup
--------------------------------------------------

for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on " .. side)
        break
    end
end

--------------------------------------------------
-- load config
--------------------------------------------------

if not fs.exists("config.txt") then
    error("Run master_setup.lua first")
end

local file = fs.open("config.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

local network = config.network
local motors = config.motors
local controllerId = config.controller

--------------------------------------------------
-- Constants
--------------------------------------------------
local power = 8
local yawPower = 6
local targetY = 80
local targetX = 0.5
local targetZ = 0.5
SMOOTHING = 0.8
local baseThrottle = 110
local vxFiltered = 0    
local vzFiltered = 0

print("Master ready")

--------------------------------------------------
-- Helper Functions
--------------------------------------------------

local function isValidNumber(x)
    return type(x) == "number"
       and x == x
       and x ~= math.huge
       and x ~= -math.huge
end

local function createPID(kp, ki, kd)
    return {
        kp = kp,
        ki = ki,
        kd = kd,
        integral = 0,
        lastError = 0,
    }
end

local function updatePID(pid, targetRate, currentRate, dt)
    local error = targetRate - currentRate
    
    pid.integral = pid.integral + error * dt
    pid.integral = math.max(-50, math.min(50, pid.integral))

    local derivative = (error - pid.lastError) / dt

    if not isValidNumber(derivative) then derivative = 0 end
    
    pid.lastError = error
    
    return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative
end

local function clampInt(v, min, max)
    v = math.floor(v + 0.5) -- round to nearest int
    return math.max(min, math.min(max, v))
end

local function clamp(v, min, max)
    return math.max(min, math.min(max, v))
end

local function smooth(new, old, factor)
    return old + (new - old) * factor
end

local function angleDiff(target, current)
    local diff = target - current
    while diff > math.pi do diff = diff - 2*math.pi end
    while diff < -math.pi do diff = diff + 2*math.pi end
    return diff
end

-- Add a TPA function
local function getTPA(throttle, base)
    -- If throttle is lower than base, reduce gains
    -- This scales gains from 100% at hover to 70% at min throttle
    if throttle < base then
        local factor = (throttle - 30) / (base - 30)
        return math.max(0.7, factor)
    end
    return 1.0
end

--------------------------------------------------
-- PID
--------------------------------------------------

local pitchPID = createPID(6.0, 0.00, 0.0)
local rollPID  = createPID(6.0, 0.00, 0.0)
local yawPID   = createPID(8.0, 0.00, 1.5)

local pitchRatePID = createPID(1.2, 0.02, 0.6)
local rollRatePID  = createPID(1.2, 0.02, 0.6)
local yawRatePID   = createPID(1.2, 0.02, 0.6)

local altPID =  createPID(3.0, 0.0, 0.5)
local velPID =  createPID(12.0, 0.8, 0.0)

local posXPID =  createPID(0.5, 0.0, 0.0)
local posZPID =  createPID(0.5, 0.0, 0.0)

local velXPID =  createPID(0.05, 0.0, 0.03)
local velZPID =  createPID(0.05, 0.0, 0.03)

local control = { fb = 0, rl = 0, yc = 0, th = 0, manual = false }

--------------------------------------------------
-- Network Thread
--------------------------------------------------

local function networkThread()
    while true do
        local id, msg = rednet.receive("Comm1")
        if not controllerId then
            controllerId = id
        end

        if msg and msg.type == "control" then
            control.fb = msg.fb or 0
            control.rl = msg.rl or 0
            control.yc = msg.yc or 0
            control.th = msg.th or 0
            control.manual = msg.manual or false
            control.hasTarget = msg.hasTarget or false
            if control.hasTarget then
                control.targetX = msg.targetX
                control.targetZ = msg.targetZ
                control.targetY = msg.targetY   -- <<< ADD
            end
        end
    end
end

--------------------------------------------------
-- Main Thread
--------------------------------------------------
local function flightThread()
    local lastTime = os.clock()

    local targetPitch = 0
    local targetRoll = 0
    local targetYaw = 0 

    local wasManual = false

    local firstTick = true

    -- Constants (no altitude numbers!)
    local RECOVERY_JUMP_THROTTLE = 240
    local RECOVERY_AIR_THROTTLE = 100
    local STUCK_TIME = 1.0        -- seconds before we declare "stuck"
    local JUMP_PULSE_TIME = 0.35  -- how long to fire jump throttle

    -- State variables (add with your other locals)
    local recoveryTimer = 0
    local jumpPulseTimer = 0
    local wasRecovery = false
    local wasYawing = false


    while true do
            local now = os.clock()
            local dt = now - lastTime
            if dt <= 0 then dt = 0.001 end
            lastTime = now

            --------------------------------------------------
            -- sensors
            --------------------------------------------------
            local pose = sublevel.getLogicalPose()
            local vel = sublevel.getLinearVelocity()
            local ang = sublevel.getAngularVelocity()
            
            local pitch, yaw, roll = pose.orientation:toEuler()
            local alt = pose.position.y
            local x = pose.position.x
            local z = pose.position.z
            local vy = vel.y
            local vx = vel.x
            local vz = vel.z

            if firstTick then   
                if control.hasTarget and control.targetY then
                    targetY = control.targetY   -- <<< ADD
                else 
                    targetY = alt
                end
                targetYaw = yaw
                if control.hasTarget then
                    targetX = control.targetX
                    targetZ = control.targetZ
                else
                    targetX = x
                    targetZ = z
                end
                firstTick = false
            end

            local pitchRate = ang.x
            local yawRate = ang.y
            local rollRate = ang.z

            local climbRate = control.th * 6.0 
            if not (control.hasTarget and control.targetY) then
                targetY = targetY + climbRate * dt
            else 
                targetY = control.targetY
            end
            local targetVY = updatePID(altPID, targetY, alt, dt)
            targetVY = clamp(targetVY, -10.0, 10.0)   
            local yCorr = updatePID(velPID, targetVY, vy, dt)

            if math.abs(vx) < 0.02 then vx = 0 end
            if math.abs(vz) < 0.02 then vz = 0 end

            vxFiltered = smooth(vx, vxFiltered or 0, SMOOTHING)
            vzFiltered = smooth(vz, vzFiltered or 0, SMOOTHING)

            vx = vxFiltered
            vz = vzFiltered

            if control.manual then
                wasManual = true

                targetPitch =
                    control.fb * 0.35

                targetRoll =
                    -control.rl * 0.35
                targetX = x
                targetZ = z
            elseif vel.y < -2.0 then
                targetPitch = 0
                targetRoll = 0
            else

                if wasManual then
                    posXPID.integral = 0; posXPID.lastError = 0
                    posZPID.integral = 0; posZPID.lastError = 0
                    velXPID.integral = 0; velXPID.lastError = 0
                    velZPID.integral = 0; velZPID.lastError = 0
                    wasManual = false
                end

                if control.hasTarget then
                    targetX = control.targetX
                    targetZ = control.targetZ
                else
                    -- Normal position hold with 5-block soft reset
                    local dx = targetX - x
                    local dz = targetZ - z
                    local distFromTarget = math.sqrt(dx*dx + dz*dz)
                    if distFromTarget > 5 then
                        targetX = x
                        targetZ = z
                        posXPID.integral = 0; posXPID.lastError = 0
                        posZPID.integral = 0; posZPID.lastError = 0
                        velXPID.integral = 0; velXPID.lastError = 0
                        velZPID.integral = 0; velZPID.lastError = 0
                    end
                end

                local cosY = math.cos(yaw)
                local sinY = math.sin(yaw)

                local localVX =  cosY * vx - sinY * vz
                local localVZ =  sinY * vx + cosY * vz

                local dx = targetX - x
                local dz = targetZ - z

                local localDX =  cosY * dx - sinY * dz
                local localDZ =  sinY * dx + cosY * dz

                -- if math.abs(localDX) < 0.15 then
                --     localDX = 0
                -- end

                -- if math.abs(localDZ) < 0.15 then
                --     localDZ = 0
                -- end

                local targetVX = updatePID(posXPID, 0, -localDX, dt)
                local targetVZ = updatePID(posZPID, 0, -localDZ, dt)

                local distToTarget = math.sqrt(dx*dx + dz*dz)

                local velLimit, tiltLimit
                if control.hasTarget and distToTarget > 10 then
                    velLimit = 20.0
                    tiltLimit = 0.4
                else
                    velLimit = 3.0
                    tiltLimit = 0.15
                end

                targetVX = clamp(targetVX, -velLimit, velLimit)
                targetVZ = clamp(targetVZ, -velLimit, velLimit)

                targetPitch = clamp(updatePID(velZPID, targetVZ, localVZ, dt), -tiltLimit, tiltLimit)
                targetRoll  = clamp(-updatePID(velXPID, targetVX, localVX, dt), -tiltLimit, tiltLimit)
            end
            
            -- Yaw stabilization
            local targetYawRate
            local yawStickActive = math.abs(control.yc) > 0.4

            if yawStickActive then
                -- Manual rate control
                targetYawRate = control.yc * 4.0
                targetYaw = yaw          -- keep updating so we know where we are
                wasYawing = true
            else
                if wasYawing then
                    -- Stick just released: snap target to current heading
                    targetYaw = yaw
                    wasYawing = false

                    -- CRITICAL: clear stale history so old errors don't bias the hold
                    yawPID.integral = 0
                    yawPID.lastError = 0
                    yawRatePID.integral = 0
                    yawRatePID.lastError = 0
                end

                -- Position hold with natural damping
                -- D term sees error growing during overshoot and fights it immediately
                targetYawRate = updatePID(yawPID, angleDiff(targetYaw, yaw), 0, dt)
            end

            -- Pitch Roll Stablization

            local pitchError = angleDiff(targetPitch, pitch)
            local rollError = angleDiff(targetRoll, roll)

            local targetPitchRate = updatePID(pitchPID, pitchError, 0, dt)
            local targetRollRate  = updatePID(rollPID, rollError, 0, dt)

            -- Calculate how much we are currently rotating (absolute value)
            local currentYawActivity = math.abs(yawRate)
            local yawCompensation = currentYawActivity * 2.5

            -- Apply tiltFactor to automatically boost thrust during maneuvers
            local upVector = math.cos(pitch) * math.cos(roll)
            local recoveryMode = upVector < 0.2

            if recoveryMode then
                if not wasRecovery then
                    recoveryTimer = 0
                    jumpPulseTimer = 0
                end
                recoveryTimer = recoveryTimer + dt
            else
                recoveryTimer = 0
                jumpPulseTimer = 0
            end
            wasRecovery = recoveryMode

            -- STUCK LOGIC:
            -- If inverted for > STUCK_TIME and vertical velocity is near zero,
            -- we are physically blocked (ground, wall, or prop strike).
            -- We IGNORE altitude entirely.
            local vyNearlyZero = math.abs(vy) < 0.8
            local isStuck = recoveryMode and (recoveryTimer > STUCK_TIME) and vyNearlyZero

            -- JUMP PULSE:
            -- If stuck, we fire a short burst of max throttle to break contact.
            -- Once jumpPulseTimer > 0, we keep jumping until it expires.
            if isStuck and jumpPulseTimer == 0 then
                jumpPulseTimer = JUMP_PULSE_TIME  -- start jump
            end

            if jumpPulseTimer > 0 then
                jumpPulseTimer = jumpPulseTimer - dt
                if jumpPulseTimer < 0 then jumpPulseTimer = 0 end
            end

            local isJumping = jumpPulseTimer > 0

            local throttle = 0

            local multiplyer = 1.0
            local invert = 1.0
            if recoveryMode then
                -- Reset targets so we don't fly sideways while tumbling
                targetX = x
                targetZ = z

                -- Kill ALL integral windup while recovering
                altPID.integral = 0; altPID.lastError = 0
                velPID.integral = 0; velPID.lastError = 0
                posXPID.integral = 0; posXPID.lastError = 0
                posZPID.integral = 0; posZPID.lastError = 0
                velXPID.integral = 0; velXPID.lastError = 0
                velZPID.integral = 0; velZPID.lastError = 0

                if isJumping then
                    -- JUMP PHASE: Break ground contact with raw power.
                    -- Physics constraints disappear once we leave the ground.
                    throttle = RECOVERY_JUMP_THROTTLE
                    
                    -- Dampen all attitude corrections during jump.
                    -- We want LIFT, not torque, while touching the ground.
                    multiplyer = 0.05
                    invert = -1.0
                else
                    -- FLIP PHASE: Either airborne-inverted, or post-jump.
                    -- Moderate throttle gives rotational authority without rocketing away.
                    throttle = RECOVERY_AIR_THROTTLE
                end
            else
                -- NORMAL FLIGHT (your existing logic)
                local tiltFactor = 1.0 / math.max(0.5, upVector)
                local yawCompensation = math.abs(yawRate) * 2.5
                throttle = math.max(30, math.min((baseThrottle + yCorr) * tiltFactor - yawCompensation, 250))
            end

            local tpaFactor = getTPA(throttle, baseThrottle)

            -- Error Calculations
            local yawCorr = updatePID(yawRatePID, targetYawRate, yawRate, dt) * yawPower * multiplyer
            local pitchCorr = updatePID(pitchRatePID, targetPitchRate, pitchRate, dt) * power * tpaFactor * multiplyer
            local rollCorr  = updatePID(rollRatePID, targetRollRate, rollRate, dt) * power * tpaFactor * multiplyer


            --------------------------------------------------
            -- Balanced Motor Mixer
            --------------------------------------------------
            local rawSpeeds = {}
            local minSpeed = math.huge
            local maxSpeed = -math.huge
            local sumOffset = 0

            for i, motor in ipairs(motors) do
                -- Calculate only the PIDs contribution (offset from throttle)
                local offset = - pitchCorr * motor.z
                               - rollCorr  * motor.x
                               - yawCorr   * motor.spin
                
                rawSpeeds[i] = offset
                sumOffset = sumOffset + offset
            end

            -- Calculate how much the 'average' motor speed changed
            -- This helps prevent the "climb on yaw" effect
            local avgOffset = sumOffset / #motors
            
            for i=1, #rawSpeeds do
                -- Add the requested throttle, but subtract the bias added by PIDs
                rawSpeeds[i] = throttle + rawSpeeds[i] - avgOffset
                
                if rawSpeeds[i] < minSpeed then minSpeed = rawSpeeds[i] end
                if rawSpeeds[i] > maxSpeed then maxSpeed = rawSpeeds[i] end
            end

            -- Air Mode / Saturation Prevention
            -- local boost = 0
            -- if minSpeed < 15 then
            --     boost = 15 - minSpeed
            -- elseif maxSpeed > 255 then
            --     boost = 255 - maxSpeed -- This will be a negative number (reduction)
            -- end

            for i, motor in ipairs(motors) do
                local finalSpeed = clampInt(invert *    rawSpeeds[i], -255, 255)
                
                rednet.send(
                    motor.id,
                    {
                        speed = finalSpeed,
                        tilt = motor.spin * 3
                    },
                    network
                )
            end

            term.clear()
            term.setCursorPos(1,1)

            print("=== DRONE STATE ===")
            print(string.format("dt: %.4f freq: %0.4f ver: 1.1", dt, 1.0/dt))

            print("\n-- Orientation (radians) --")
            print(string.format("Pitch: %.3f", pitch))
            print(string.format("Roll : %.3f", roll))
            print(string.format("Yaw  : %.3f", yaw))

            print("\n-- Altitued --")
            print(string.format("X Y Z  : %.3f %.3f %.3f", pose.position.x, pose.position.y, pose.position.z))
            print(string.format("Target : %.3f %.3f %.3f", targetX, targetY, targetZ))
            print(string.format("Vel    : %.3f %.3f %.3f", vel.x, vel.y, vel.z))

            print("\n-- Control --")

            print(string.format(
                "FB      : %.3f",
                control.fb
            ))

            print(string.format(
                "RL      : %.3f",
                control.rl
            ))

            print(string.format(
                "YawCtrl : %.3f",
                control.yc
            ))

            print(string.format(
                "Throttle: %.3f",
                control.th
            ))

            print(string.format(
                "Manual  : %s",
                tostring(control.manual)
            ))

            sleep(0.02)
    end
end

--------------------------------------------------
-- Return Thread
--------------------------------------------------
local function telemetryThread()
    while true do
        if controllerId then
            local pose = sublevel.getLogicalPose()
            local vel = sublevel.getLinearVelocity()
            local p, y, r = pose.orientation:toEuler()
            rednet.send(controllerId, {
                type = "telemetry",
                x = pose.position.x,
                y = pose.position.y,
                z = pose.position.z,
                vx = vel.x,
                vy = vel.y,
                vz = vel.z,
                yaw = y,
                pitch = p,
                roll = r,
            }, network)
        end
        sleep(0.2)
    end
end

parallel.waitForAny( networkThread, flightThread, telemetryThread )