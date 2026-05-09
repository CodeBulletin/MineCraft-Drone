term.clear()
term.setCursorPos(1,1)

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

--------------------------------------------------
-- Constants
--------------------------------------------------
local power = 8
local yawPower = 14
local targetAlt = 80
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
local yawPID   = createPID(6.0, 0.00, 0.0)

local pitchRatePID = createPID(1.2, 0.02, 0.6)
local rollRatePID  = createPID(1.2, 0.02, 0.6)
local yawRatePID   = createPID(1.2, 0.02, 0.4)

local altPID =  createPID(2.0, 0.0, 0.0)
local velPID =  createPID(12.0, 0.8, 3.0)

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

        local id, msg =
            rednet.receive("Comm1")

        if msg and msg.type == "control" then

            control.fb = msg.fb or 0
            control.rl = msg.rl or 0
            control.yc = msg.yc or 0
            control.th = msg.th or 0

            control.manual =
                msg.manual or false
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

            local pitchRate = ang.x
            local yawRate = ang.y
            local rollRate = ang.z
            if math.abs(vx) < 0.02 then vx = 0 end
            if math.abs(vz) < 0.02 then vz = 0 end

            vxFiltered = smooth(vx, vxFiltered or 0, SMOOTHING)
            vzFiltered = smooth(vz, vzFiltered or 0, SMOOTHING)

            vx = vxFiltered
            vz = vzFiltered

            
            if control.manual then

                --------------------------------------------------
                -- Direct stick control
                --------------------------------------------------

                targetPitch =
                    control.fb * 0.35

                targetRoll =
                    -control.rl * 0.35

                --------------------------------------------------
                -- Update hover target
                --------------------------------------------------

                targetX = x
                targetZ = z

            else

                local cosY = math.cos(yaw)
                local sinY = math.sin(yaw)

                local localVX =  cosY * vx - sinY * vz
                local localVZ =  sinY * vx + cosY * vz

                local dx = targetX - x
                local dz = targetZ - z

                local localDX =  cosY * dx - sinY * dz
                local localDZ =  sinY * dx + cosY * dz

                if math.abs(localDX) < 0.15 then
                    localDX = 0
                end

                if math.abs(localDZ) < 0.15 then
                    localDZ = 0
                end

                local targetVX = updatePID(posXPID, 0, -localDX, dt)
                local targetVZ = updatePID(posZPID, 0, -localDZ, dt)

                targetVX = clamp(targetVX, -3.0, 3.0)
                targetVZ = clamp(targetVZ, -3.0, 3.0)

                targetPitch = clamp(updatePID(velXPID, targetVZ, localVZ, dt), -0.25, 0.25)
                targetRoll  = clamp(-updatePID(velZPID, targetVX, localVX, dt), -0.25, 0.25)
            end

            -- xyz satbilzation
            local targetYawRate

            if math.abs(control.yc) > 0.05 then
                targetYawRate =
                    control.yc * 4.0

                targetYaw = yaw

            else
                targetYawRate =
                    updatePID(
                        yawPID,
                        angleDiff(targetYaw, yaw),
                        0,
                        dt
                    )
            end

            -- Pitch Roll Velocity Stablization
            local climbRate = control.th * 3.0 
            -------------------------------------------------- -- Limit descent speed --------------------------------------------------
            targetAlt = targetAlt + climbRate * dt

            local targetVelRate = updatePID(altPID, targetAlt, alt, dt)
            local targetPitchRate = updatePID(pitchPID, targetPitch, pitch, dt)
            local targetRollRate  = updatePID(rollPID, targetRoll, roll, dt)

            targetVelRate = clamp(targetVelRate, -10.0, 10.0)

            local velCorr = updatePID(velPID, targetVelRate, vy, dt)
            local throttle = math.max(30, math.min(baseThrottle + velCorr, 250));
            local tpaFactor = getTPA(throttle, baseThrottle)

            -- Error Calculations
            local yawCorr = updatePID(yawRatePID, targetYawRate, yawRate, dt) * yawPower
            local pitchCorr = updatePID(pitchRatePID, targetPitchRate, pitchRate, dt) * power * tpaFactor
            local rollCorr  = updatePID(rollRatePID, targetRollRate, rollRate, dt) * power * tpaFactor       

            for _, motor in ipairs(motors) do
                local speed =
                    throttle
                    - pitchCorr * motor.z
                    - rollCorr  * motor.x
                    - yawCorr   * motor.spin

                if not isValidNumber(speed) then
                    speed = 0
                end

                speed = clampInt(speed,0,256)

                rednet.send(
                    motor.id,
                    {
                        speed = speed,
                        tilt = motor.spin * 3
                    },
                    network
                )
            end

            term.clear()
            term.setCursorPos(1,1)

            print("=== DRONE STATE ===")
            print(string.format("dt: %.4f freq: %0.4f", dt, 1.0/dt))

            print("\n-- Orientation (radians) --")
            print(string.format("Pitch: %.3f", pitch))
            print(string.format("Roll : %.3f", roll))
            print(string.format("Yaw  : %.3f", yaw))

            print("\n-- Altitued --")
            print(string.format("X Y Z  : %.3f %.3f %.3f", pose.position.x, pose.position.y, pose.position.z))
            print(string.format("Target : %.3f %.3f %.3f", targetX, targetAlt, targetZ))
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

parallel.waitForAny( networkThread, flightThread )