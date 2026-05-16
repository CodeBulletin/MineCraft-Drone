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
local ALIGN_VEL_TOLERANCE = 0.2     -- Max linear velocity (blocks/s) to be considered stable
local ALIGN_ANG_TOLERANCE = 0.1     -- Max angular velocity (rad/s) to be considered stable

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

local function normalizeAngle(a)
    while a > math.pi do a = a - 2*math.pi end
    while a < -math.pi do a = a + 2*math.pi end
    return a
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

local pathCrossPID = createPID(0.6, 0.0, 0.3)   -- lateral correction back to line
local pathAlongPID = createPID(0.6, 0.0, 0.3)   -- forward progress toward B

local control = { fb = 0, rl = 0, yc = 0, th = 0, manual = false }

--------------------------------------------------
-- Network Thread
--------------------------------------------------

local function networkThread()
    -- Proactive Handshake: Send an immediate ping to wake up a pre-running controller
    local pose = sublevel.getLogicalPose()
    local vel = sublevel.getLinearVelocity()
    local p, y, r = pose.orientation:toEuler()
    local bootPing = {
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
    }

    if controllerId then
        rednet.send(controllerId, bootPing, network)
    else
        -- Fallback: Broadcast if the master setup doesn't have a saved controller ID
        rednet.broadcast(bootPing, network)
    end

    while true do
        -- FIX: Listen on the dynamic 'network' variable instead of hardcoded "Comm1"
        local id, msg = rednet.receive(network)
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
                control.prevX = msg.prevX
                control.prevZ = msg.prevZ
                control.targetYaw = msg.targetYaw          -- <<< ADD
                control.useTargetYaw = msg.useTargetYaw    -- <<< ADD
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

    -- NEW: Variables to track path alignment state
    local alignState = false
    local activePathKey = ""

    -- Constants (no altitude numbers!)
    local RECOVERY_JUMP_THROTTLE = 240
    local RECOVERY_AIR_THROTTLE = 100
    local STUCK_TIME = 1.0        -- seconds before we declare "stuck"
    local JUMP_PULSE_TIME = 0.35  -- how long to fire jump throttle

    -- State variables
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
                    targetY = control.targetY   
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

                targetPitch = control.fb * 0.35
                targetRoll = -control.rl * 0.35
                targetX = x
                targetZ = z
            else
                if wasManual then
                    posXPID.integral = 0; posXPID.lastError = 0
                    posZPID.integral = 0; posZPID.lastError = 0
                    velXPID.integral = 0; velXPID.lastError = 0
                    velZPID.integral = 0; velZPID.lastError = 0
                    pathCrossPID.integral = 0; pathCrossPID.lastError = 0
                    pathAlongPID.integral = 0; pathAlongPID.lastError = 0
                    
                    activePathKey = "" -- Reset path state
                    wasManual = false
                end

                -- Pre-compute drone-local velocity for both branches
                local cosY = math.cos(yaw)
                local sinY = math.sin(yaw)
                local localVX = cosY * vx - sinY * vz
                local localVZ = sinY * vx + cosY * vz

                local velLimit, tiltLimit

                -- ============================================================
                -- AUTO PATH FOLLOWING
                -- ============================================================
                local inPathMode = control.hasTarget and control.prevX and control.prevZ
                local pathDegenerate = false

                if inPathMode then
                    local px = control.targetX - control.prevX
                    local pz = control.targetZ - control.prevZ
                    if px*px + pz*pz < 0.0001 then
                        pathDegenerate = true
                    end
                end

                if inPathMode and not pathDegenerate then
                    local Ax, Az = control.prevX, control.prevZ
                    local Bx, Bz = control.targetX, control.targetZ

                    local px = Bx - Ax
                    local pz = Bz - Az
                    local pathLen = math.sqrt(px*px + pz*pz)

                    -- Unit vectors of path frame (forward = A->B)
                    local ufx = px / pathLen
                    local ufz = pz / pathLen
                    local urx = ufz           -- right vector (90° clockwise)
                    local urz = -ufx

                    -- Drone position relative to A
                    local dx = x - Ax
                    local dz = z - Az

                    -- UNCLAMPED projection: t > 1 means overshoot past B
                    local proj = dx*ufx + dz*ufz
                    local t = proj / pathLen

                    -- Closest point on the A->B segment (clamped for cross-track only)
                    local closestT = clamp(t, 0, 1)
                    local cx = Ax + closestT*px
                    local cz = Az + closestT*pz

                    -- Signed cross-track error (+ = right of path, - = left)
                    local cross = (x - cx)*urx + (z - cz)*urz

                    -- Distance and horizontal velocity to endpoint B
                    local dxB = Bx - x
                    local dzB = Bz - z
                    local distToB = math.sqrt(dxB*dxB + dzB*dzB)
                    local hVel = math.sqrt(vx*vx + vz*vz)

                    -- Arrival tolerance
                    local ARRIVE_DIST = 1.0
                    local ARRIVE_VEL = 0.5
                    local isArrived = distToB < ARRIVE_DIST and hVel < ARRIVE_VEL

                    -- Dynamic deceleration profile (smoothstep)
                    local maxVel = 20.0
                    local minVel = 0.5
                    local approachFactor = clamp(distToB / 10.0, 0, 1)
                    approachFactor = approachFactor * approachFactor * (3 - 2 * approachFactor)
                    velLimit = minVel + approachFactor * (maxVel - minVel)
                    tiltLimit = 0.08 + approachFactor * (0.4 - 0.08)

                    -- True geometric path direction (locked for entire segment)
                    local pathYaw = math.atan2(px, pz)

                    -- NEW: Determine what heading target we are actively pointing toward
                    local desiredHeading = pathYaw
                    if control.useTargetYaw and control.targetYaw then
                        desiredHeading = control.targetYaw
                    end

                    -- Smoothly slew targetYaw toward our active desiredHeading target
                    local yawError = angleDiff(desiredHeading, targetYaw)
                    local YAW_SLEW_RATE = 3.0
                    local maxYawStep = YAW_SLEW_RATE * dt
                    if math.abs(yawError) > maxYawStep then
                        yawError = yawError > 0 and maxYawStep or -maxYawStep
                    end
                    targetYaw = normalizeAngle(targetYaw + yawError)


                    --------------------------------------------------
                    -- ALIGNMENT CHECK
                    --------------------------------------------------
                    -- Identify if this is a new path we haven't aligned to yet
                    local currentPathKey = tostring(Bx) .. "_" .. tostring(Bz)
                    if activePathKey ~= currentPathKey then
                        alignState = true
                        activePathKey = currentPathKey
                    end

                    -- Calculate real-world alignment errors against our active target heading
                    local yawErrorActual = math.abs(angleDiff(desiredHeading, yaw))
                    local altErrorActual = math.abs(targetY - alt)

                    -- Calculate velocity magnitudes to ensure mechanical stabilization
                    -- (Using the filtered vx and vz you already computed above)
                    local linearVelMag = math.sqrt(vx*vx + vy*vy + vz*vz)
                    local angularVelMag = math.sqrt(pitchRate*pitchRate + yawRate*yawRate + rollRate*rollRate)

                    -- Exit alignment state only when position, heading, drift, and rotation are stable
                    if alignState 
                    and yawErrorActual < 0.15 
                    and altErrorActual < 1 
                    and linearVelMag < ALIGN_VEL_TOLERANCE 
                    and angularVelMag < ALIGN_ANG_TOLERANCE then
                        
                        alignState = false
                    end

                    --------------------------------------------------
                    -- STATE MACHINE
                    --------------------------------------------------
                    if isArrived then
                        -- --------------------------------------------------
                        -- ARRIVED: hold position at B using position PIDs
                        -- --------------------------------------------------
                        pathCrossPID.integral = 0; pathCrossPID.lastError = 0
                        pathAlongPID.integral = 0; pathAlongPID.lastError = 0

                        targetX = Bx
                        targetZ = Bz

                        local pdx = targetX - x
                        local pdz = targetZ - z
                        local localDX = cosY * pdx - sinY * pdz
                        local localDZ = sinY * pdx + cosY * pdz

                        local targetVX = updatePID(posXPID, 0, -localDX, dt)
                        local targetVZ = updatePID(posZPID, 0, -localDZ, dt)

                        local holdLimit = 0.5
                        targetVX = clamp(targetVX, -holdLimit, holdLimit)
                        targetVZ = clamp(targetVZ, -holdLimit, holdLimit)

                        if math.abs(targetVX) < 0.2 then targetVX = 0 end
                        if math.abs(targetVZ) < 0.2 then targetVZ = 0 end

                        targetPitch = clamp(updatePID(velZPID, targetVZ, localVZ, dt), -tiltLimit, tiltLimit)
                        targetRoll  = clamp(-updatePID(velXPID, targetVX, localVX, dt), -tiltLimit, tiltLimit)

                    elseif alignState then
                        -- --------------------------------------------------
                        -- ALIGNING: hold at start point A until yaw/alt match
                        -- --------------------------------------------------
                        pathCrossPID.integral = 0; pathCrossPID.lastError = 0
                        pathAlongPID.integral = 0; pathAlongPID.lastError = 0

                        targetX = Ax
                        targetZ = Az

                        local pdx = targetX - x
                        local pdz = targetZ - z
                        local localDX = cosY * pdx - sinY * pdz
                        local localDZ = sinY * pdx + cosY * pdz

                        local targetVX = updatePID(posXPID, 0, -localDX, dt)
                        local targetVZ = updatePID(posZPID, 0, -localDZ, dt)

                        local holdLimit = 1.0
                        targetVX = clamp(targetVX, -holdLimit, holdLimit)
                        targetVZ = clamp(targetVZ, -holdLimit, holdLimit)

                        if math.abs(targetVX) < 0.1 then targetVX = 0 end
                        if math.abs(targetVZ) < 0.1 then targetVZ = 0 end

                        targetPitch = clamp(updatePID(velZPID, targetVZ, localVZ, dt), -tiltLimit, tiltLimit)
                        targetRoll  = clamp(-updatePID(velXPID, targetVX, localVX, dt), -tiltLimit, tiltLimit)
                        
                    else
                        -- --------------------------------------------------
                        -- EN ROUTE: follow path with overshoot correction
                        -- --------------------------------------------------
                        local errAlong = pathLen - proj

                        if t > 1.0 and pathAlongPID.integral > 0 then
                            pathAlongPID.integral = 0
                        end

                        local vPathRight   = updatePID(pathCrossPID, 0, cross, dt)
                        local vPathForward = updatePID(pathAlongPID, 0, -errAlong, dt)

                        vPathRight   = clamp(vPathRight, -velLimit, velLimit)
                        vPathForward = clamp(vPathForward, -velLimit, velLimit)

                        local VEL_DEADZONE = 0.25
                        if math.abs(vPathForward) < VEL_DEADZONE then vPathForward = 0 end
                        if math.abs(vPathRight) < VEL_DEADZONE then vPathRight = 0 end

                        local yawDiff = yaw - pathYaw
                        local cDiff = math.cos(yawDiff)
                        local sDiff = math.sin(yawDiff)

                        local targetVX = cDiff * vPathRight - sDiff * vPathForward
                        local targetVZ = sDiff * vPathRight + cDiff * vPathForward

                        targetPitch = clamp(updatePID(velZPID, targetVZ, localVZ, dt), -tiltLimit, tiltLimit)
                        targetRoll  = clamp(-updatePID(velXPID, targetVX, localVX, dt), -tiltLimit, tiltLimit)
                    end

                else
                    -- ============================================================
                    -- NORMAL POSITION HOLD (manual or no valid path segment)
                    -- ============================================================
                    if control.hasTarget then
                        targetX = control.targetX
                        targetZ = control.targetZ
                        if control.targetY then
                            targetY = control.targetY
                        end
                    else
                        activePathKey = "" -- reset path state memory
                        
                        -- 5-block soft reset
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

                    local dx = targetX - x
                    local dz = targetZ - z
                    local localDX = cosY * dx - sinY * dz
                    local localDZ = sinY * dx + cosY * dz

                    local targetVX = updatePID(posXPID, 0, -localDX, dt)
                    local targetVZ = updatePID(posZPID, 0, -localDZ, dt)

                    local distToTarget = math.sqrt(dx*dx + dz*dz)
                    if distToTarget > 10 then
                        velLimit = 20.0
                        tiltLimit = 0.4
                    else
                        velLimit = 3.0
                        tiltLimit = 0.15
                    end

                    targetVX = clamp(targetVX, -velLimit, velLimit)
                    targetVZ = clamp(targetVZ, -velLimit, velLimit)

                    if math.abs(targetVX) < 0.2 then targetVX = 0 end
                    if math.abs(targetVZ) < 0.2 then targetVZ = 0 end

                    targetPitch = clamp(updatePID(velZPID, targetVZ, localVZ, dt), -tiltLimit, tiltLimit)
                    targetRoll  = clamp(-updatePID(velXPID, targetVX, localVX, dt), -tiltLimit, tiltLimit)
                end
            end
            
            -- Yaw stabilization    
            local targetYawRate
            local yawStickActive = math.abs(control.yc) > 0.4

            if yawStickActive then
                -- Manual rate control always overrides everything
                targetYawRate = control.yc * 4.0
                targetYaw = yaw
                wasYawing = true
            else
                if wasYawing then
                    -- Stick just released
                    if not (control.hasTarget and control.prevX and control.prevZ) then
                        -- Only snap to current heading if NOT in auto path mode
                        targetYaw = yaw
                    end
                    wasYawing = false
                    yawPID.integral = 0; yawPID.lastError = 0
                    yawRatePID.integral = 0; yawRatePID.lastError = 0
                end

                local yawError = angleDiff(targetYaw, yaw)
                local YAW_DEADBAND = 0.03  -- ~1.7 degrees
                if math.abs(yawError) < YAW_DEADBAND then
                    yawError = 0
                end
                targetYawRate = updatePID(yawPID, yawError, 0, dt)
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
            local vyNearlyZero = math.abs(vy) < 0.8
            local isStuck = recoveryMode and (recoveryTimer > STUCK_TIME) and vyNearlyZero

            -- JUMP PULSE:
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
                targetX = x
                targetZ = z

                altPID.integral = 0; altPID.lastError = 0
                velPID.integral = 0; velPID.lastError = 0
                posXPID.integral = 0; posXPID.lastError = 0
                posZPID.integral = 0; posZPID.lastError = 0
                velXPID.integral = 0; velXPID.lastError = 0
                velZPID.integral = 0; velZPID.lastError = 0

                if isJumping then
                    throttle = RECOVERY_JUMP_THROTTLE
                    multiplyer = 0.05
                    invert = -1.0
                else
                    throttle = RECOVERY_AIR_THROTTLE
                end
            else
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
                local offset = - pitchCorr * motor.z
                               - rollCorr  * motor.x
                               - yawCorr   * motor.spin
                
                rawSpeeds[i] = offset
                sumOffset = sumOffset + offset
            end

            local avgOffset = sumOffset / #motors
            
            for i=1, #rawSpeeds do
                rawSpeeds[i] = throttle + rawSpeeds[i] - avgOffset
                
                if rawSpeeds[i] < minSpeed then minSpeed = rawSpeeds[i] end
                if rawSpeeds[i] > maxSpeed then maxSpeed = rawSpeeds[i] end
            end

            for i, motor in ipairs(motors) do
                local finalSpeed = clampInt(invert * rawSpeeds[i], -255, 255)
                
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
            print(string.format("dt: %.4f freq: %0.4f ver: 1.5", dt, 1.0/dt))

            print("\n-- Orientation (radians) --")
            print(string.format("Pitch: %.3f | %.3f", pitch, targetPitch))
            print(string.format("Roll : %.3f | %.3f", roll, targetRoll))
            print(string.format("Yaw  : %.3f | %.3f", yaw, targetYaw))

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
                "Manual  : %s | Aligning %s",
                tostring(control.manual),
                tostring(alignState)
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