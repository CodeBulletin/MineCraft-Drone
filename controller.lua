--------------------------------------------------
-- Network
--------------------------------------------------

for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on " .. side)
        break
    end
end

local redstonelink = peripheral.find("redstone_link_bridge")  
assert(redstonelink, "No redstone_link_bridge found")  

--------------------------------------------------
-- Config
--------------------------------------------------

if not fs.exists("controller.txt") then
    error("Run controllerSetup.lua first")
end

local file = fs.open("controller.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

local TARGET_ID = config.targetID
local CHANNEL = config.channel

--------------------------------------------------
-- Waypoints / Auto
--------------------------------------------------

local waypoints = {}
local autoMode = false
local autoIndex = 1
local autoState = "idle"   -- "idle", "moving", "waiting", "paused"
local autoWaitTimer = 0
local manualPauseTimer = 0
local lastManual = false
local arriveTimer = 0
local droneTelemetry = nil
local lastTelemetryTime = 0

local AUTO_RESUME_DELAY = 2.0
local WAYPOINT_ARRIVE_DIST = 1.0
local WAYPOINT_ARRIVE_VEL = 0.5
local ARRIVE_SUSTAIN = 0.5

local function loadWaypoints()
    if not fs.exists("waypoints.txt") then return false end
    local f = fs.open("waypoints.txt", "r")
    waypoints = {}
    while true do
        local line = f.readLine()
        if not line then break end
        line = line:gsub("%s+", "")
        if line ~= "" and not line:find("^%-%-") then
            local xStr, zStr, yStr = line:match("([^,]+),([^,]+),([^,]+)")
            if xStr and zStr then
                local wx, wz, wy = tonumber(xStr), tonumber(zStr), tonumber(yStr) 
                if wx and wz then
                    table.insert(waypoints, {x = wx, z = wz, y = wy})
                end
            end
        end
    end
    f.close()
    print("Loaded " .. #waypoints .. " waypoints")
    return #waypoints > 0
end

loadWaypoints()

if fs.exists("auto.txt") then
    local f = fs.open("auto.txt", "r")
    if f then
        local content = f.readAll():gsub("%s+", "")
        f.close()
        if content == "true" and #waypoints > 0 then
            autoMode = true
            autoState = "moving"
            print("Auto enabled on boot")
        end
    end
end

--------------------------------------------------
-- Settings
--------------------------------------------------

local UPDATE_RATE = 0.02

local DEADZONE = 0.08
local EXPO = 0.45
local SMOOTH = 0.45

local HEARTBEAT_TIME = 1.0

--------------------------------------------------
-- State
--------------------------------------------------

local filtered = {
    fb = 0,
    rl = 0,
    yc = 0,
    th = 0
}

local lastSent = nil
local lastHeartbeat = 0
local txCount = 0
local rxCount = 0
local lastRaw = { f=0, b=0, r=0, l=0, u=0, d=0, yr=0, yl=0 }

--------------------------------------------------
-- Helpers
--------------------------------------------------

local function norm(v)
    return v / 15
end

local function clamp(v, mn, mx)
    return math.max(mn, math.min(mx, v))
end

local function deadzone(v, dz)
    if math.abs(v) < dz then
        return 0
    end
    local sign = v > 0 and 1 or -1
    return sign * ((math.abs(v) - dz) / (1 - dz))
end

local function expo(v, e)
    return (1 - e) * v + e * (v * v * v)
end

local function smooth(new, old, factor)
    return old + (new - old) * factor
end

local function changed(a, b)
    if not a or not b then
        return true
    end
    return
        math.abs(a.fb - b.fb) > 0.01 or
        math.abs(a.rl - b.rl) > 0.01 or
        math.abs(a.yc - b.yc) > 0.01 or
        math.abs(a.th - b.th) > 0.01
end

--------------------------------------------------
-- Read Raw Input
--------------------------------------------------

local function getRawInput()
    local f = redstonelink.getLinkSignal("minecraft:red_wool", "minecraft:red_wool")
    local b = redstonelink.getLinkSignal("minecraft:orange_wool", "minecraft:orange_wool")
    local r = redstonelink.getLinkSignal("minecraft:light_gray_wool", "minecraft:light_gray_wool")
    local l = redstonelink.getLinkSignal("minecraft:gray_wool", "minecraft:gray_wool")
    local u = redstonelink.getLinkSignal("minecraft:lime_wool", "minecraft:lime_wool")
    local d = redstonelink.getLinkSignal("minecraft:green_wool", "minecraft:green_wool")
    local yr = redstonelink.getLinkSignal("minecraft:cyan_wool", "minecraft:cyan_wool")
    local yl = redstonelink.getLinkSignal("minecraft:blue_wool", "minecraft:blue_wool")

    lastRaw.f = f; lastRaw.b = b; lastRaw.r = r; lastRaw.l = l
    lastRaw.u = u; lastRaw.d = d; lastRaw.yr = yr; lastRaw.yl = yl

    return {
        fb = norm(f)  - norm(b),
        rl = norm(r)  - norm(l),
        yc = norm(yr) - norm(yl),
        th = norm(u)  - norm(d)
    }
end

--------------------------------------------------
-- Process Input
--------------------------------------------------

local function processInput(raw)
    local out = {}

    out.fb = deadzone(raw.fb, DEADZONE)
    out.rl = deadzone(raw.rl, DEADZONE)
    out.yc = deadzone(raw.yc, DEADZONE + 0.02)
    out.th = deadzone(raw.th, DEADZONE + 0.02)

    out.fb = expo(out.fb, EXPO)
    out.rl = expo(out.rl, EXPO)
    out.yc = expo(out.yc, EXPO)

    filtered.fb = smooth(out.fb, filtered.fb, SMOOTH)
    filtered.rl = smooth(out.rl, filtered.rl, SMOOTH)
    filtered.yc = smooth(out.yc, filtered.yc, SMOOTH)
    filtered.th = smooth(out.th, filtered.th, SMOOTH)

    out.fb = clamp(filtered.fb, -1, 1)
    out.rl = clamp(filtered.rl, -1, 1)
    out.yc = clamp(filtered.yc, -1, 1)
    out.th = clamp(filtered.th, -1, 1)

    if math.abs(out.yc) < 0.02 then out.yc = 0 end
    if math.abs(out.th) < 0.02 then out.th = 0 end

    out.manual = math.abs(out.fb) > 0.1 or math.abs(out.rl) > 0.1
    return out
end

--------------------------------------------------
-- RX Thread (telemetry from drone)
--------------------------------------------------

local function rxThread()
    while true do
        local id, msg = rednet.receive(CHANNEL)
        if msg and msg.type == "telemetry" and id == TARGET_ID then
            droneTelemetry = msg
            lastTelemetryTime = os.clock()
            rxCount = rxCount + 1
        end
    end
end

--------------------------------------------------
-- Debug UI
--------------------------------------------------

local function drawDebugUI(raw, input, hasTarget, targetX, targetZ, targetY, now, dt)
    term.clear()
    local w, h = term.getSize()
    local col = math.floor(w / 2) + 1

    -- Header
    term.setCursorPos(1, 1)
    local conn = "NO CONN"
    if droneTelemetry then
        local age = now - lastTelemetryTime
        if age < 1.0 then conn = "CONN OK"
        elseif age < 3.0 then conn = "SLOW"
        else conn = "STALE" end
    end
    local title = "=== DRONE CONTROLLER ==="
    term.write(title .. string.rep(" ", w - #title - #conn - 2) .. "[" .. conn .. "]")

    -- Separator
    term.setCursorPos(1, 2)
    term.write(string.rep("-", w))

    -- Column divider
    for y = 3, h do
        term.setCursorPos(col, y)
        term.write("|")
    end

    -- LEFT COLUMN
    local y = 3

    term.setCursorPos(1, y); term.write("-- INPUTS --"); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("FB  raw:%+5.2f (%2d|%2d)", raw.fb, lastRaw.f, lastRaw.b)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("    out:%+5.2f  man:%s", input.fb, tostring(input.manual))); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("RL  raw:%+5.2f (%2d|%2d)", raw.rl, lastRaw.r, lastRaw.l)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("    out:%+5.2f", input.rl)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("YC  raw:%+5.2f (%2d|%2d)", raw.yc, lastRaw.yr, lastRaw.yl)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("    out:%+5.2f", input.yc)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("TH  raw:%+5.2f (%2d|%2d)", raw.th, lastRaw.u, lastRaw.d)); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("    out:%+5.2f", input.th)); y = y + 1

    y = y + 1
    term.setCursorPos(1, y); term.write("-- SETTINGS --"); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("DZ:%.2f EX:%.2f SM:%.2f", DEADZONE, EXPO, SMOOTH)); y = y + 1

    y = y + 1
    term.setCursorPos(1, y); term.write("-- MODE --"); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("MANUAL: %s", tostring(input.manual))); y = y + 1
    term.setCursorPos(1, y); term.write(string.format("AUTO:   %s %s", tostring(autoMode), autoState)); y = y + 1
    if hasTarget then
        term.setCursorPos(1, y); term.write(string.format("TGT: %.1f, %.1f %.1f", targetX, targetZ, targetY)); y = y + 1
    end

    -- RIGHT COLUMN
    y = 3

    term.setCursorPos(col + 2, y); term.write("-- DRONE TELEMETRY --"); y = y + 1
    if droneTelemetry then
        term.setCursorPos(col + 2, y); term.write(string.format("POS:  %6.1f  %6.1f", droneTelemetry.x, droneTelemetry.z)); y = y + 1
        term.setCursorPos(col + 2, y); term.write(string.format("ALT:  %6.1f", droneTelemetry.y)); y = y + 1
        term.setCursorPos(col + 2, y); term.write(string.format("VEL:  %5.1f %5.1f %5.1f", droneTelemetry.vx, droneTelemetry.vy, droneTelemetry.vz)); y = y + 1
        term.setCursorPos(col + 2, y); term.write(string.format("YAW:  %6.3f", droneTelemetry.yaw)); y = y + 1
        term.setCursorPos(col + 2, y); term.write(string.format("PITCH:%6.3f  ROLL:%6.3f", droneTelemetry.pitch, droneTelemetry.roll)); y = y + 1
        term.setCursorPos(col + 2, y); term.write(string.format("AGE:  %.2fs", now - lastTelemetryTime)); y = y + 1
    else
        term.setCursorPos(col + 2, y); term.write("NO TELEMETRY"); y = y + 1
        term.setCursorPos(col + 2, y); term.write("Waiting for drone..."); y = y + 1
    end

    y = y + 1
    term.setCursorPos(col + 2, y); term.write("-- AUTO NAV --"); y = y + 1
    if #waypoints > 0 then
        term.setCursorPos(col + 2, y); term.write(string.format("WP:   %d / %d", autoIndex, #waypoints)); y = y + 1
        if waypoints[autoIndex] then
            local wp = waypoints[autoIndex]
            term.setCursorPos(col + 2, y); term.write(string.format("CUR:  %.1f, %.1f", wp.x, wp.z)); y = y + 1
            if droneTelemetry and autoState == "moving" then
                local dx = wp.x - droneTelemetry.x
                local dz = wp.z - droneTelemetry.z
                local dist = math.sqrt(dx*dx + dz*dz)
                term.setCursorPos(col + 2, y); term.write(string.format("DIST: %.2f", dist)); y = y + 1
            end
        end
        if autoState == "waiting" then
            term.setCursorPos(col + 2, y); term.write(string.format("WAIT: %.1f s", autoWaitTimer)); y = y + 1
        elseif autoState == "moving" then
            term.setCursorPos(col + 2, y); term.write(string.format("ARR:  %.2f / %.1f", arriveTimer, ARRIVE_SUSTAIN)); y = y + 1
        end
    else
        term.setCursorPos(col + 2, y); term.write("NO WAYPOINTS"); y = y + 1
    end

    y = y + 1
    term.setCursorPos(col + 2, y); term.write("-- NETWORK --"); y = y + 1
    term.setCursorPos(col + 2, y); term.write(string.format("TX:   %d pkts", txCount)); y = y + 1
    term.setCursorPos(col + 2, y); term.write(string.format("RX:   %d pkts", rxCount)); y = y + 1
    term.setCursorPos(col + 2, y); term.write(string.format("RATE: %.3fs", dt)); y = y + 1
end

--------------------------------------------------
-- TX Thread
--------------------------------------------------

local function txThread()
    local lastTime = os.clock()   -- <<< ADD THIS
    
    while true do
        local raw = getRawInput()
        local input = processInput(raw)
        local now = os.clock()
        
        -- <<< ADD THIS BLOCK
        local dt = now - lastTime
        if dt <= 0 then dt = 0.001 end
        lastTime = now

        --------------------------------------------------
        -- Auto Navigation State Machine
        --------------------------------------------------
        local manualActive = input.manual
        local hasTarget = false
        local targetX, targetZ, targetY

        if manualActive then
            manualPauseTimer = 0
            if autoMode and autoState ~= "paused" then
                autoState = "paused"
            end
        else
            if lastManual then
                manualPauseTimer = 0
            end
            manualPauseTimer = manualPauseTimer + dt
        end
        lastManual = manualActive

        if autoMode and autoState == "paused" and manualPauseTimer >= AUTO_RESUME_DELAY then
            autoState = "moving"
        end

        if autoMode and not manualActive then
            if autoState == "moving" then
                local wp = waypoints[autoIndex]
                hasTarget = true
                targetX = wp.x
                targetZ = wp.z
                targetY = wp.y or (droneTelemetry and droneTelemetry.y or 80)

                if droneTelemetry then
                    local dx = wp.x - droneTelemetry.x
                    local dz = wp.z - droneTelemetry.z
                    local dist = math.sqrt(dx*dx + dz*dz)
                    local hVel = math.sqrt(droneTelemetry.vx^2 + droneTelemetry.vz^2)

                    if dist < WAYPOINT_ARRIVE_DIST and hVel < WAYPOINT_ARRIVE_VEL then
                        arriveTimer = arriveTimer + dt
                        if arriveTimer >= ARRIVE_SUSTAIN then
                            autoState = "waiting"
                            autoWaitTimer = 30.0
                            arriveTimer = 0
                        end
                    else
                        arriveTimer = 0
                    end
                end

            elseif autoState == "waiting" then
                local wp = waypoints[autoIndex]
                hasTarget = true
                targetX = wp.x
                targetZ = wp.z
                targetY = wp.y or targetY  -- <<< ADD (preserve last known)

                autoWaitTimer = autoWaitTimer - dt
                if autoWaitTimer <= 0 then
                    autoIndex = autoIndex + 1
                    if autoIndex > #waypoints then
                        autoIndex = 1
                    end
                    autoState = "moving"
                    autoWaitTimer = 0
                end
            end
        end

        --------------------------------------------------
        -- Build & Send Packet
        --------------------------------------------------
        local packet = {
            type = "control",
            fb = input.fb,
            rl = input.rl,
            yc = input.yc,
            th = input.th,
            manual = input.manual,
            hasTarget = hasTarget,
            timestamp = now
        }
        if hasTarget then
            packet.targetX = targetX
            packet.targetZ = targetZ
            packet.targetY = targetY   -- <<< ADD
        end

        local targetChanged = hasTarget ~= (lastSent and lastSent.hasTarget or false)
            or (hasTarget and lastSent and (targetX ~= lastSent.targetX or targetZ ~= lastSent.targetZ))

        if changed(input, lastSent) or now - lastHeartbeat > HEARTBEAT_TIME or targetChanged then
            rednet.send(TARGET_ID, packet, CHANNEL)
            txCount = txCount + 1
            lastSent = {
                fb = input.fb,
                rl = input.rl,
                yc = input.yc,
                th = input.th,
                hasTarget = hasTarget,
                targetX = targetX,
                targetZ = targetZ
            }
            lastHeartbeat = now
        end

        --------------------------------------------------
        -- Debug UI
        --------------------------------------------------
        drawDebugUI(raw, input, hasTarget, targetX, targetZ, targetY, now, dt)

        sleep(UPDATE_RATE)
    end
end

--------------------------------------------------
-- Start
--------------------------------------------------

parallel.waitForAny(txThread, rxThread)