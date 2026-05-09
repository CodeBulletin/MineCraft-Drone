--------------------------------------------------
-- Network
--------------------------------------------------

rednet.open("top")

local redstonelink = peripheral.wrap("back")

local TARGET_ID = 11
local CHANNEL = "Comm1"

--------------------------------------------------
-- Settings
--------------------------------------------------

local UPDATE_RATE = 0.02

local DEADZONE = 0.08
local EXPO = 0.45
local SMOOTH = 0.22

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

    return sign *
        ((math.abs(v) - dz) / (1 - dz))
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

    local f =
        redstonelink.getLinkSignal(
            "minecraft:red_wool",
            "minecraft:red_wool"
        )

    local b =
        redstonelink.getLinkSignal(
            "minecraft:orange_wool",
            "minecraft:orange_wool"
        )

    local r =
        redstonelink.getLinkSignal(
            "minecraft:light_gray_wool",
            "minecraft:light_gray_wool"
        )

    local l =
        redstonelink.getLinkSignal(
            "minecraft:gray_wool",
            "minecraft:gray_wool"
        )

    local u =
        redstonelink.getLinkSignal(
            "minecraft:lime_wool",
            "minecraft:lime_wool"
        )

    local d =
        redstonelink.getLinkSignal(
            "minecraft:green_wool",
            "minecraft:green_wool"
        )

    local yr =
        redstonelink.getLinkSignal(
            "minecraft:cyan_wool",
            "minecraft:cyan_wool"
        )

    local yl =
        redstonelink.getLinkSignal(
            "minecraft:blue_wool",
            "minecraft:blue_wool"
        )

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

    --------------------------------------------------
    -- Deadzone
    --------------------------------------------------

    out.fb = deadzone(raw.fb, DEADZONE)
    out.rl = deadzone(raw.rl, DEADZONE)
    out.yc = deadzone(raw.yc, DEADZONE)
    out.th = deadzone(raw.th, DEADZONE)

    --------------------------------------------------
    -- Expo
    --------------------------------------------------

    out.fb = expo(out.fb, EXPO)
    out.rl = expo(out.rl, EXPO)
    out.yc = expo(out.yc, EXPO)

    --------------------------------------------------
    -- Smoothing
    --------------------------------------------------

    filtered.fb =
        smooth(out.fb, filtered.fb, SMOOTH)

    filtered.rl =
        smooth(out.rl, filtered.rl, SMOOTH)

    filtered.yc =
        smooth(out.yc, filtered.yc, SMOOTH)

    filtered.th =
        smooth(out.th, filtered.th, SMOOTH)

    --------------------------------------------------
    -- Clamp
    --------------------------------------------------

    out.fb = clamp(filtered.fb, -1, 1)
    out.rl = clamp(filtered.rl, -1, 1)
    out.yc = clamp(filtered.yc, -1, 1)
    out.th = clamp(filtered.th, -1, 1)

    --------------------------------------------------
    -- Hover Assist
    --------------------------------------------------

    out.manual =
        math.abs(out.fb) > 0.05 or
        math.abs(out.rl) > 0.05

    return out
end

--------------------------------------------------
-- TX Thread
--------------------------------------------------

local function txThread()

    while true do

        local raw = getRawInput()
        local input = processInput(raw)

        local now = os.clock()

        --------------------------------------------------
        -- Send on change or heartbeat
        --------------------------------------------------

        if changed(input, lastSent)
        or now - lastHeartbeat > HEARTBEAT_TIME then

            rednet.send(
                TARGET_ID,
                {
                    type = "control",

                    fb = input.fb,
                    rl = input.rl,
                    yc = input.yc,
                    th = input.th,

                    manual = input.manual,

                    timestamp = now
                },
                CHANNEL
            )

            lastSent = {
                fb = input.fb,
                rl = input.rl,
                yc = input.yc,
                th = input.th
            }

            lastHeartbeat = now
        end

        --------------------------------------------------
        -- UI
        --------------------------------------------------

        term.setCursorPos(1,1)
        term.clear()

        print("=== DRONE CONTROLLER ===")

        print("")

        print(string.format(
            "Forward : %+0.2f",
            input.fb
        ))

        print(string.format(
            "Right   : %+0.2f",
            input.rl
        ))

        print(string.format(
            "Yaw     : %+0.2f",
            input.yc
        ))

        print(string.format(
            "Throttle: %+0.2f",
            input.th
        ))

        print("")

        if input.manual then
            print("MODE: MANUAL")
        else
            print("MODE: POSITION HOLD")
        end

        sleep(UPDATE_RATE)
    end
end

--------------------------------------------------
-- Start
--------------------------------------------------

parallel.waitForAny(
    txThread
)