term.clear()
term.setCursorPos(1,1)

--------------------------------------------------
-- modem
--------------------------------------------------

for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on "..side)
        break
    end
end

--------------------------------------------------
-- config
--------------------------------------------------

if not fs.exists("motor.txt") then
    error("Run motor_calibrate.lua first")
end

local file = fs.open("motor.txt","r")
local config = textutils.unserialize(file.readAll())
file.close()

--------------------------------------------------
-- peripherals
--------------------------------------------------

local controller =
    peripheral.find("Create_RotationSpeedController")

if not controller then
    error("No Rotation Speed Controller")
end

local gearshift =
    peripheral.find("Create_SequencedGearshift")

if not gearshift then
    error("No Sequenced Gearshift")
end

--------------------------------------------------
-- runtime
--------------------------------------------------

local timeout = 0.25

local currentTilt = 0

print("")
print("Motor Ready")

--------------------------------------------------
-- cleanup
--------------------------------------------------

local function cleanup()

    print("")
    print("Stopping motor...")

    --------------------------------------------------
    -- stop motor
    --------------------------------------------------

    pcall(function()
        controller.setTargetSpeed(0)
    end)

    --------------------------------------------------
    -- return tilt to zero
    --------------------------------------------------

    if currentTilt ~= 0 then

        local direction =
            currentTilt > 0 and 1 or -1

        pcall(function()

            if not gearshift.isRunning() then

                gearshift.rotate(
                    math.abs(currentTilt),
                    -direction
                )
            end
        end)

        currentTilt = 0
    end

    print("Safe shutdown complete")
end

--------------------------------------------------
-- main runtime
--------------------------------------------------

local function runtime()

    print("")
    print("Motor Ready")

    while true do

        local senderId, packet =
            rednet.receive(config.network, timeout)

        if type(packet) == "table" then

            local speed =
                tonumber(packet.speed) or 0

            local targetTilt =
                tonumber(packet.tilt) or 0

            --------------------------------------------------
            -- motor speed
            --------------------------------------------------

            controller.setTargetSpeed(speed)

            --------------------------------------------------
            -- tilt control
            --------------------------------------------------

            local delta =
                targetTilt - currentTilt

            --------------------------------------------------
            -- only move if needed
            --------------------------------------------------

            if math.abs(delta) >= 1
            and not gearshift.isRunning() then

                local direction =
                    delta > 0 and 1 or -1

                gearshift.rotate(
                    math.abs(delta),
                    direction
                )

                currentTilt =
                    targetTilt
            end

            --------------------------------------------------
            -- debug
            --------------------------------------------------

            term.clear()
            term.setCursorPos(1,1)

            print("=== MOTOR ===")
            print("")
            print("Speed :", speed)
            print("Tilt  :", currentTilt)

        else

            --------------------------------------------------
            -- failsafe
            --------------------------------------------------

            controller.setTargetSpeed(0)

            if currentTilt ~= 0
            and not gearshift.isRunning() then

                local direction =
                    currentTilt > 0 and 1 or -1

                gearshift.rotate(
                    math.abs(currentTilt),
                    -direction
                )

                currentTilt = 0
            end

            term.clear()
            term.setCursorPos(1,1)

            print("=== FAILSAFE ===")
            print("")
            print("SIGNAL LOST")
        end
    end
end

--------------------------------------------------
-- protected execution
--------------------------------------------------

local ok, err = xpcall(

    function()

        parallel.waitForAny(

            runtime,

            function()
                os.pullEvent("terminate")
                error("Terminated")
            end
        )
    end,

    debug.traceback
)

--------------------------------------------------
-- always cleanup
--------------------------------------------------

cleanup()

--------------------------------------------------
-- show error
--------------------------------------------------

if not ok then

    term.clear()
    term.setCursorPos(1,1)

    print("=== CRASH ===")
    print("")
    print(err)
end