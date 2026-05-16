term.clear()
term.setCursorPos(1,1)

--------------------------------------------------
-- open modem
--------------------------------------------------

for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on "..side)
        break
    end
end

--------------------------------------------------

print("")
write("Network Name: ")
local NETWORK = read()

print("")
write("Controller Id: ")
local CONTROLLER = read()

--------------------------------------------------

local motors = {}

print("")
print("Searching for motors...")

local timer = os.startTimer(2)

while true do

    local event, p1, p2, p3 = os.pullEvent()

    --------------------------------------------------
    -- broadcast setup
    --------------------------------------------------

    if event == "timer"
    and p1 == timer then

        rednet.broadcast({
            type = "setup_request",
            network = NETWORK
        },"setup")

        print("Broadcast sent")

        timer = os.startTimer(2)
    end

    --------------------------------------------------
    -- receive motor registration
    --------------------------------------------------

    if event == "rednet_message" then

        local senderId = p1
        local message = p2
        local protocol = p3

        if protocol == "setup"
        and type(message) == "table"
        and message.type == "register"
        and message.network == NETWORK then

            motors[#motors+1] = {
                id = senderId,
                x = message.offsetX,
                z = message.offsetZ,
                spin = message.spinDir
            }

            print("")
            print("Motor Registered")
            print("ID:", senderId)
            print("X:", message.offsetX)
            print("Z:", message.offsetZ)
            print("Spin:", message.spinDir)

            if #motors >= 4 then
                break
            end
        end
    end
end

--------------------------------------------------
-- save config
--------------------------------------------------

local file = fs.open("config.txt","w")

file.write(textutils.serialize({
    network = NETWORK,
    controller = CONTROLLER,
    motors = motors
}))

file.close()

print("")
print("Setup Complete!")