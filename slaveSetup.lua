term.clear()
term.setCursorPos(1,1)

print("=== MOTOR CALIBRATION ===")

write("Network Name: ")
local network = read()

print("")
print("Coordinate System:")
print("+X = RIGHT")
print("-X = LEFT")
print("+Z = FRONT")
print("-Z = REAR")
print("")

write("Offset X: ")
local offsetX = tonumber(read())

write("Offset Z: ")
local offsetZ = tonumber(read())

print("")
print("Spin Direction:")
print("1  = CW")
print("-1 = CCW")

write("Spin Dir: ")
local spinDir = tonumber(read())

local file = fs.open("motor.txt","w")

file.write(textutils.serialize({
    network = network,
    offsetX = offsetX,
    offsetZ = offsetZ,
    spinDir = spinDir
}))

file.close()

print("")
print("Saved!")

--------------------------------------------------
-- setup registration
--------------------------------------------------

print("")
print("Waiting for setup...")

while true do

    local senderId, message, protocol =
        rednet.receive("setup")

    if type(message) == "table"
    and message.type == "setup_request"
    and message.network == config.network then

        rednet.send(senderId,{
            type = "register",
            network = config.network,
            offsetX = config.offsetX,
            offsetZ = config.offsetZ,
            spinDir = config.spinDir
        },"setup")

        print("Registered")
        break
    end
end