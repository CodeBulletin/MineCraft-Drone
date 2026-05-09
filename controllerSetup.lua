term.clear()
term.setCursorPos(1,1)

print("=== CONTROLLER SETUP ===")
print("")

--------------------------------------------------
-- Open Modem
--------------------------------------------------

local modemFound = false

for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on "..side)
        modemFound = true
        break
    end
end

if not modemFound then
    print("No modem found!")
    return
end

--------------------------------------------------
-- Ask Settings
--------------------------------------------------

print("")

write("Target Computer ID: ")
local targetID = tonumber(read())

write("Channel Name: ")
local channel = read()

--------------------------------------------------
-- Save Config
--------------------------------------------------

local file = fs.open("controller.txt","w")

file.write(textutils.serialize({
    targetID = targetID,
    channel = channel
}))

file.close()

--------------------------------------------------
-- Done
--------------------------------------------------

print("")
print("Controller Config Saved!")
print("")

print("TargetID: "..targetID)
print("Channel : "..channel)