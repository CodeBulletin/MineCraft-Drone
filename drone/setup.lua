term.clear()
term.setCursorPos(1,1)

-- Open modem
for _, side in ipairs(peripheral.getNames()) do
    if peripheral.getType(side) == "modem" then
        rednet.open(side)
        print("Opened modem on " .. side)
        break
    end
end

print("\n=== DRONE MASTER SETUP ===")
write("Network Name [drone_net]: ")
local NETWORK = read()
if NETWORK == "" then NETWORK = "drone_net" end

write("Controller Id: ")
local CONTROLLER = tonumber(read())

print("\nDrone Dimensions:")
write("Motor X Offset (Left/Right distance): ")
local distX = math.abs(tonumber(read()) or 1)

write("Motor Z Offset (Front/Rear distance): ")
local distZ = math.abs(tonumber(read()) or 1)

-- Standard Quad-X layout configuration
local layout = {
    { name = "FRONT-RIGHT", x = distX,  z = distZ,  spin = -1 },
    { name = "FRONT-LEFT",  x = -distX, z = distZ,  spin = 1  },
    { name = "REAR-LEFT",   x = -distX, z = -distZ, spin = -1 },
    { name = "REAR-RIGHT",  x = distX,  z = -distZ, spin = 1  }
}

local motors = {}

for i, target in ipairs(layout) do
    term.clear()
    term.setCursorPos(1,1)
    print("=== PAIRING MOTORS ===")
    print("Progress: " .. #motors .. "/4 motors registered")
    print("-----------------------------------")
    print("ACTION REQUIRED:")
    print("Go to the " .. target.name .. " motor")
    print("and run the motor setup script.")
    print("-----------------------------------")
    print("Waiting on channel 'setup_sys'...")

    while true do
        local senderId, message, protocol = rednet.receive("setup_sys")

        if type(message) == "table" and message.type == "motor_ready" then
            
            -- Inject the configuration to this specific motor
            rednet.send(senderId, {
                type = "assign_config",
                network = NETWORK,
                offsetX = target.x,
                offsetZ = target.z,
                spinDir = target.spin
            }, "setup_sys")

            -- Wait for the motor to confirm it saved the data
            local cId, cMsg, cProto = rednet.receive("setup_sys", 2)
            if type(cMsg) == "table" and cMsg.type == "motor_confirmed" then
                motors[#motors+1] = {
                    id = senderId,
                    x = target.x,
                    z = target.z,
                    spin = target.spin
                }
                print("\nSuccess! Registered " .. target.name .. " (ID: " .. senderId .. ")")
                sleep(1.5)
                break
            end
        end
    end
end

--------------------------------------------------
-- Save Master Config
--------------------------------------------------
local file = fs.open("config.txt", "w")
file.write(textutils.serialize({
    network = NETWORK,
    controller = CONTROLLER,
    motors = motors
}))
file.close()

term.clear()
term.setCursorPos(1,1)
print("=== SETUP COMPLETE ===")
print("All 4 motors paired and configured successfully.")