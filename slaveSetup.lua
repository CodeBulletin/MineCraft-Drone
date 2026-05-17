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

print("\nWaiting for Master to assign role...")

-- Start a timer to repeatedly ping the master until it answers
local pingTimer = os.startTimer(1)

while true do
    local event, p1, p2, p3 = os.pullEvent()

    --------------------------------------------------
    -- Broadcast readiness
    --------------------------------------------------
    if event == "timer" and p1 == pingTimer then
        rednet.broadcast({ type = "motor_ready" }, "setup_sys")
        pingTimer = os.startTimer(1)
    end

    --------------------------------------------------
    -- Receive Assignment
    --------------------------------------------------
    if event == "rednet_message" then
        local senderId = p1
        local message = p2
        local protocol = p3

        if protocol == "setup_sys" 
        and type(message) == "table" 
        and message.type == "assign_config" then
            
            -- Save the injected configuration
            local file = fs.open("motor.txt", "w")
            file.write(textutils.serialize({
                network = message.network,
                offsetX = message.offsetX,
                offsetZ = message.offsetZ,
                spinDir = message.spinDir
            }))
            file.close()

            -- Confirm receipt back to master
            rednet.send(senderId, { type = "motor_confirmed" }, "setup_sys")

            print("\n=== ROLE ASSIGNED ===")
            print("Network: " .. message.network)
            print("X Offset: " .. message.offsetX)
            print("Z Offset: " .. message.offsetZ)
            print("Spin Dir: " .. message.spinDir)
            print("\nMotor Setup Complete.")
            break
        end
    end
end