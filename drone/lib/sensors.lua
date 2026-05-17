local utils = require("lib.utils")
local sensors = {}

local SMOOTHING = 0.8
local vxFiltered = 0    
local vzFiltered = 0

function sensors.read()
    local pose = sublevel.getLogicalPose()
    local vel = sublevel.getLinearVelocity()
    local ang = sublevel.getAngularVelocity()
    
    local pitch, yaw, roll = pose.orientation:toEuler()
    
    local vx = vel.x
    local vz = vel.z
    if math.abs(vx) < 0.02 then vx = 0 end
    if math.abs(vz) < 0.02 then vz = 0 end

    vxFiltered = utils.smooth(vx, vxFiltered, SMOOTHING)
    vzFiltered = utils.smooth(vz, vzFiltered, SMOOTHING)

    return {
        pose = pose,
        vel = vel,
        ang = ang,
        pitch = pitch,
        yaw = yaw,
        roll = roll,
        alt = pose.position.y,
        x = pose.position.x,
        z = pose.position.z,
        vy = vel.y,
        vx = vxFiltered,
        vz = vzFiltered,
        pitchRate = ang.x,
        yawRate = ang.y,
        rollRate = ang.z
    }
end

return sensors