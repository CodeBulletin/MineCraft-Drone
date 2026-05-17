local pid = require("lib.pid")
local utils = require("lib.utils")
local enroute = {}

function enroute.update(ctx, cross, errAlong, t, pathYaw)
    if t > 1.0 and ctx.pathAlongPID.integral > 0 then
        ctx.pathAlongPID.integral = 0
    end

    local vPathRight   = pid.update(ctx.pathCrossPID, 0, cross, ctx.dt)
    local vPathForward = pid.update(ctx.pathAlongPID, 0, -errAlong, ctx.dt)

    vPathRight   = utils.clamp(vPathRight, -ctx.velLimit, ctx.velLimit)
    vPathForward = utils.clamp(vPathForward, -ctx.velLimit, ctx.velLimit)

    local VEL_DEADZONE = 0.25
    if math.abs(vPathForward) < VEL_DEADZONE then vPathForward = 0 end
    if math.abs(vPathRight) < VEL_DEADZONE then vPathRight = 0 end

    local yawDiff = ctx.yaw - pathYaw
    local cDiff = math.cos(yawDiff)
    local sDiff = math.sin(yawDiff)

    local targetVX = cDiff * vPathRight - sDiff * vPathForward
    local targetVZ = sDiff * vPathRight + cDiff * vPathForward

    ctx.targetPitch = utils.clamp(pid.update(ctx.velZPID, targetVZ, ctx.localVZ, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
    ctx.targetRoll  = utils.clamp(-pid.update(ctx.velXPID, targetVX, ctx.localVX, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
end

return enroute