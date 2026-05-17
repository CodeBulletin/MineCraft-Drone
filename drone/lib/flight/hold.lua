local pid = require("lib.pid")
local utils = require("lib.utils")
local hold = {}

function hold.update(ctx)
    if ctx.control.hasTarget then
        ctx.targetX = ctx.control.targetX
        ctx.targetZ = ctx.control.targetZ
        if ctx.control.targetY then ctx.targetY = ctx.control.targetY end
    else
        ctx.activePathKey = ""
        local dx = ctx.targetX - ctx.x
        local dz = ctx.targetZ - ctx.z
        local distFromTarget = math.sqrt(dx*dx + dz*dz)
        if distFromTarget > 5 then
            ctx.targetX = ctx.x
            ctx.targetZ = ctx.z
            ctx.posXPID.integral = 0; ctx.posXPID.lastError = 0
            ctx.posZPID.integral = 0; ctx.posZPID.lastError = 0
            ctx.velXPID.integral = 0; ctx.velXPID.lastError = 0
            ctx.velZPID.integral = 0; ctx.velZPID.lastError = 0
        end
    end

    local dx = ctx.targetX - ctx.x
    local dz = ctx.targetZ - ctx.z
    local localDX = ctx.cosY * dx - ctx.sinY * dz
    local localDZ = ctx.sinY * dx + ctx.cosY * dz

    local targetVX = pid.update(ctx.posXPID, 0, -localDX, ctx.dt)
    local targetVZ = pid.update(ctx.posZPID, 0, -localDZ, ctx.dt)

    local distToTarget = math.sqrt(dx*dx + dz*dz)
    if distToTarget > 10 then
        ctx.velLimit = 20.0
        ctx.tiltLimit = 0.4
    else
        ctx.velLimit = 3.0
        ctx.tiltLimit = 0.15
    end

    targetVX = utils.clamp(targetVX, -ctx.velLimit, ctx.velLimit)
    targetVZ = utils.clamp(targetVZ, -ctx.velLimit, ctx.velLimit)

    if math.abs(targetVX) < 0.2 then targetVX = 0 end
    if math.abs(targetVZ) < 0.2 then targetVZ = 0 end

    ctx.targetPitch = utils.clamp(pid.update(ctx.velZPID, targetVZ, ctx.localVZ, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
    ctx.targetRoll  = utils.clamp(-pid.update(ctx.velXPID, targetVX, ctx.localVX, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
end

return hold