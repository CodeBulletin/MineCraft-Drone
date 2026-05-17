local pid = require("lib.pid")
local utils = require("lib.utils")
local align = {}

function align.update(ctx, Ax, Az)
    ctx.pathCrossPID.integral = 0; ctx.pathCrossPID.lastError = 0
    ctx.pathAlongPID.integral = 0; ctx.pathAlongPID.lastError = 0

    ctx.targetX = Ax
    ctx.targetZ = Az

    local pdx = ctx.targetX - ctx.x
    local pdz = ctx.targetZ - ctx.z
    local localDX = ctx.cosY * pdx - ctx.sinY * pdz
    local localDZ = ctx.sinY * pdx + ctx.cosY * pdz

    local targetVX = pid.update(ctx.posXPID, 0, -localDX, ctx.dt)
    local targetVZ = pid.update(ctx.posZPID, 0, -localDZ, ctx.dt)

    local holdLimit = 1.0
    targetVX = utils.clamp(targetVX, -holdLimit, holdLimit)
    targetVZ = utils.clamp(targetVZ, -holdLimit, holdLimit)

    if math.abs(targetVX) < 0.1 then targetVX = 0 end
    if math.abs(targetVZ) < 0.1 then targetVZ = 0 end

    ctx.targetPitch = utils.clamp(pid.update(ctx.velZPID, targetVZ, ctx.localVZ, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
    ctx.targetRoll  = utils.clamp(-pid.update(ctx.velXPID, targetVX, ctx.localVX, ctx.dt), -ctx.tiltLimit, ctx.tiltLimit)
end

return align