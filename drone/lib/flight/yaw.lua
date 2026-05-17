local pid = require("lib.pid")
local utils = require("lib.utils")
local yawLogic = {}

function yawLogic.update(ctx)
    local yawStickActive = math.abs(ctx.control.yc) > 0.4

    if yawStickActive then
        ctx.targetYawRate = ctx.control.yc * 4.0
        ctx.targetYaw = ctx.yaw
        ctx.wasYawing = true
    else
        if ctx.wasYawing then
            if not (ctx.control.hasTarget and ctx.control.prevX and ctx.control.prevZ) then
                ctx.targetYaw = ctx.yaw
            end
            ctx.wasYawing = false
            ctx.yawPID.integral = 0; ctx.yawPID.lastError = 0
            ctx.yawRatePID.integral = 0; ctx.yawRatePID.lastError = 0
        end

        local yawError = utils.angleDiff(ctx.targetYaw, ctx.yaw)
        local YAW_DEADBAND = 0.03
        if math.abs(yawError) < YAW_DEADBAND then yawError = 0 end
        
        ctx.targetYawRate = pid.update(ctx.yawPID, yawError, 0, ctx.dt)
    end
end

return yawLogic