local pid = require("lib.pid")
local utils = require("lib.utils")
local recovery = {}

local RECOVERY_JUMP_THROTTLE = 240
local RECOVERY_AIR_THROTTLE = 100
local STUCK_TIME = 1.0
local JUMP_PULSE_TIME = 0.35

function recovery.update(ctx)
    local upVector = math.cos(ctx.pitch) * math.cos(ctx.roll)
    local recoveryMode = upVector < 0.2

    if recoveryMode then
        if not ctx.wasRecovery then
            ctx.recoveryTimer = 0
            ctx.jumpPulseTimer = 0
        end
        ctx.recoveryTimer = ctx.recoveryTimer + ctx.dt
    else
        ctx.recoveryTimer = 0
        ctx.jumpPulseTimer = 0
    end
    ctx.wasRecovery = recoveryMode

    local vyNearlyZero = math.abs(ctx.vy) < 0.8
    local isStuck = recoveryMode and (ctx.recoveryTimer > STUCK_TIME) and vyNearlyZero

    if isStuck and ctx.jumpPulseTimer == 0 then
        ctx.jumpPulseTimer = JUMP_PULSE_TIME
    end

    if ctx.jumpPulseTimer > 0 then
        ctx.jumpPulseTimer = math.max(0, ctx.jumpPulseTimer - ctx.dt)
    end

    local isJumping = ctx.jumpPulseTimer > 0
    local throttle, multiplyer, invert = 0, 1.0, 1.0

    if recoveryMode then
        ctx.targetX = ctx.x
        ctx.targetZ = ctx.z

        ctx.altPID.integral = 0; ctx.altPID.lastError = 0
        ctx.velPID.integral = 0; ctx.velPID.lastError = 0
        ctx.posXPID.integral = 0; ctx.posXPID.lastError = 0
        ctx.posZPID.integral = 0; ctx.posZPID.lastError = 0
        ctx.velXPID.integral = 0; ctx.velXPID.lastError = 0
        ctx.velZPID.integral = 0; ctx.velZPID.lastError = 0

        if isJumping then
            throttle = RECOVERY_JUMP_THROTTLE
            multiplyer = 0.05
            invert = -1.0
        else
            throttle = RECOVERY_AIR_THROTTLE
        end
    else
        local tiltFactor = 1.0 / math.max(0.5, upVector)
        local yawCompensation = math.abs(ctx.yawRate) * 2.5
        throttle = utils.clamp((ctx.baseThrottle + ctx.yCorr) * tiltFactor - yawCompensation, 30, 250)
    end

    return throttle, invert, multiplyer
end

return recovery