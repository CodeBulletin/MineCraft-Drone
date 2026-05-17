local utils   = require("lib.utils")
local manual  = require("lib.flight.manual")
local hold    = require("lib.flight.hold")
local align   = require("lib.flight.align")
local enroute = require("lib.flight.enroute")
local arrived = require("lib.flight.arrived")

local state = {}

function state.dispatch(ctx)
    if ctx.control.manual then
        manual.update(ctx)
        return
    end

    if ctx.wasManual then
        ctx.posXPID.integral = 0; ctx.posXPID.lastError = 0
        ctx.posZPID.integral = 0; ctx.posZPID.lastError = 0
        ctx.velXPID.integral = 0; ctx.velXPID.lastError = 0
        ctx.velZPID.integral = 0; ctx.velZPID.lastError = 0
        ctx.pathCrossPID.integral = 0; ctx.pathCrossPID.lastError = 0
        ctx.pathAlongPID.integral = 0; ctx.pathAlongPID.lastError = 0
        ctx.activePathKey = ""
        ctx.wasManual = false
    end

    local inPathMode = ctx.control.hasTarget and ctx.control.prevX and ctx.control.prevZ
    local pathDegenerate = false

    if inPathMode then
        local px = ctx.control.targetX - ctx.control.prevX
        local pz = ctx.control.targetZ - ctx.control.prevZ
        if px*px + pz*pz < 0.0001 then pathDegenerate = true end
    end

    if inPathMode and not pathDegenerate then
        local Ax, Az = ctx.control.prevX, ctx.control.prevZ
        local Bx, Bz = ctx.control.targetX, ctx.control.targetZ

        local px = Bx - Ax
        local pz = Bz - Az
        local pathLen = math.sqrt(px*px + pz*pz)

        local ufx = px / pathLen
        local ufz = pz / pathLen
        local urx = ufz
        local urz = -ufx

        local dx = ctx.x - Ax
        local dz = ctx.z - Az
        local proj = dx*ufx + dz*ufz
        local t = proj / pathLen

        local closestT = utils.clamp(t, 0, 1)
        local cx = Ax + closestT*px
        local cz = Az + closestT*pz
        local cross = (ctx.x - cx)*urx + (ctx.z - cz)*urz

        local dxB = Bx - ctx.x
        local dzB = Bz - ctx.z
        local distToB = math.sqrt(dxB*dxB + dzB*dzB)
        local hVel = math.sqrt(ctx.vx*ctx.vx + ctx.vz*ctx.vz)

        local isArrived = distToB < 1.0 and hVel < 0.5
        local approachFactor = utils.clamp(distToB / 10.0, 0, 1)
        approachFactor = approachFactor * approachFactor * (3 - 2 * approachFactor)
        
        ctx.velLimit = 0.5 + approachFactor * (20.0 - 0.5)
        ctx.tiltLimit = 0.08 + approachFactor * (0.4 - 0.08)

        local pathYaw = math.atan2(px, pz)
        local desiredHeading = (ctx.control.useTargetYaw and ctx.control.targetYaw) and ctx.control.targetYaw or pathYaw

        local yawError = utils.angleDiff(desiredHeading, ctx.targetYaw)
        local maxYawStep = 3.0 * ctx.dt
        if math.abs(yawError) > maxYawStep then
            yawError = yawError > 0 and maxYawStep or -maxYawStep
        end
        ctx.targetYaw = utils.normalizeAngle(ctx.targetYaw + yawError)

        local currentPathKey = tostring(Bx) .. "_" .. tostring(Bz)
        if ctx.activePathKey ~= currentPathKey then
            ctx.alignState = true
            ctx.activePathKey = currentPathKey
        end

        local yawErrorActual = math.abs(utils.angleDiff(desiredHeading, ctx.yaw))
        local altErrorActual = math.abs(ctx.targetY - ctx.alt)
        local linearVelMag = math.sqrt(ctx.vx*ctx.vx + ctx.vy*ctx.vy + ctx.vz*ctx.vz)
        local angularVelMag = math.sqrt(ctx.pitchRate*ctx.pitchRate + ctx.yawRate*ctx.yawRate + ctx.rollRate*ctx.rollRate)

        if ctx.alignState 
        and yawErrorActual < 0.15 
        and altErrorActual < 1 
        and linearVelMag < ctx.ALIGN_VEL_TOLERANCE 
        and angularVelMag < ctx.ALIGN_ANG_TOLERANCE then
            ctx.alignState = false
        end

        if isArrived then
            arrived.update(ctx, Bx, Bz)
        elseif ctx.alignState then
            align.update(ctx, Ax, Az)
        else
            enroute.update(ctx, cross, pathLen - proj, t, pathYaw)
        end
    else
        hold.update(ctx)
    end
end

return state