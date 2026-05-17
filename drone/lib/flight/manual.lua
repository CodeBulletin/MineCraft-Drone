local manual = {}

function manual.update(ctx)
    ctx.wasManual = true
    ctx.targetPitch = ctx.control.fb * 0.35
    ctx.targetRoll = -ctx.control.rl * 0.35
    ctx.targetX = ctx.x
    ctx.targetZ = ctx.z
end

return manual