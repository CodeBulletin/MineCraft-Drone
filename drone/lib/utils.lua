local utils = {}

function utils.isValidNumber(x)
    return type(x) == "number" and x == x and x ~= math.huge and x ~= -math.huge
end

function utils.clampInt(v, min, max)
    v = math.floor(v + 0.5)
    return math.max(min, math.min(max, v))
end

function utils.clamp(v, min, max)
    return math.max(min, math.min(max, v))
end

function utils.smooth(new, old, factor)
    return old + (new - old) * factor
end

function utils.angleDiff(target, current)
    local diff = target - current
    while diff > math.pi do diff = diff - 2*math.pi end
    while diff < -math.pi do diff = diff + 2*math.pi end
    return diff
end

function utils.getTPA(throttle, base)
    if throttle < base then
        local factor = (throttle - 30) / (base - 30)
        return math.max(0.7, factor)
    end
    return 1.0
end

function utils.normalizeAngle(a)
    while a > math.pi do a = a - 2*math.pi end
    while a < -math.pi do a = a + 2*math.pi end
    return a
end

return utils