local utils = require("lib.utils")
local pid = {}

function pid.create(kp, ki, kd)
    return {
        kp = kp,
        ki = ki,
        kd = kd,
        integral = 0,
        lastError = 0,
    }
end

function pid.update(p, targetRate, currentRate, dt)
    local error = targetRate - currentRate
    
    p.integral = p.integral + error * dt
    p.integral = math.max(-50, math.min(50, p.integral))

    local derivative = (error - p.lastError) / dt
    if not utils.isValidNumber(derivative) then derivative = 0 end
    
    p.lastError = error
    
    return p.kp * error + p.ki * p.integral + p.kd * derivative
end

return pid