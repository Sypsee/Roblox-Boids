local boid = {}
boid.__index = boid

function clampMagnitude(v : Vector3, minMagnitude : number, maxMagnitude : number)
    if v.Magnitude > maxMagnitude then
        return v.Unit * maxMagnitude
    elseif v.Magnitude < minMagnitude then
        return v.Unit * minMagnitude
    end
    
    return v
end

function boid.Create(maxSpeed)
    local self = setmetatable({
        Position = Vector3.zero,
        CFrame = CFrame.identity,
        Velocity = Vector3.zero,
        MaxSpeed = maxSpeed
    }, boid)

    return self
end

function boid:Update(dt)
    self.Position += self.Velocity * dt
end

function boid:AddForce(force : Vector3)
    self.Velocity += force
    self.Velocity = clampMagnitude(self.Velocity, 1, self.MaxSpeed)
end

-- Just for the boid solver to access boidParts CFrame
function boid:SetCFrame(cframe : CFrame)
    self.CFrame = cframe
end

return boid