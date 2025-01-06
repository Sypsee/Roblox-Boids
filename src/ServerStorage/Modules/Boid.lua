local boid = {}
boid.__index = boid

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

    if (self.Velocity.Magnitude > self.MaxSpeed) then
        self.Velocity = self.Velocity.Unit * self.MaxSpeed
    end
end

function boid:SetCFrame(cframe : CFrame)
    self.CFrame = cframe
end

return boid