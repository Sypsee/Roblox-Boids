local boid = {}
boid.__index = boid

local utils = require(script.Parent.BoidUtils)

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

function boid:Accelerate(acceleration : Vector3)
    self.Velocity += acceleration
    self.Velocity = utils.clampMagnitude(self.Velocity, 1, self.MaxSpeed)
end

-- Just for the boid solver to access boidParts CFrame
function boid:SetCFrame(cframe : CFrame)
    self.CFrame = cframe
end

return boid