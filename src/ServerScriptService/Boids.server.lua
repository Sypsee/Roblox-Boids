local serverStorage = game:GetService("ServerStorage")
local replicatedStorage = game:GetService("ReplicatedStorage")
local runService = game:GetService("RunService")

local boidsService = require(serverStorage.Modules.Boid)
local boidsSolver = require(serverStorage.Modules.BoidsSolver)

type Boid = typeof(boidsService)

local boids : {Boid} = {}
local boidParts : {Part} = {}

local maxBoids = 500
local targetPos = Vector3.new(0, -50, 0)
local maxBounds = 160

runService.Heartbeat:Connect(function(dt)
    boidsSolver.Solve(dt, boids, targetPos)

    for i, boid in boids do
        boid:Update(dt)

        boidParts[i].Position = boid.Position
        -- boidParts[i].Att1.Position = boid.Position.Unit + boid.Velocity.Unit * 5
        boidParts[i].CFrame = CFrame.lookAt(boid.Position, boid.Position + boid.Velocity)
        boid:SetCFrame(boidParts[i].CFrame)

        if boid.Position.Magnitude > maxBounds then -- escapers shall face the judgment
            boidParts[i]:Destroy()
            table.remove(boidParts, i)
            table.remove(boids, i)
        end
    end
end)

for i=0, maxBoids do
    local boidPart : Part = replicatedStorage.Assets.Boid:Clone()
    boidPart.Parent = workspace.Boids
    table.insert(boidParts, boidPart)
    
    local boid = boidsService.Create(math.random(20, 30))
    boid.Velocity = boidPart.CFrame.LookVector * boid.MaxSpeed
    boid:SetCFrame(boidPart.CFrame)
    table.insert(boids, boid)
    
    runService.Heartbeat:Wait()
end