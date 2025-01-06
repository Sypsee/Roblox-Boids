local serverStorage = game:GetService("ServerStorage")
local replicatedStorage = game:GetService("ReplicatedStorage")
local runService = game:GetService("RunService")

local boidsService = require(serverStorage.Modules.Boid)
local boidsSolver = require(serverStorage.Modules.BoidsSolver)

type Boid = typeof(boidsService)

local boids : {Boid} = {}
local boidParts : {Part} = {}

local maxBoids = 100
local targetPos = Vector3.new(0, 20, 0)

task.spawn(function()
    for i=0, maxBoids do
        local boidPart : Part = replicatedStorage.Assets.Boid:Clone()
        boidPart.Parent = workspace.Boids
        table.insert(boidParts, boidPart)
        
        local boid = boidsService.Create(math.random(20, 50))
        boid.Velocity = Vector3.new(math.random(0, 15), math.random(0, 15), math.random(0, 15))
        table.insert(boids, boid)
        
        runService.Heartbeat:Wait()
    end
end)

runService.Heartbeat:Connect(function(dt)
    boidsSolver.Solve(dt, boids, targetPos)

    for i, boid in boids do
        boid:Update(dt)

        boidParts[i].Position = boid.Position
        boidParts[i].CFrame = CFrame.lookAlong(boid.Position, boid.Position + boid.Velocity, boidParts[i].CFrame.UpVector)
        boid:SetCFrame(boidParts[i].CFrame)
    end
end)