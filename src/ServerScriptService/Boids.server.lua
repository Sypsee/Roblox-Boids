-- This is an example script on how you can create the boids

local serverStorage = game:GetService("ServerStorage")
local replicatedStorage = game:GetService("ReplicatedStorage")
local runService = game:GetService("RunService")

local boidsService = require(serverStorage.Modules.Boid)
local boidsSolver = require(serverStorage.Modules.BoidsSolver)

type Boid = typeof(boidsService)

local boids : {Boid} = {}
local boidParts : {Part} = {}

local maxBoids = 500
local maxBounds = 160

local solver = boidsSolver.Init()

runService.Heartbeat:Connect(function(dt)
    solver:Solve(dt, boids)

    for i, boid in boids do
        boid:Update(dt)

        boidParts[i].Position = boid.Position
        boidParts[i].CFrame = CFrame.lookAt(boid.Position, boid.Position + boid.Velocity)
        boid:SetCFrame(boidParts[i].CFrame)

        if boid.Position.Magnitude > maxBounds then -- escapers shall face the judgment
            boidParts[i]:Destroy()
            table.remove(boidParts, i)
            table.remove(boids, i)
        end
    end
end)

for i=1, maxBoids do
    local boidPart : Part = replicatedStorage.Assets.Boid:Clone()
    boidPart.Parent = workspace.Boids
    boidPart:SetAttribute("Index", i)
    table.insert(boidParts, boidPart)
    
    local boid = boidsService.Create(math.random(20, 30))
    boid.Velocity = boidPart.CFrame.LookVector * boid.MaxSpeed
    boid:SetCFrame(boidPart.CFrame)
    table.insert(boids, boid)
    
    runService.Heartbeat:Wait()
end