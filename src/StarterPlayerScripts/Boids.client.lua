-- This is an example script on how you can create the boids

local replicatedStorage = game:GetService("ReplicatedStorage")

local workers = script.Parent.Parent:WaitForChild("Workers")
local runEvent : BindableEvent = script.Parent.Parent:WaitForChild("Events").Run
-- can use this event to update position
local updatePosEvent : BindableEvent = script.Parent.Parent.Events.UpdatePos

local maxBoids = 1000
local maxBounds = 300
local maxNumOfActors = 8

-- floor(100/8) -> 12 boids per worker
local boidsPerWorker = math.floor(maxBoids/maxNumOfActors)

for i=1, maxNumOfActors do
    local boidParts : {Part} = {}

    for i=1, boidsPerWorker do
        local boidPart : Part = replicatedStorage.Assets.Boid:Clone()
        boidPart.Parent = workspace.Boids
        boidPart.Name = tostring(i)
        table.insert(boidParts, boidPart)
    end

    local temp : Actor = workers.Template:Clone()
    temp.Parent = workers
    temp.BoidGroup.Enabled = true
    temp.Name = tostring(i)
    
    runEvent:Fire(boidsPerWorker, boidParts, maxBounds)

    task.wait()
end

workers.Template:Destroy()