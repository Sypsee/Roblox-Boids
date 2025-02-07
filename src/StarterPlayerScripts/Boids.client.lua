-- This is an example script on how you can create the boids

local replicatedStorage = game:GetService("ReplicatedStorage")

local boidsService = require(replicatedStorage.Src.Modules.Boid)
local workers = script.Parent.Parent:WaitForChild("Workers")

local maxBoids = 500
local maxBounds = 300
local maxNumOfActors = 8

-- floor(100/8) -> 12 boids per worker
local boidsPerWorker = math.round(maxBoids/maxNumOfActors)
local boids = {}

for i=1, boidsPerWorker*maxNumOfActors do
	local boid = boidsService.Create(math.random(1, 6))
	table.insert(boids, boid)
end

local sharedBoids = SharedTable.new(boids)

local counter = 1

for i=1, maxNumOfActors do
	local boidParts : {Part} = {}
	
    for j=1, boidsPerWorker do
        local boidPart : Part = replicatedStorage.Assets.Boid:Clone()
        boidPart.Parent = workspace.Boids
		boidPart.Name = tostring(counter)
		counter += 1
		table.insert(boidParts, boidPart)
	end

    local temp : Actor = workers.Template:Clone()
    temp.Parent = workers
    temp.BoidGroup.Enabled = true
	temp.Name = tostring(i)
	
	temp:SendMessage("Create", sharedBoids, boidParts, maxBounds, boidsPerWorker)

    task.wait()
end

workers.Template:Destroy()