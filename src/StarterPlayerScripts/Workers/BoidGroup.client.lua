-- [This is under a actor]

local replicatedStorage = game:GetService("ReplicatedStorage")
local runService = game:GetService("RunService")

local boidsService = require(replicatedStorage.Src.Modules.Boid)
local boidsSolver = require(replicatedStorage.Src.Modules.BoidsSolver)

local solver = boidsSolver.Init()

script.Parent:BindToMessageParallel("Create", function(boids : SharedTable, boidParts: {BasePart}, maxBounds : number, boidsPerWorker : number)
	local actorNum = tonumber(script.Parent.Name)
	
	for i=1, SharedTable.size(boids) do
		boids[i] = boidsService.Create(boids[i].MaxSpeed)
		boids[i].Velocity = boids[i]._CFrame.LookVector * boids[i].MaxSpeed
		boids[i].Position = Vector3.one * math.random(-5, 5)
	end
	
	local actorBoids = SharedTable.clone(boids, false)
	for i=tonumber(script.Parent.Name) * boidsPerWorker, SharedTable.size(actorBoids) do
		actorBoids[i] = nil
	end
	
	for i=1, ((tonumber(script.Parent.Name)-1) * boidsPerWorker)-1 do
		actorBoids[i] = nil
	end
	
	local index = 1
	
	-- Shift them to start
	for i = 1, SharedTable.size(actorBoids) do
		if actorBoids[i] then
			actorBoids[index] = actorBoids[i]
			if index ~= i then
				actorBoids[i] = nil
			end
			index = index + 1
		end
	end
	
	task.synchronize()
	runService.Heartbeat:ConnectParallel(function(dt)
		solver:Solve(dt, actorBoids, boids)

		for i, boid in boids do
			if boid.Position.Magnitude > maxBounds then
				boid.Position = Vector3.zero
				boid.Velocity = Vector3.zero
			end

			boid.Position += boid.Velocity * dt
			boid._CFrame = CFrame.lookAt(boid.Position, boid.Position + boid.Velocity)
			
			task.synchronize()
			if boidParts[i] == nil then continue end
			boidParts[i].CFrame = boid._CFrame
		end
	end)
end)