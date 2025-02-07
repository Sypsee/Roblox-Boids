local solver = {}
solver.__index = solver
local utils = require(script.Parent.BoidUtils)

-- DEF -> DEFAULT

-- The max distance between 2 boids to make then grouped
local DEF_MAX_NEIGHBOUR_DISTANCE = 15
-- The max view angle for an boid to see in (-1 to disable, range [-1, 1])
local DEF_MAX_VIEW_ANGLE = -1
-- The max view range the boid can see for
local DEF_MAX_VIEW_RANGE = 7
-- This factor decides the tendency of boid to move at the center of a group
local DEF_COHESION_FACTOR = 0.1
-- This factor decides the tendency of boid to move aligned with other boids
local DEF_ALIGNMENT_FACTOR = 1
-- This factor decides the tendency of boid to move away from boids
local DEF_SEPRATION_FACTOR = 1
-- This factor decides the tendency of boid to avoid obstacles (MUST BE HIGH)
local DEF_OBSTACLE_FACTOR = 120
-- This is more performance heavy but MUST BE used for enclosed spaces
local DEF_RAYCAST_AVOIDANCE = true
-- Dont change if using raycast avoidance (tells the distance after which obstacles should be avoided)
local DEF_OBSTACLE_DISTANCE = 5
-- This factor decides the tendency of boid to move towards the target (MUST BE LOW)
local DEF_TARGET_FACTOR = 0.02
-- The distance between 2 boids to seprate them so they dont intersect
local DEF_SEPRATING_DISTANCE = 3
-- The max speed for a boid when steering towards a new velocity
local DEF_MAX_STEERING_SPEED = 12
-- Target point (the point towards the boid can go when not in group)
local DEF_TARGET_POINT = Vector3.zero

-- The raycast params that decide the obstacles
local DEF_PARAMS = RaycastParams.new()
DEF_PARAMS.FilterType = Enum.RaycastFilterType.Exclude
DEF_PARAMS.FilterDescendantsInstances = {workspace.Boids}

-- The directions which a boid can look into to avoid obstacles
local DIRECTIONS = utils.GetDirections(300)

local boidsParams = OverlapParams.new()
boidsParams.FilterType = Enum.RaycastFilterType.Include
boidsParams.FilterDescendantsInstances = {workspace.Boids}

local obstacleParams = OverlapParams.new()
obstacleParams.FilterType = Enum.RaycastFilterType.Exclude
obstacleParams.FilterDescendantsInstances = {workspace.Boids}

export type solver = typeof(solver) & {
	rayParams: RaycastParams,
	maxNeighbourDist: number,
	maxViewAngle: number,
	maxViewRange: number,
	cohesionFactor: number,
	alignmentFactor: number,
	seprationFactor: number,
	obstacleFactor: number,
	raycastAvoidance: boolean,
	obstacleDistance: number,
	targetFactor: number,
	sepratingDistance: number,
	maxSteeringSpeed: number,
	target: Vector3
}

type Boid = {
	Position : Vector3,
	_CFrame : CFrame,
	Velocity : Vector3,
	MaxSpeed : number,
	Accelerate : (self, force : Vector3) -> ()
}

-- NOTE: Boid is a custom object not A ROBLOX INSTANCE
-- This makes the code thread safe and easier to customize according to ones needs
-- There is no dependency of roblox instances so idk what you were calling deprecated
-- This is multi-threaded using actors to give out the most performance
-- I dont think I would like to make my code MORE THEN 200 loc which is useless destroys simplicity and modularity in my case


function solver.Init()
    local self = setmetatable({
        rayParams = DEF_PARAMS,
        maxNeighbourDist = DEF_MAX_NEIGHBOUR_DISTANCE,
        maxViewAngle = DEF_MAX_VIEW_ANGLE,
        maxViewRange = DEF_MAX_VIEW_RANGE,
        cohesionFactor = DEF_COHESION_FACTOR,
        alignmentFactor = DEF_ALIGNMENT_FACTOR,
        seprationFactor = DEF_SEPRATION_FACTOR,
        obstacleFactor = DEF_OBSTACLE_FACTOR,
        raycastAvoidance = DEF_RAYCAST_AVOIDANCE,
        obstacleDistance = DEF_OBSTACLE_DISTANCE,
        targetFactor = DEF_TARGET_FACTOR,
        sepratingDistance = DEF_SEPRATING_DISTANCE,
        maxSteeringSpeed = DEF_MAX_STEERING_SPEED,
        target = DEF_TARGET_POINT
    }, solver)

    return self
end

-- A simple function which steers towards the given direction v
-- Clamps the magnitude or speed to the maxSpeed
function solver.SteerTowards(v : Vector3, boid, maxSteeringSpeed : number) : Vector3
	local newV = v.Unit * boid.MaxSpeed - boid.Velocity
	return utils.clampMagnitude(newV, 0, maxSteeringSpeed)
end


--[[
Checks through a list of possible directions in a unit sphere and raycasts there
if Is path obstructed? if no then select that dir and move there
else find another dir
]]
function solver.FindUnobstructedDirection(self : solver, boid : Boid) : (Vector3, boolean?)
    local bestDir = boid._CFrame.LookVector
	
    if self.raycastAvoidance then
        local foundBest = false

        for i, _dir in DIRECTIONS do
            local dir : Vector3 = boid._CFrame:VectorToWorldSpace(_dir)
            if boid._CFrame.LookVector:Dot(dir) > 0.5 then continue end

            local res = workspace:Raycast(boid.Position, _dir.Unit * self.maxViewRange, self.rayParams)
            local isPathObstructed = res and res.Instance

            if isPathObstructed then continue end
			
			bestDir = dir
			foundBest = true
			break
        end

        return bestDir, foundBest
	end
	
	local parts = workspace:GetPartBoundsInRadius(boid.Position, self.obstacleDistance, obstacleParams)

	for _, part in parts do
		local diff = (part.Position - boid.Position)
		if diff.Magnitude == 0 then continue end

		bestDir -= diff.Unit / (diff.Magnitude)
	end

	return bestDir, #parts ~= 0
end

-- Shoot a ray in front to check for any possible obstructions in the path
-- If yes then call the FindUnobstructedDirection function to get new path
function solver.CheckForObstruction(self: solver, boid : Boid) : Vector3
	if self.raycastAvoidance then
		local ray = workspace:Raycast(boid.Position, boid._CFrame.LookVector * self.maxViewRange, self.rayParams)
		if not ray then return Vector3.zero end
		
		local unobstructedDir, foundBest = solver.FindUnobstructedDirection(self, boid)
		local unobstructedForce = solver.SteerTowards(unobstructedDir, boid, self.maxSteeringSpeed) * self.obstacleFactor
		if not foundBest or utils.isNanVec3(unobstructedForce) then return Vector3.zero end
		
		return unobstructedForce
	else
		local unobstructedDir, foundBest = solver.FindUnobstructedDirection(self, boid)
		local unobstructedForce = solver.SteerTowards(unobstructedDir, boid, self.maxSteeringSpeed) * self.obstacleFactor
		if not foundBest or utils.isNanVec3(unobstructedForce) then return Vector3.zero end
		
		return unobstructedForce
	end
end

function solver:Solve(dt : number, boids : {Boid}, allBoids : {Boid})
	for _n=1, SharedTable.size(boids) do
		local boid = boids[_n]
		if not boid then continue end

        local acceleration = Vector3.zero
        local dir = Vector3.zero

        local sumAdjPos = Vector3.zero
        local sumAdjDir = Vector3.zero
		local adjBoids = 0
		
		-- Loop through all other neighbouring boids and do some checks like viewRange then
		-- Add the all the other boids Velocities and Positions to variables like sumAdjPos and sumAdjDir
		-- Increment the adjBoids
        for _m, part in workspace:GetPartBoundsInRadius(boid.Position, self.maxNeighbourDist, boidsParams) do
			local adjBoidIndex = tonumber(part.Name)
			if _n == adjBoidIndex then continue end
			
			local adjBoid = allBoids[adjBoidIndex]
            if boid.Velocity:Dot(adjBoid.Velocity) < self.maxViewAngle then continue end
			
			local diff = adjBoid.Position - boid.Position
			local dist = diff.Magnitude
			
            if dist <= self.sepratingDistance then
                dir -= (diff / dist)
            end

            adjBoids += 1
            sumAdjPos += adjBoid.Position
            sumAdjDir += adjBoid.Velocity
        end
		
		-- If the boid is not in much crowd then we can gather them all at a TARGET_POINT
		-- which will make them together again
        if self.target then
            local diff = self.target - boid.Position
            local targetForce = solver.SteerTowards(diff, boid, self.maxSteeringSpeed) / (adjBoids + 1) * self.targetFactor

            if not utils.isNanVec3(targetForce) then
                acceleration += targetForce
            end
        end

		acceleration += solver.CheckForObstruction(self, boid)
		
		-- If the adjBoids are more then 0 then we can add the flock behaviour forces
		-- like cohesion, sepration and alignment
		-- Here we steer the boids towards the avg pos(center of group) and avg dir of the neighbouring boids
		-- We then add a seprationForce to avoid them from getting too close
        if adjBoids > 0 then
            local avgPos = sumAdjPos / adjBoids
            local avgDir = sumAdjDir / adjBoids

            local offsetToCenter = avgPos - boid.Position

            local alignmentForce = solver.SteerTowards(avgDir, boid, self.maxSteeringSpeed) * self.alignmentFactor
            local cohesionForce = solver.SteerTowards(offsetToCenter, boid, self.maxSteeringSpeed) * self.cohesionFactor
            local seprationForce = solver.SteerTowards(dir, boid, self.maxSteeringSpeed) * self.seprationFactor

            if not utils.isNanVec3(cohesionForce) then
                acceleration += cohesionForce
            end
            if not utils.isNanVec3(alignmentForce) then
                acceleration += alignmentForce
            end
            if not utils.isNanVec3(seprationForce) then
                acceleration += seprationForce
			end
        end

		if utils.isNanVec3(acceleration) then return end
		
		-- Finally we can call the accelerate function of our custom boid object with all the forces applied
		-- accel * dt -> velocity
		boid.Velocity += acceleration * dt
		boid.Velocity = utils.clampMagnitude(boid.Velocity, 1, boid.MaxSpeed)
    end
end

return solver