local solver = {}
solver.__index = solver
local utils = require(script.Parent.BoidUtils)

-- The max distance between 2 boids to make then grouped
local DEF_MAX_NEIGHBOUR_DISTANCE = 10
-- The max view angle for an boid to see in (-1 to disable, range [-1, 1])
local DEF_MAX_VIEW_ANGLE = -1
-- The max view range the boid can see for
local DEF_MAX_VIEW_RANGE = 10
-- This factor decides the tendency of boid to move at the center of a group
local DEF_COHESION_FACTOR = 0.15
-- This factor decides the tendency of boid to move aligned with other boids
local DEF_ALIGNMENT_FACTOR = 1.1
-- This factor decides the tendency of boid to move away from boids
local DEF_SEPRATION_FACTOR = 0.85
-- This factor decides the tendency of boid to avoid obstacles (MUST BE HIGH)
local DEF_OBSTACLE_FACTOR = 100
-- This is more performance heavy but MUST BE used for enclosed spaces
local DEF_RAYCAST_AVOIDANCE = false
-- Dont change if using raycast avoidance (tells the distance after which obstacles should be avoided)
local DEF_OBSTACLE_DISTANCE = 3
-- This factor decides the tendency of boid to move towards the target (MUST BE LOW)
local DEF_TARGET_FACTOR = 0.1
-- The distance between 2 boids to seprate them so they dont intersect
local DEF_SEPRATING_DISTANCE = 3
-- The max speed for a boid when steering towards a new velocity
local DEF_MAX_STEERING_SPEED = 15
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
boidsParams.FilterType = Enum.RaycastFilterType.Exclude
boidsParams.FilterDescendantsInstances = {workspace.Boids}

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

function solver.FindUnobstructedDirection(boid, maxViewRange : number, rayParams : RaycastParams, raycastAvoidance : boolean, obstacleRange : boolean?)
    local bestDir = boid.CFrame.LookVector

    if raycastAvoidance then
        local foundBest = false

        for i, _dir in DIRECTIONS do
            local dir : Vector3 = boid.CFrame:VectorToWorldSpace(_dir)
            if boid.CFrame.LookVector:Dot(dir) > 0.5 then continue end

            local res = workspace:Raycast(boid.Position, _dir.Unit * maxViewRange, rayParams)
            local isPathObstructed = res and res.Instance

            if not isPathObstructed then
                bestDir = dir
                foundBest = true
                break
            end
        end

        return bestDir, foundBest
    else
        local parts = workspace:GetPartBoundsInRadius(boid.Position, obstacleRange, obstacleParams)

        for _, part in parts do
            local diff = (part.Position - boid.Position)
            if diff.Magnitude == 0 then continue end

            bestDir -= diff.Unit / (diff.Magnitude)
        end

        return bestDir, #parts ~= 0
    end
end

function solver.SteerTowards(v : Vector3, boid, maxSteeringSpeed : number)
    local newV = v.Unit * boid.MaxSpeed - boid.Velocity
    return utils.clampMagnitude(newV, 0, maxSteeringSpeed)
end

function solver:Solve(dt : number, boids)
    for _n, boid in boids do
        local acceleration = Vector3.zero
        local dir = Vector3.zero

        local sumAdjPos = Vector3.zero
        local sumAdjDir = Vector3.zero
        local adjBoids = 0

        -- for _m, adjBoid in boids do
        --     local diff = adjBoid.Position - boid.Position
        --     local dist = diff.Magnitude
        --     if _n == _m or dist > self.maxNeighbourDist then continue end
        --     if boid.Velocity:Dot(adjBoid.Velocity) < self.maxViewAngle then continue end
            
        --     if dist <= self.sepratingDistance then
        --         dir -= (diff / dist)
        --     end

        --     adjBoids += 1
        --     sumAdjPos += adjBoid.Position
        --     sumAdjDir += adjBoid.Velocity
        -- end

        for _m, part in workspace:GetPartBoundsInRadius(boid.Position, self.maxNeighbourDist, boidsParams) do
            local adjBoid = boids[part:GetAttribute("Index")]
            if adjBoid == nil then continue end

            local diff = adjBoid.Position - boid.Position
            local dist = diff.Magnitude
            if _n == part:GetAttribute("Index")then continue end
            if boid.Velocity:Dot(adjBoid.Velocity) < self.maxViewAngle then continue end
            
            if dist <= self.sepratingDistance then
                dir -= (diff / dist)
            end

            adjBoids += 1
            sumAdjPos += adjBoid.Position
            sumAdjDir += adjBoid.Velocity
        end

        if self.target then
            local diff = self.target - boid.Position
            local targetForce = solver.SteerTowards(diff, boid, self.maxSteeringSpeed) / (adjBoids + 1) * self.targetFactor

            if not utils.isNanVec3(targetForce) then
                acceleration += targetForce
            end
        end

        if self.raycastAvoidance then
            local ray = workspace:Raycast(boid.Position, boid.CFrame.LookVector * self.maxViewRange, self.rayParams)

            if ray then
                local unobstructedDir, foundBest = solver.FindUnobstructedDirection(boid, self.maxViewRange, self.rayParams, self.raycastAvoidance, self.obstacleDistance)

                if foundBest then
                    local unobstructedForce = solver.SteerTowards(unobstructedDir, boid, self.maxSteeringSpeed) * self.obstacleFactor

                    if not utils.isNanVec3(unobstructedForce) then
                        acceleration += unobstructedForce
                    end
                end
            end
        else
            local unobstructedDir, foundBest = solver.FindUnobstructedDirection(boid, self.maxViewRange, self.rayParams, self.raycastAvoidance, self.obstacleDistance)

            if foundBest then
                local unobstructedForce = solver.SteerTowards(unobstructedDir, boid, self.maxSteeringSpeed) * self.obstacleFactor

                if not utils.isNanVec3(unobstructedForce) then
                    acceleration += unobstructedForce
                end
            end
        end

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

        if not utils.isNanVec3(acceleration) then
            boid:Accelerate(acceleration * dt)
        end
    end
end

return solver