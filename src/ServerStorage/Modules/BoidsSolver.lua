local solver = {}
solver.__index = solver
local utils = require(script.Parent.BoidUtils)

local DEF_MAX_NEIGHBOUR_DISTANCE = 10
local DEF_MAX_VIEW_ANGLE = -0.8
local DEF_MAX_VIEW_RANGE = 10
local DEF_COHESION_FACTOR = 0.1
local DEF_ALIGNMENT_FACTOR = 1.0
local DEF_SEPRATION_FACTOR = 1.0
local DEF_OBSTACLE_FACTOR = 100
local DEF_TARGET_FACTOR = 0.2
local DEF_SEPRATING_DISTANCE = 4
local DEF_MAX_STEERING_SPEED = 10
local DEF_TARGET_POINT = Vector3.zero

local DEF_PARAMS = RaycastParams.new()
DEF_PARAMS.FilterType = Enum.RaycastFilterType.Exclude
DEF_PARAMS.FilterDescendantsInstances = {workspace.Boids}

local DIRECTIONS = utils.GetDirections(1000)

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
        targetFactor = DEF_TARGET_FACTOR,
        sepratingDistance = DEF_SEPRATING_DISTANCE,
        maxSteeringSpeed = DEF_MAX_STEERING_SPEED,
        target = DEF_TARGET_POINT
    }, solver)

    return self
end

function solver.FindUnobstructedDirection(boid, maxViewRange : number, rayParams : RaycastParams) : Vector3
    local bestDir = boid.CFrame.LookVector
    local foundBest = false

    for i, _dir in DIRECTIONS do
        local dir : Vector3 = boid.CFrame:VectorToWorldSpace(_dir)
        local res = workspace:Raycast(boid.Position, _dir.Unit * maxViewRange, rayParams)
        local isPathObstructed = res and res.Instance

        if not isPathObstructed and boid.CFrame.LookVector:Dot(dir) < 0 then
            bestDir = dir
            foundBest = true
            break
        end
    end

    return bestDir, foundBest
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

        for _m, adjBoid in boids do
            local diff = adjBoid.Position - boid.Position
            local dist = diff.Magnitude
            if _n == _m or dist > self.maxNeighbourDist then continue end
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

        local ray = workspace:Raycast(boid.Position, boid.CFrame.LookVector * self.maxViewRange, self.rayParams)

        if ray then
            local unobstructedDir, foundBest = solver.FindUnobstructedDirection(boid, self.maxViewRange, self.rayParams)

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