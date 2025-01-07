local solver = {}

local maxNeighbourRadius = 10
local maxViewAngle = -0.5
local maxViewRange = 10
local cohesionFactor = 0.1
local alignmentFactor = 1.0
local seprationFactor = 1.0
local obstacleSeprationFactor = 100
local targetFactor = 0.2
local sepratingDistance = 4
local maxSteeringSpeed = 10

local params = RaycastParams.new()
params.FilterType = Enum.RaycastFilterType.Exclude
params.FilterDescendantsInstances = {workspace.Boids}

function isNan(n : number) : boolean
    return n ~= n
end

function isNanVec3(v : Vector3) : boolean
    return isNan(v.X) or isNan(v.Y) or isNan(v.Z)
end

function clampMagnitude(v : Vector3, minMagnitude : number, maxMagnitude : number)
    if v.Magnitude > maxMagnitude then
        return v.Unit * maxMagnitude
    elseif v.Magnitude < minMagnitude then
        return v.Unit * minMagnitude
    end
    
    return v
end

-- Gives all possible points the boid can check to go to for avoiding obstacles
function solver.GetDirections(numPoints : number) : {Vector3}
    local points : {Vector3} = {}
	local goldenRatio = (1 + math.sqrt(5)) / 2
	local turnFraction = math.pi * 2 * goldenRatio
    
    for i = 0, numPoints - 1 do
		local t = i / numPoints
		local inclination = math.acos(1 - 2 * t)
		local azimuth = turnFraction * i

		local x = math.sin(inclination) * math.cos(azimuth)
		local y = math.sin(inclination) * math.sin(azimuth)
		local z = math.cos(inclination)

		table.insert(points, Vector3.new(-x, -y, -z))
	end

    return points
end

local directions = solver.GetDirections(1000)

function solver.FindUnobstructedDirection(boid) : Vector3
    local bestDir = boid.CFrame.LookVector
    local foundBest = false

    for i, _dir in directions do
        local dir : Vector3 = boid.CFrame:VectorToWorldSpace(_dir)
        local res = workspace:Raycast(boid.Position, _dir.Unit * maxViewRange, params)
        local isPathObstructed = res and res.Instance

        if not isPathObstructed and boid.CFrame.LookVector:Dot(dir) < 0 then
            bestDir = dir
            foundBest = true
            break
        end
    end

    return bestDir, foundBest
end

function solver.SteerTowards(v : Vector3, velocity : Vector3, maxSpeed : number)
    local newV = v.Unit * maxSpeed - velocity
    return clampMagnitude(newV, 0, maxSteeringSpeed)
end

function solver.Solve(dt : number, boids, target : Vector3)
    for _n, boid in boids do
        local force = Vector3.zero
        local dir = Vector3.zero

        local sumAdjPos = Vector3.zero
        local sumAdjDir = Vector3.zero
        local adjBoids = 0

        for _m, adjBoid in boids do
            local diff = adjBoid.Position - boid.Position
            local dist = diff.Magnitude
            if _n == _m or dist > maxNeighbourRadius then continue end
            -- if boid.Velocity:Dot(adjBoid.Velocity) < maxViewAngle then continue end
            
            if dist <= sepratingDistance then
                dir -= (diff / dist)
            end

            adjBoids += 1
            sumAdjPos += adjBoid.Position
            sumAdjDir += adjBoid.Velocity
        end

        if target then
            local diff = target - boid.Position
            local targetForce = solver.SteerTowards(diff, boid.Velocity, boid.MaxSpeed) / (adjBoids + 1) * targetFactor

            if not isNanVec3(targetForce) then
                force += targetForce
            end
        end

        local ray = workspace:Raycast(boid.Position, boid.CFrame.LookVector * maxViewRange, params)

        if ray then
            local unobstructedDir, foundBest = solver.FindUnobstructedDirection(boid)

            if foundBest then
                local unobstructedForce = solver.SteerTowards(unobstructedDir, boid.Velocity, boid.MaxSpeed) * obstacleSeprationFactor

                if not isNanVec3(unobstructedForce) then
                    force += unobstructedForce
                end
            end
        end

        if adjBoids > 0 then
            local avgPos = sumAdjPos / adjBoids
            local avgDir = sumAdjDir / adjBoids

            local offsetToCenter = avgPos - boid.Position

            local alignmentForce = solver.SteerTowards(avgDir, boid.Velocity, boid.MaxSpeed) * alignmentFactor
            local cohesionForce = solver.SteerTowards(offsetToCenter, boid.Velocity, boid.MaxSpeed) * cohesionFactor
            local seprationForce = solver.SteerTowards(dir, boid.Velocity, boid.MaxSpeed) * seprationFactor

            if not isNanVec3(cohesionForce) then
                force += cohesionForce
            end
            if not isNanVec3(alignmentForce) then
                force += alignmentForce
            end
            if not isNanVec3(seprationForce) then
                force += seprationForce
            end
        end

        if not isNanVec3(force) then
            boid:AddForce(force * dt)
        end
    end
end

return solver