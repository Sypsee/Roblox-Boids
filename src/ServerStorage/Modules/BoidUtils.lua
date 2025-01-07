local utils = {}

function utils.isNan(n : number) : boolean
    return n ~= n
end

function utils.isNanVec3(v : Vector3) : boolean
    return utils.isNan(v.X) or utils.isNan(v.Y) or utils.isNan(v.Z)
end

function utils.clampMagnitude(v : Vector3, minMagnitude : number, maxMagnitude : number)
    if v.Magnitude > maxMagnitude then
        return v.Unit * maxMagnitude
    elseif v.Magnitude < minMagnitude then
        return v.Unit * minMagnitude
    end
    
    return v
end

-- Gives all possible points the boid can check to go to for avoiding obstacles
function utils.GetDirections(numPoints : number) : {Vector3}
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

return utils