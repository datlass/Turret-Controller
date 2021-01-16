-- PID
--From AeroGameFramework by sleitnick here for testing purposes
-- August 11, 2020

--[[

	PID stands for Proportional-Integral-Derivative. One example of PID controllers in
	real-life is to control the input to each motor of a drone to keep it stabilized.
	Another example is cruise-control on a car.

	-----------------------------------------------

	Constructor:

		pid = PID.new(min, max, kP, kD, kI)


	Methods:

		pid:Calculate(dt, setpoint, pv)
		> Calculates and returns the new value
			> dt: DeltaTime
			> setpoint: The current point
			> pv: The process variable (i.e. goal)

		pid:Reset()
		> Resets the PID

	-----------------------------------------------

--]]

local VectorUtil = require(script.Parent.VectorUtil)

local PID = {}
PID.__index = PID

function PID.new(min, max, kp, kd, ki)
	local self = setmetatable({}, PID)
	self._min = min or -math.huge
	self._max = max or math.huge
	self._kp = kp
	self._kd = kd
	self._ki = ki
	self._preError = 0
	self._integral = 0
	self.VectorMode = false
	return self
end

function PID.newVectorMode(maxMagnitude, kp, kd, ki)
	local self = setmetatable({}, PID)
	self._max = maxMagnitude or math.huge -- doesn't need min only clamps magnitude
	self._kp = kp
	self._kd = kd
	self._ki = ki
	self._preError = Vector3.new()
	self._integral = Vector3.new()
	self.VectorMode = true -- uses Vector3 values for integral so it doesn't error
	return self
end


function PID:Reset()
	if not self.VectorMode then
		self._preError = 0
		self._integral = 0
	else
		self._preError = Vector3.new()
		self._integral = Vector3.new()
	end
end


function PID:Calculate(dt, setpoint, pv)
	local err = (setpoint - pv)
	local pOut = (self._kp * err)
	self._integral += (err * dt)
	local iOut = (self._ki * self._integral)
	local deriv = ((err - self._preError) / dt)
	local dOut = (self._kd * deriv)
	local output = math.clamp((pOut + iOut + dOut), self._min, self._max)
	self._preError = err
	return output
end

function PID:CalculateVector(dt, setpoint, pv)
	local err = (setpoint - pv)
	local pOut = (self._kp * err)
	self._integral += (err * dt)
	local iOut = (self._ki * self._integral)
	local deriv = ((err - self._preError) / dt)
	local dOut = (self._kd * deriv)
	local outputPreClamp = pOut + iOut + dOut
	local output = VectorUtil.ClampMagnitude(outputPreClamp,self._max)
	self._preError = err
	return output
end



function PID:SetMinMax(min, max)
	self._min = min
	self._max = max
end


return PID