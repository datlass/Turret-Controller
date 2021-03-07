-- Joint Controller
-- Dthecoolest
-- December 10, 2020

local EMPTYCFRAME = CFrame.new()
local ZEROVECTOR = Vector3.new()

local RunService = game:GetService("RunService")

local TurretController = {}
TurretController.__index = TurretController

local VectorUtil = require(script.VectorUtil)
local PID = require(script.PID)

--The table
--Units are in degrees
local ConstraintsTemplate = {
	["YawLeft"] = 5;
	["YawRight"] = 5;
	["ElevationAngle"] = 40;
	["DepressionAngle"] = 40;
}

--[[---------------------------------------------------------
	Joint attachment is required if the part 1 is offset make it equal to the Orientation of the Part 1 of the motor6D initially when
	facing forwards

	Base Attachment is defines constraints axis
]]
function TurretController.new(JointMotor6D : Motor6D, Constraints)

	local self = setmetatable({}, TurretController)
	self.JointMotor6D = JointMotor6D
	self.JointMotor6DC0Store = JointMotor6D.C0
	self.TurretInfo = JointMotor6D.Part1:FindFirstChild("TurretAttachment") or JointMotor6D.Part1
	self.TurretBase = JointMotor6D.Part0:FindFirstChild("BaseAttachment") or JointMotor6D.Part0:FindFirstChild("TurretAttachment")  or JointMotor6D.Part0

	self.Constraints = Constraints
	if Constraints.YawLeft == 0 and Constraints.YawRight ==0 then
		self.NoYaw = true
	else
		self.NoYaw = false
	end

	if Constraints.ElevationAngle == 0 and Constraints.DepressionAngle ==0 then
		self.NoPitch = true
	else
		self.NoPitch = false
	end

	self.LerpMotors = true
	self.ConstantSpeed = true

	--some arbritrary values feel free to change any time doing run time
	self.LerpAlpha = 1/4
	self.AngularSpeed = math.rad(90) -- some random value idk

	--PID mode settings
	self.UsePIDLookAt = false
	self:ConstructPIDController(math.rad(180),nil,nil,nil) -- creates PIDcontroller with the default settings
	self.Mass = 1 --makes it slower and feel bulkier
	self.CurrentAngularVelocityClamp = math.rad(45) --Clamps the angular velocity, number in radians per second
	self.Restitution = 0.5 -- Value 0-1, 1 means elastic same amount of bounce, 0 means rigid no bounce

	self.CurrentAngularVelocity = Vector3.new() -- Don't edit unless you want to control your own angular velocity

	--Uhh don't edit these variables are used for EulerClamp function to detect which axis has been clamped
	self.HasBeenClamped = false --detect if lookAt has been clamped or not for PID mode
	self.YawEdgeClamped = false
	self.ElevationEdgeClamped = false

	return self
end

function TurretController:ConstructPIDController(maxAngularAcceleration,kP, kD, kI)
	local accel = maxAngularAcceleration  --clamps output acceleration radians per second, if no input then it's math.huge
	local kp = kP or 0.5
	local kd = kD or 0.1
	local ki = kI or 0.2
	self.PIDController = PID.newVectorMode(accel,kp,kd,ki)
end

function TurretController:LookAt(lookAtPosition:Vector3,step)
	self.CurrentLookAtPosition = lookAtPosition
	if self.UsePIDLookAt then
		self:PIDLookAt(lookAtPosition, step)
		return
	end
	--Constants from self
	local currentJointMotor6D = self.JointMotor6D
	local turretBase = self.TurretBase
	local originalC0Position = self.JointMotor6DC0Store.Position
	local turretInfo = self.TurretInfo

	local turretPosition
	local turretCFrameRotationOffset
	if turretInfo:IsA("Attachment") then
		turretPosition = turretInfo.WorldPosition
		turretCFrameRotationOffset = (turretInfo.CFrame-turretInfo.CFrame.Position):Inverse()
	else--It's a Part
		turretPosition = turretInfo.Position
		turretCFrameRotationOffset =  EMPTYCFRAME 
	end

	local baseCFrame : CFrame
	if turretBase:IsA("Attachment") then
		baseCFrame = turretBase.WorldCFrame-turretBase.WorldCFrame.Position
	else
		baseCFrame = turretBase.CFrame-turretBase.CFrame.Position
	end

	local relativeToWorld = currentJointMotor6D.Part0.CFrame:Inverse()
	local lookAtWorld = CFrame.lookAt(turretPosition,lookAtPosition,baseCFrame.UpVector)--goal LookAt CFrame

	local goalCFrame

	if self.Constraints then
		local turretRelativeCF = baseCFrame:ToObjectSpace(lookAtWorld)
		local x , y , z = turretRelativeCF:ToOrientation()
		--print(math.deg(x),math.deg(y),math.deg(z))
		local constrainedX , constrainedY = self:EulerClampXY(x,y)

		--Detect quadrant of lookAt position
		local jointPosition = currentJointMotor6D.Part0.CFrame*originalC0Position
		local quadrantLookAtFromJointPosition = CFrame.lookAt(jointPosition,lookAtPosition,baseCFrame.UpVector)	
		local baseRelative = baseCFrame:ToObjectSpace(quadrantLookAtFromJointPosition)
		local _,y, _ = baseRelative:ToOrientation()
		constrainedY = math.abs(constrainedY)*math.sign(y)--use the quadrants of the lookAtFromJoint
		--print(math.deg(constrainedX),math.deg(constrainedY))
		--print(math.deg(constrainedY))
		goalCFrame = relativeToWorld*baseCFrame*CFrame.fromOrientation(constrainedX,constrainedY,z)*turretCFrameRotationOffset
	else
		goalCFrame = relativeToWorld*lookAtWorld*turretCFrameRotationOffset
	end

	--For Lerping
	local goalRotationCFrame = goalCFrame-goalCFrame.Position
	local currentRotation = currentJointMotor6D.C0-currentJointMotor6D.C0.Position

	self.CurrentRotationLookAt = currentRotation
	self.GoalRotationLookAt = goalRotationCFrame
	--if the bool is true then use lerp
	if self.LerpMotors then
		
		local adjustedLerpAlpha
		if step and self.ConstantSpeed then
			local angularDistance = VectorUtil.AngleBetween(currentRotation.LookVector,goalRotationCFrame.LookVector)
			local estimatedTime = self.AngularSpeed/angularDistance
			adjustedLerpAlpha = math.min(step*estimatedTime,1)
		elseif step then
			adjustedLerpAlpha = step
		end

		local newRotationCF = currentRotation:lerp(goalRotationCFrame, adjustedLerpAlpha or self.LerpAlpha)

		self.JointMotor6D.C0 = CFrame.new(originalC0Position)*newRotationCF
	else
		self.JointMotor6D.C0 = CFrame.new(originalC0Position)*goalRotationCFrame
	end
end

--[[
	uses a different rotation method, step is mandatory here as PID needs dt for the D error
]]
function TurretController:PIDLookAt(lookAtPosition:Vector3,step)
	--Constants from self
	local currentJointMotor6D = self.JointMotor6D
	local turretBase = self.TurretBase
	local originalC0Position = self.JointMotor6DC0Store.Position
	local turretInfo = self.TurretInfo

	local turretPosition
	local turretCFrameRotationOffset
	if turretInfo:IsA("Attachment") then
		turretPosition = turretInfo.WorldPosition
		turretCFrameRotationOffset = (turretInfo.CFrame-turretInfo.CFrame.Position):Inverse()
	else--It's a Part
		turretPosition = turretInfo.Position
		turretCFrameRotationOffset =  EMPTYCFRAME 
	end

	local baseCFrame : CFrame
	if turretBase:IsA("Attachment") then
		baseCFrame = turretBase.WorldCFrame-turretBase.WorldCFrame.Position
	else
		baseCFrame = turretBase.CFrame-turretBase.CFrame.Position
	end

	local relativeToWorld = currentJointMotor6D.Part0.CFrame:Inverse()
	local lookAtWorld = CFrame.lookAt(turretPosition,lookAtPosition,baseCFrame.UpVector)--goal LookAt CFrame

	local goalCFrame

	if self.Constraints then
		local turretRelativeCF = baseCFrame:ToObjectSpace(lookAtWorld)
		local x , y , z = turretRelativeCF:ToOrientation()
		--print(math.deg(x),math.deg(y),math.deg(z))
		local constrainedX , constrainedY = self:EulerClampXY(x,y)

		--Detect quadrant of lookAt position
		local jointPosition = currentJointMotor6D.Part0.CFrame*originalC0Position
		local quadrantLookAtFromJointPosition = CFrame.lookAt(jointPosition,lookAtPosition,baseCFrame.UpVector)	
		local baseRelative = baseCFrame:ToObjectSpace(quadrantLookAtFromJointPosition)
		local _,y, _ = baseRelative:ToOrientation()
		constrainedY = math.abs(constrainedY)*math.sign(y)--use the quadrants of the lookAtFromJoint

		--print(math.deg(constrainedX),math.deg(constrainedY))
		--print(math.deg(constrainedY))
		goalCFrame = relativeToWorld*baseCFrame*CFrame.fromOrientation(constrainedX,constrainedY,z)*turretCFrameRotationOffset
	else
		goalCFrame = relativeToWorld*lookAtWorld*turretCFrameRotationOffset
	end

	--For Lerping but with PID
	local goalRotationCFrame = goalCFrame-goalCFrame.Position
	local currentRotation = currentJointMotor6D.C0-currentJointMotor6D.C0.Position

	--apply angular velocity rotation
	local differenceCFrame = currentRotation:ToObjectSpace(goalRotationCFrame)
	local axis, angle = differenceCFrame:ToAxisAngle()
	local angularDisplacementDifference = axis*angle
	local outputAcceleration = self.PIDController:CalculateVector(step,ZEROVECTOR,angularDisplacementDifference)
	self.CurrentAngularVelocity -= outputAcceleration * step/self.Mass

	local clampMagnitude = self.CurrentAngularVelocityClamp
	if clampMagnitude then
		self.CurrentAngularVelocity = VectorUtil.ClampMagnitude(self.CurrentAngularVelocity,clampMagnitude)
	end

	local applyCurrentAngularVelocity = self.CurrentAngularVelocity
	local magnitudeCurrentAngularVelocity = applyCurrentAngularVelocity.Magnitude
	local axisCurrentAngularVelocity = applyCurrentAngularVelocity.Unit

	local cframeCurrentAngularVelocity
	-- if there is a rotation or else it's a .Unit Nil error
	if axisCurrentAngularVelocity == axisCurrentAngularVelocity then 
		cframeCurrentAngularVelocity = CFrame.fromAxisAngle(axisCurrentAngularVelocity,magnitudeCurrentAngularVelocity)
	else
		cframeCurrentAngularVelocity = CFrame.new()
	end

	self.JointMotor6D.C0 *= cframeCurrentAngularVelocity

	--check if the angular velocity goes over the boundries/hits the dead end
	local currentTurretCFrame : CFrame
	if turretInfo:IsA("Attachment") then
		currentTurretCFrame = turretInfo.WorldCFrame
	else--It's a Part
		currentTurretCFrame =  self.JointMotor6D.Part1.CFrame 
	end

	--check constraints again
	if self.Constraints then
		local turretRelativeCF = baseCFrame:ToObjectSpace(currentTurretCFrame)
		local x , y , z = turretRelativeCF:ToOrientation()
		local constrainedX , constrainedY = self:EulerClampXY(x,y)
		--Detect quadrant of lookAt position
		--Don't do it for Checking constraints or else quadrant will just flip unexpectedly????
		
		if self.HasBeenClamped then -- hit the edges
			--print("Yeah")
			if self.Restitution then
				--print(self.NoYaw, self.ElevationEdgeClamped)
				if self.NoYaw and self.ElevationEdgeClamped then -- only elevation up and down
					--print("bounce") -- test out if it bounces or not :P
					self.CurrentAngularVelocity *= -self.Restitution
					local newgoalCFrame = relativeToWorld*baseCFrame*CFrame.fromOrientation(constrainedX,constrainedY,z)*turretCFrameRotationOffset
					local goal = newgoalCFrame - newgoalCFrame.Position
					self.JointMotor6D.C0 = CFrame.new(originalC0Position)*goal -- restrain it within goal
				end
				if self.NoPitch and self.YawEdgeClamped then -- only elevation up and down/ pitch rotation
					self.CurrentAngularVelocity *= -self.Restitution
					local newgoalCFrame = relativeToWorld*baseCFrame*CFrame.fromOrientation(constrainedX,constrainedY,z)*turretCFrameRotationOffset
					local goal = newgoalCFrame - newgoalCFrame.Position
					self.JointMotor6D.C0 = CFrame.new(originalC0Position)*goal -- restrain it within goal
				end
				if not self.NoPitch and not self.NoYaw then -- Doesn't match the no yaw and no pitch criteria
					self.CurrentAngularVelocity *= -self.Restitution
					local newgoalCFrame = relativeToWorld*baseCFrame*CFrame.fromOrientation(constrainedX,constrainedY,z)*turretCFrameRotationOffset
					local goal = newgoalCFrame - newgoalCFrame.Position
					self.JointMotor6D.C0 = CFrame.new(originalC0Position)*goal -- restrain it within goal
				end
			end
		end
	end

end

function TurretController:LookAtForSecondsThenDisable(finalLookAtPosition:Vector3, timeLength, replicationFunction)
	local totalTime = 0
	local totalDesiredRunTime = timeLength or 5
	local lookAtPosition = finalLookAtPosition or self.CurrentLookAtPosition

	local replicationTime = 0
	local replicationInterval = 0.2


	self.LookAtConnection = RunService.Heartbeat:Connect(function(step)
		totalTime += step
		if totalTime > totalDesiredRunTime then
			if self.LookAtConnection then
				self.LookAtConnection:Disconnect()
				self:DisableLocalControl()
			end
		end
		replicationTime += step
		if replicationTime > replicationInterval then
			replicationTime = 0
			replicationFunction()
		end
		self:LookAt(lookAtPosition,step)
	end)
end

function TurretController:EnableLocalControl()
	if self.LookAtConnection then
		self.LookAtConnection:Disconnect()
	end

	if not self.LocalControl then
		self.ServerJointMotor6D = self.JointMotor6D
		local originalTurretMotor = self.JointMotor6D
		originalTurretMotor.Enabled = false
		local turretMotorClone = self.JointMotor6D:Clone()
		turretMotorClone.Enabled = true
		turretMotorClone.Parent = originalTurretMotor.Parent
		self.JointMotor6D = turretMotorClone
		self.LocalControl = true
	else
		--warn("Local control already enabled")
	end
end

function TurretController:DisableLocalControl()
	if 	self.LocalControl then
		self.JointMotor6D:Destroy()
		self.ServerJointMotor6D.Enabled = true
		self.JointMotor6D = self.ServerJointMotor6D
		self.LocalControl = false
	else
		warn("Local control already disabled")
	end
end

function TurretController:GetLocalC0Orientation()
	if self.ServerJointMotor6D and self.LocalControl then
		return self.ServerJointMotor6D, self.JointMotor6D.C0:ToOrientation()
	end
end
-- negative z is front, x is rightvector
function TurretController:EulerClampXY(x,y)
	local Constraints =  self.Constraints
	--there is an issue when y is between -179 to 180 degrees
	local degY = math.deg(y)
	local degX = math.deg(x)
	local newY = math.clamp(degY,-Constraints.YawRight,Constraints.YawLeft)
	local newX = math.clamp(degX,-Constraints.DepressionAngle, Constraints.ElevationAngle)

	local hasBeenClamped = false
	local yawEdgeClamped = false
	local elevationEdgeClamped = false

	if degY < -Constraints.YawRight or degY > Constraints.YawLeft then
		hasBeenClamped = true
		yawEdgeClamped = true
	end
	if degX < -Constraints.DepressionAngle or degX > Constraints.ElevationAngle then
		hasBeenClamped = true
		elevationEdgeClamped = true
	end

	self.YawEdgeClamped = yawEdgeClamped
	self.ElevationEdgeClamped = elevationEdgeClamped
	self.HasBeenClamped = hasBeenClamped

	return math.rad(newX), math.rad(newY)
end

return TurretController