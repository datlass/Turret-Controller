-- Joint Controller
-- Dthecoolest
-- December 10, 2020

local EMPTYCFRAME = CFrame.new()

local TurretController = {}
TurretController.__index = TurretController

local VectorUtil = require(script.VectorUtil)

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

	self.LerpMotors = true
	self.ConstantSpeed = true

	--some arbritrary values feel free to change
	self.LerpAlpha = 1/4
	self.AngularSpeed = math.rad(180)

	return self
end

function TurretController:LookAt(lookAtPosition:Vector3,step)

	--Constants from self
	local currentJointMotor6D = self.JointMotor6D
	local turretBase = self.TurretBase
	local originalC0Position = self.JointMotor6DC0Store.Position
	local turretInfo = self.TurretInfo

	local turretPosition
	local turretCFrameRotationOffset
	if turretInfo:IsA("Attachment") then
		turretPosition = turretInfo.WorldPosition
		turretCFrameRotationOffset = turretInfo.CFrame-turretInfo.CFrame.Position
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
	local lookAtWorld = CFrame.lookAt(turretPosition,lookAtPosition,baseCFrame.UpVector)


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

-- negative z is front, x is rightvector
function TurretController:EulerClampXY(x,y)
	local Constraints =  self.Constraints
	--there is an issue when y is between -179 to 180 degrees
	local newY = math.clamp(math.deg(y),-Constraints.YawRight,Constraints.YawLeft)
	local newX = math.clamp(math.deg(x),-Constraints.DepressionAngle, Constraints.ElevationAngle)
	return math.rad(newX), math.rad(newY)
end

return TurretController