--[[
    Testing with dummy with only one motor but both elevation and yaw constraints
]]

local RunService = game:GetService("RunService")
local ReplicatedStorage = game:GetService("ReplicatedStorage")

local TurretController = require(ReplicatedStorage:FindFirstChild("TurretController",true))

local model = workspace.OneMotorDummy
local Target = workspace.Target

local Motor6DValues = model.Motor6DValues:GetChildren()
local ModelMotor6Ds = {}
for i,v in pairs (Motor6DValues) do
    ModelMotor6Ds[v.Name] = v.Value
end

local gunHeadConstraints = {
	["YawLeft"] = 89;
	["YawRight"] = 89;
	["ElevationAngle"] = 45;
	["DepressionAngle"] = 45;
}

local GunHead = TurretController.new(ModelMotor6Ds["GunHead"],gunHeadConstraints)

RunService.Heartbeat:Connect(function(step)
    local goalPos = Target.Position
	GunHead:LookAt(goalPos,step)
end)
