--for command bar thing

local targetModel = workspace.LowerBody

local modelDescendants = targetModel:GetDescendants()

local folder = Instance.new("Folder")
folder.Parent = targetModel
folder.Name = "Motor6DValues"

for i, descendant in pairs(modelDescendants) do
    if descendant:IsA("Motor6D") then
        local object = Instance.new("ObjectValue")
        object.Value = descendant
        object.Name = descendant.Name
        object.Parent = folder
    end
end

return nothing