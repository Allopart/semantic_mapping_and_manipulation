require 'torch'
require 'image'
require 'nn'

input = torch.Tensor(inputMat)
input = input:cuda()

for i = 1,(#input)[1] do
  input[i]=image.rgb2yuv(input[i])
end

mean[class] = {}
std[class] = {}

for i=1,3 do
	mean[class][i] = input[{ {},i,{},{} }]:mean()
	std[class][i] = input[{ {},i,{},{} }]:std()
	input[{ {},i,{},{} }]:add(-mean[class][i])
	input[{ {},i,{},{} }]:div(std[class][i])
end

label = torch.Tensor(labelMat)
label = label:cuda()

torch.save(paths.concat(class, "model", "lua_mean"), mean[class])
torch.save(paths.concat(class, "model", "lua_std"), std[class])
