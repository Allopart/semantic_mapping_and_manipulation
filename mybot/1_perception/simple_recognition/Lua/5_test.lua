require 'torch'
require 'image'
require 'optim'
require 'cunn'
require 'cutorch'


input=torch.Tensor(inputMat)
input = input:cuda()


for i = 1,(#input)[1] do
	input[i]=image.rgb2yuv(input[i])
end

for i=1,3 do
	input[{ {},i,{},{} }]:add(-mean[class][i])
	input[{ {},i,{},{} }]:div(std[class][i])
end

prediction = full_model:forward(input)


