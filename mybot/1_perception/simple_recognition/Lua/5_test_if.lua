-- original image & rotated image

require 'torch'
require 'image'
require 'optim'
require 'cunn'
require 'cutorch'

input=torch.Tensor(inputMat)
input = input:float()

numData = (#input)[1]

for i = 1,(#input)[1] do
	input[i]=image.rgb2yuv(input[i])
end


label = torch.Tensor((#input)[1]):fill(0)
input_r = input:transpose(3,4)
input = torch.cat(input, input_r, 1)

for i=1,3 do
	input[{ {},i,{},{} }]:add(-mean[class][i])
	input[{ {},i,{},{} }]:div(std[class][i])
end

input = input:cuda()

pred = model[class]:forward(input)

maxVal, maxIndex = torch.max(pred[{{},classNumber}], 1)

-- object가 lotated되었는지 아닌지를 판단. 

rotated=0
if (maxVal[1]>logProb) then
	if (maxIndex[1]<=numData) then 
		label[maxIndex[1]] = classNumber;
	else
		label[maxIndex[1]-numData] = classNumber;  
		rotated = 1
	end
end


