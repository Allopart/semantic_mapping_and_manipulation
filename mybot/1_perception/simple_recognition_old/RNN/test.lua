require 'torch'
require 'rnn'
require 'cunn'
require 'gnuplot'

numObj=6
--numAct=5d

--trajectory = torch.CudaTensor{0,0,0,1,0,0,1,0,0,-0.560684, 0.853306, -0.1115} 
trajectory = torch.CudaTensor(trajectory) 

result = {}
table.insert(result, trajectory)

context = trajectory[{{1,numObj}}]

for i = 1,24 do

  out = rnnmodel:forward(trajectory)

  trajectory = torch.cat(context, out) 
  table.insert(result, trajectory)
end

rnnmodel:forget()

