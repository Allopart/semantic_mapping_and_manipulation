require 'torch'
require 'nn'
require 'optim'
require 'cunn'

-- input dimensions
ninputs = (#input)[2]*(#input)[3]*(#input)[4]
-- 32 by 32 by 3
nhiddens = 1000

full_model = nn.Sequential()

model[class] = nn.Sequential()
model[class]:add(nn.Reshape(ninputs))
model[class]:add(nn.Linear(ninputs,nhiddens))
model[class]:add(nn.RReLU())
model[class]:add(nn.Dropout(0.5))
--model[class]:add(nn.Tanh())
model[class]:add(nn.Linear(nhiddens,nhiddens))
model[class]:add(nn.RReLU())
model[class]:add(nn.Dropout(0.5))
--model[class]:add(nn.Tanh())
model[class]:add(nn.Linear(nhiddens,nhiddens))
model[class]:add(nn.RReLU())
model[class]:add(nn.Dropout(0.5))
--model[class]:add(nn.Tanh())
model[class]:add(nn.Linear(nhiddens,nhiddens))
model[class]:add(nn.RReLU())
--model[class]:add(nn.Tanh())
model[class]:add(nn.Linear(nhiddens,numClass[class]))

full_model:add(model[class])
full_model:add(nn.LogSoftMax())
--model[class]:add(nn.LogSoftMax())

criterion = nn.ClassNLLCriterion()

--model[class]:cuda()
full_model:cuda()
criterion:cuda()

optimState = {
	learningRate = 1e-3,
  learningRateDecay=5e-7,
	weightDecay = 0.0001,
	momentum = 0.5,
	optimMethod = optim.sgd
}

