require 'torch'
require 'rnn'
require 'cunn'
require 'gnuplot'

rho=2
updateInterval=rho-1
lr=0.01
--numAct = 5 
numObj = 6 

function gradientUpgrade(model, x, y, criterion, criterion_t, lr, iter)
 
  if iter>10000 then lr=0.001 end

	local prediction = model:forward(x)

--  local err = criterion_t:forward(prediction[2], y[2])
 	
  local err = criterion:forward(prediction,y)
	local gradOutputs = criterion:backward(prediction,y)

	model:backward(x, gradOutputs)
	if iter%updateInterval==0 then
		model:backwardThroughTime()
		model:updateParameters(lr)
		model:zeroGradParameters()
	end

	return err
end


for i = 1,#trajectories do
	trajectories[i]=torch.CudaTensor(trajectories[i])
  print(#trajectories)
end


-- make model
model = nn.Sequential()
model:add(nn.Recurrent(320, nn.Linear(numObj+3,320), nn.Linear(320,320), nn.RReLU(), rho))
model:add(nn.Dropout(0.5))

model:add(nn.Linear(320,320)) 
model:add(nn.RReLU())
model:add(nn.Dropout(0.5))


--[[
  sub = nn.ConcatTable()
    ssub = nn.Sequential()
    ssub:add(nn.Linear(320, numAct))
    ssub:add(nn.LogSoftMax())

  sub:add(ssub)
  sub:add(nn.Linear(320, 3))

model:add(sub) 
--model:add(nn.JoinTable(2))
--]]

model:add(nn.Linear(320,3))
model:cuda()
criterion = nn.MSECriterion()
--[[
criterion = nn.ParallelCriterion()
criterion:add(nn.ClassNLLCriterion(),0.01)
criterion:add(nn.MSECriterion())
--]]
criterion:cuda()

--criterion_t = nn.MSECriterion() 
--criterion_t:cuda()


function train(iter) 
	shuffle = torch.randperm(#trajectories)

	local sum_err_iter=0
  for i=1,#trajectories do
		local sum_err=0

		for j=1,(#trajectories[shuffle[i]])[1]-updateInterval do
      local inputs = trajectories[shuffle[i]][{{j},{}}]
      local outputs = trajectories[shuffle[i]][{{j+1},{numObj+1,numObj+3}}]
        
--        local output1 = trajectories[shuffle[i]][j+1][{{numObj+1,numObj+numAct}}]
--        a, output1 = torch.max(output1, 1)
        
--        local output2 = trajectories[shuffle[i]][{{j+1}, {numObj+numAct+1,numObj+numAct+3}}]

        -- maxindex를 집어넣자. 
--      local outputs = {output1, output2}     

  	local err = gradientUpgrade(model, inputs, outputs, criterion, criterion_t, lr,iter)
    sum_err = sum_err + err
		end

		sum_err_iter = sum_err_iter + sum_err/((#trajectories[shuffle[i]])[1]-updateInterval)
    model:forget()
	end

	return sum_err_iter/(#trajectories)

end

startTime = os.time()
maxIter = 30000
totalErr = torch.Tensor(maxIter)
model:training()

for iter=1,maxIter do

	totalErr[iter] = train(iter)
	remainedIter = maxIter-iter
	elapsedTime = os.time()-startTime
	remainedTime = elapsedTime*remainedIter/iter
	nHours = string.format("%02.f", math.floor(remainedTime/3600))
	nMins = string.format("%02.f", math.floor(remainedTime/60-(nHours*60)))
	nSecs = string.format("%02.f", math.floor(remainedTime-nHours*3600-nMins*60))

	print(" + remained "..nHours..":"..nMins..":"..nSecs.." ["..iter.."/"..maxIter.."] Error="..totalErr[iter])
end

gnuplot.plot('Error', totalErr, '~')
gnuplot.title('RNN convergence')
gnuplot.xlabel('Iteration')
gnuplot.ylabel('Mean square error')

-- model save!!
torch.save('/home/yongho/Demonstration/model', model)
