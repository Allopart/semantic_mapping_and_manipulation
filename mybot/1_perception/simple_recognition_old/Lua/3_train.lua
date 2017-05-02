require 'torch'
require 'optim'
require 'cunn'

parameters, gradParameters = full_model:getParameters()	

-- This matrix records the current confusion accross classes
confusion = optim.ConfusionMatrix(className[class])

function train() 

	epoch = epoch or 1

	shuffle = torch.randperm((#input)[1])

	for t = 1, (#input)[1], batchSize[class] do
		local batchInputs = {}
		local batchLabels = {}
		for i = t, math.min(t+batchSize[class]-1, (#input)[1]) do
			local batchInput = input[shuffle[i]]
			local batchLabel = label[shuffle[i]]
--		        batchInput = batchInput:cuda()

			table.insert(batchInputs, batchInput)
			table.insert(batchLabels, batchLabel)
		end

		local feval = function(x)
			-- get new parameters
			if x~=parameters then 
				parameters:copy(x)
				-- parameters are copied from x
			end 

			-- reset gradients
			gradParameters:zero()

			-- f is the average of all criterions
			local f=0 
			
			for i=1, #batchInputs do
				-- estimate f
				local output = full_model:forward(batchInputs[i])
				local err = criterion:forward(output, batchLabels[i])
				f = f + err
				-- estimate df/dW
				local df_do = criterion:backward(output, batchLabels[i])


				full_model:backward(batchInputs[i], df_do) 

				confusion:add(output, batchLabels[i])
			end
			gradParameters:div(#batchInputs)
			f=f/(#batchInputs)

			return f, gradParameters
		end

		-- optimize on current mini-batch
		optimMethod = optim.sgd
		-- optim.sgd(feval, parameters, ptimState)
		optimMethod(feval, parameters, optimState)

	end

	print (confusion)
	confusion:zero()
	epoch = epoch + 1
end

startTime = os.time()
full_model:training()

for i = 1,maxIter[class] do
    train()
    remainedIter = maxIter[class]-i
    elapsedTime = os.time() - startTime
    remainedTime = elapsedTime*remainedIter/i

	nHours = string.format("%02.f", math.floor(remainedTime/3600)); 
	nMins = string.format("%02.f", math.floor(remainedTime/60-(nHours*60))); 
	nSecs = string.format("%02.f", math.floor(remainedTime-nHours*3600-nMins*60)); 

    print(" + remained "..nHours..":"..nMins..":"..nSecs.." ["..i.."/"..maxIter[class].."]")
end

full_model:evaluate()
print (class)
print (paths.concat(class, "model", "nn_model"))
torch.save(paths.concat(class, "model", "nn_model"), model[class])

print('train.lua finished')

