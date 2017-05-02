require 'torch'
require 'cunn'

mean[class] = torch.load(paths.concat(class, "model", "lua_mean"))
std[class] = torch.load(paths.concat(class, "model", "lua_std"))
model[class] = torch.load(paths.concat(class, "model", "nn_model"))

full_model = nn.Sequential()

full_model:add(model[class])
full_model:add(nn.Exp())
full_model:add(nn.AddConstant(1000))
full_model:add(nn.Log())
full_model:add(nn.LogSoftMax())
full_model:cuda()

full_model:evaluate()


print ("model loaded")
