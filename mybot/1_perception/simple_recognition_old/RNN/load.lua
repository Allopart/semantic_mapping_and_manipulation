require 'torch'
require 'rnn'
require 'cunn'

rnnmodel = torch.load('/home/yongho/Demonstration/model') 
rnnmodel:evaluate()

print("rnn model Loaded")
