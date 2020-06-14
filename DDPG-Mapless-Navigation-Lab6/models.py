import numpy as np
import random

import torch
import torch.nn as nn
import torch.nn.functional as F

# TODO(Lab-02): Complete the network model.
class PolicyNet(nn.Module):
	def __init__(self):
		super(PolicyNet, self).__init__()
		self.fc1 = nn.Linear(23,512)
		self.fc2 = nn.Linear(512,512)
		self.fc3 = nn.Linear(512,512)
		self.fc4 = nn.Linear(512,2)
		self.relu = nn.ReLU(inplace=True)
		self.tanh = nn.Tanh()

	def forward(self, s):
		output = self.relu(self.fc1(s)) 
		output = self.relu(self.fc2(output)) 
		output = self.relu(self.fc3(output))
		output = self.tanh(self.fc4(output))	
		return output

class QNet(nn.Module):
	def __init__(self):
		super(QNet, self).__init__()
		self.fc1 = nn.Linear(23,512)
		self.fc2 = nn.Linear(514,512)
		self.fc3 = nn.Linear(512,512)
		self.fc4 = nn.Linear(512,1)
		self.relu = nn.ReLU(inplace=True)

    
	def forward(self, s, a):
		output = self.relu(self.fc1(s)) 
		output = torch.cat([output,a],1)
		output = self.relu(self.fc2(output)) 
		output = self.relu(self.fc3(output))
		output = self.fc4(output)
		return output
