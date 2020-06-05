import numpy as np
import random

import torch
import torch.nn as nn
import torch.nn.functional as F

# TODO(Lab-02): Complete the network model.
class PolicyNet(nn.Module):
    def __init__(self):
        super(PolicyNet, self).__init__()

    def forward(self, s):
        pass

class QNet(nn.Module):
    def __init__(self):
        super(QNet, self).__init__()
    
    def forward(self, s, a):
        pass
