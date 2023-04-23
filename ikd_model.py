
import torch
import torch.nn as nn
import torch.nn.functional as F

"""
This is the class for the Inverse Kinodynamic Model.

The model has 2 inputs: 
- The velocity of the controller, which we denote as v.
- The angular velocity from the IMU, which we denote as av'. This is the ground truth.
The model has 1 output:
- The velocity of the controller + the Angular Velocity of the controller, which we denoted as (v, av).

This neural network is a simple neural network with 2 hidden layers.
"""

class IKDModel(nn.Module):
    def __init__(self, dim_input, dim_output):
        super(IKDModel, self).__init__()
        self.l1 = nn.Linear(dim_input, 32)
        self.l2 = nn.Linear(32, 32)
        self.correction = nn.Linear(32, dim_output)

    def forward(self, input):
        x = F.relu(self.l1(input))
        x = F.relu(self.l2(x))
        x = self.correction(x)
        return x