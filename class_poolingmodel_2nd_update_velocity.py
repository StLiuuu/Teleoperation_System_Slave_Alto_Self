import numpy as np
import control.matlab as control
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim
import scipy.io
import matplotlib
from torch.utils.data import Dataset, DataLoader
import math
import matplotlib.animation as animation
from IPython.display import HTML


class PoolingModel(nn.Module):
    
    def __init__(self, feature_size, hidden_size, action_dim):
        
        super(PoolingModel, self).__init__()
        
        # dimensionality of state features
        self.feature_size = feature_size
        
        # hidden layer dimensionalities
        self.hidden_size = hidden_size
        
        # dimensionality of action space
        self.action_dim = action_dim
        
        # encodes state-action pairs into feature vectors
        self.encoder1 = nn.Sequential(
            nn.Linear(self.feature_size, self.hidden_size),
            nn.Softplus(),
            nn.Linear(self.hidden_size, self.hidden_size),
            nn.Softplus(),
            nn.Linear(self.hidden_size, self.hidden_size),
        )

        
        # mean pooling over trajectory features
        self.pooling_layer = lambda x: x.sum(dim=-2)
        
        self.running_pooling_layer = lambda x: x.cumsum(dim=-2)
        
        self.max_seq_len = 5000
        
        self.division_mask = torch.tensor(range(1, self.max_seq_len + 1), dtype=torch.float32).reshape(1, -1, 1)
        
        # maps pooled representations to trajectory features
        self.encoder2 = nn.Sequential(
            nn.Linear(self.hidden_size + 9, self.hidden_size),
            nn.Softplus(),
            nn.Linear(self.hidden_size, self.hidden_size),
            nn.Softplus()
        )
        
        self.mu = nn.Linear(self.hidden_size, action_dim)
        
        self.log_sigma = nn.Linear(self.hidden_size, action_dim)

    
    def forward(self, input_traj, delta_t=0.01253533):
        
        # generate predictions for each time step
        batch_size, time_steps, state_size = input_traj.shape
        
        # forward pass
        outputs = self.encoder1(input_traj)
        
        # mean pooling
        outputs = self.running_pooling_layer(outputs)
        
        division_mask = self.division_mask[:, :outputs.shape[-2], :]
        outputs = outputs / division_mask
        
        velocities = input_traj[:, 1:, :] - input_traj[:, :-1, :]
        input_vels = torch.cat([torch.zeros_like(velocities[:, :1, :]), velocities], dim=1)
        input_vels = input_vels * 5
        
        # estimate acceleration
        accelerations = input_vels[:, 1:, :] - input_vels[:, :-1, :]
        input_accs = torch.cat([torch.zeros_like(accelerations[:, :1, :]), accelerations], dim=1)
        
        # estimate jerk
        jerks = input_accs[:, 1:, :] - input_accs[:, :-1, :]
        input_jerks = torch.cat([torch.zeros_like(jerks[:, :1, :]), jerks], dim=1)
        
        normalised_time = torch.linspace(0, 5, time_steps).unsqueeze(0).unsqueeze(-1).repeat(batch_size, 1, 1)
        
        enc2_inputs = torch.cat([outputs, input_traj, input_vels, input_accs, input_jerks, normalised_time], dim=-1)
        outputs = self.encoder2(enc2_inputs)
        vel_preds = self.mu(outputs)
        next_state_preds = input_traj[:, :-1, :] + delta_t * vel_preds[:, :-1, :]
        
        #return next_state_preds
        return vel_preds