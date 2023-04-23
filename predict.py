import numpy as np
import torch
from scipy.io import loadmat
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
from ikd_training import IKDModel
# import plt
import matplotlib.pyplot as plt

# model = IKDModel(2, 1)
# model.load_state_dict(torch.load("./ikddata2.pt"))
# data_name = './dataset/ikddata2.csv'
# data = pd.read_csv(data_name)
# joystick = np.array([eval(i) for i in data["joystick"]])
# executed = np.array([eval(i) for i in data["executed"]])
# data = np.concatenate((joystick, executed), axis = 1)
# N = len(executed)
# N_train = N // 10 * 9
# N_test = N - N_train
# idx_train = np.random.choice(N, N_train, replace = False)
# mask = np.ones(N, dtype=bool)
# mask[idx_train] = False
# data_train = data[np.invert(mask),]
# data_test = data[mask,]

# outputs = []
# correct = []
# time = []



# # iterate over all training data, and make predictions accordingly.
# for i in range(len(data_train)):
#     time.append(i)
#     d = data_train[i]
#     v = d[0]
#     av = d[1]
#     true_av = d[2]
#     input = torch.FloatTensor([v, true_av])
#     with torch.no_grad():
#         output = model(input)
#     print("------------------------------------------------")
#     print("Our Input:", torch.FloatTensor([v, true_av]))
#     print("Our Velocity:", output.item())
#     print("Correct Output:", av)
#     print("-------------------------------------------------")

#     correct.append(av)
#     outputs.append(output.item())

# inputs = torch.FloatTensor(correct)
# outputs = torch.FloatTensor(outputs)

# fig, ax = plt.subplots()
# ax.scatter(time, outputs, label='Output')
# ax.scatter(time, inputs, label='Correct Output')
# ax.legend()
# ax.set_xlabel('Time')
# ax.set_ylabel('Predicted Joystick AV and Actual Joystick AV')
# plt.show()

model = IKDModel(2, 1)
model.load_state_dict(torch.load("./ikddata2.pt"))
# make input tensor
radius = float(1 / 0.57)
av = float(2.0 / radius)
input = torch.FloatTensor([2.0, av])
# make prediction
with torch.no_grad():
    output = model(input)
print("------------------------------------------------")
print("Our Input:", input)
print("Our Angular Velocity Prediction:", output.item() / radius)
print("-------------------------------------------------")
