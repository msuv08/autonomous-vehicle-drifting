# NOTE: this script assumes training data is merged and ready.
# Takes from IKDDATA2, which is what training script uses.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import torch
from scipy.io import loadmat
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
from ikd_training import IKDModel
# import plt
import matplotlib.pyplot as plt

# load in ikd data
data_name = './dataset/ikddata2.csv'

data = pd.read_csv(data_name)

# get joystick and executed data
joystick = np.array([eval(i) for i in data["joystick"]])
executed = np.array([eval(i) for i in data["executed"]])

# concatenate joystick and executed data
data = np.concatenate((joystick, executed), axis = 1)

# get length of data
N = len(executed)

curvatures = []

for i, row in enumerate(data):
    velocity = row[0]
    angular_velocity = row[1]
    true_angular_velocity = row[2]

    # get the curvature
    curvature = angular_velocity / velocity
    if curvature > 0.0:
        curvatures.append(curvature)
    else:
        curvatures.append(0.0)

print(curvatures)
# Bin the curvatures
bin_ranges = np.linspace(min(curvatures), max(curvatures), num=4)  # Change the number of bins by adjusting 'num'
hist, bin_edges = np.histogram(curvatures, bins=bin_ranges)

# Plot the histogram
plt.figure(figsize=(10, 5))
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="k", align="edge")
plt.xlabel("Curvature")
plt.ylabel("Frequency")
plt.title("Curvature Histogram of Training Data")
plt.show()


velocities = []
angular_velocities = []
true_angular_velocities = []

for i, row in enumerate(data):
    velocity = row[0]
    angular_velocity = row[1]
    true_angular_velocity = row[2]

    velocities.append(velocity)
    angular_velocities.append(angular_velocity)
    true_angular_velocities.append(true_angular_velocity)

# Bin the velocities
bin_ranges = np.linspace(min(velocities), max(velocities), num=10)  # Change the number of bins by adjusting 'num'
hist, bin_edges = np.histogram(velocities, bins=bin_ranges)

# Plot the histogram
plt.figure(figsize=(10, 5))
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="k", align="edge")
plt.xlabel("Velocity")
plt.ylabel("Frequency")

plt.title("Velocity Histogram of Training Data")
plt.show()

# Bin the angular velocities
bin_ranges = np.linspace(min(angular_velocities), max(angular_velocities), num=10)  # Change the number of bins by adjusting 'num'
hist, bin_edges = np.histogram(angular_velocities, bins=bin_ranges)

# Plot the histogram
plt.figure(figsize=(10, 5))
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="k", align="edge")
plt.xlabel("Angular Velocity")
plt.ylabel("Frequency")
plt.title("Angular Velocity Histogram of Training Data")
plt.show()

# Bin the true angular velocities
bin_ranges = np.linspace(min(true_angular_velocities), max(true_angular_velocities), num=10)  # Change the number of bins by adjusting 'num'
hist, bin_edges = np.histogram(true_angular_velocities, bins=bin_ranges)

# Plot the histogram
plt.figure(figsize=(10, 5))
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="k", align="edge")
plt.xlabel("True Angular Velocity")
plt.ylabel("Frequency")
plt.title("True Angular Velocity Histogram of Training Data")
plt.show()

# prediction stuff

model = IKDModel(2, 1)
model.load_state_dict(torch.load("./ikddata2.pt"))

new_av = []

for i, row in enumerate(data):
    velocity = row[0]
    angular_velocity = row[1]
    true_angular_velocity = row[2]

    input = torch.FloatTensor([velocity, angular_velocity])
    with torch.no_grad():
        output = model(input)
    av = output.item()
    new_av.append(new_av)


# Bin the new angular velocities
bin_ranges = np.linspace(min(new_av), max(new_av), num=10)  # Change the number of bins by adjusting 'num'
hist, bin_edges = np.histogram(new_av, bins=bin_ranges)

# Plot the histogram
plt.figure(figsize=(10, 5))
plt.bar(bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="k", align="edge")
plt.xlabel("New Angular Velocity")
plt.ylabel("Frequency")
plt.title("New Angular Velocity Histogram of Training Data")
plt.show()

    
