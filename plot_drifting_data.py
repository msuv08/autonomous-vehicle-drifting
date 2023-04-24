import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import torch
from ikd_training import IKDModel

# load in the drifting data from the "drift6.csv" file
data = pd.read_csv("./drift6.csv")
joystick = np.array([eval(i) for i in data["joystick"]])
executed = np.array([eval(i) for i in data["executed"]])
drifting_data = np.concatenate((joystick, executed), axis = 1)

# plot the data, showing a line for the actual joystick AV and a line for the predicted joystick AV

actual_av = []
joystick_av = []
time = []
model = IKDModel(2, 1)
model.load_state_dict(torch.load("./ikddata2.pt"))
corrected_av = []

for i, row in enumerate(drifting_data):
    time.append(i)
    actual_av.append(row[0])
    joystick_av.append(row[1])
    input = torch.FloatTensor([row[0], row[1]])
    with torch.no_grad():
        output = model(input)
    corrected_av.append(output.item())


fig, ax = plt.subplots()
ax.plot(time, actual_av, label='IMU AV')
ax.plot(time, joystick_av, label='Joystick AV')
ax.plot(time, corrected_av, label='Corrected Joystick AV')
ax.legend()
ax.set_xlabel('Time')
ax.set_ylabel('Joystick AV')
plt.show()




