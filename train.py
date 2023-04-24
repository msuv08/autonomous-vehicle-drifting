import numpy as np
import torch
from scipy.io import loadmat
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
import sys
import os
from ikd_model import IKDModel


if __name__ == '__main__':

    data_name = './dataset/ikddata2.csv'
    data = pd.read_csv(data_name)
    joystick = np.array([eval(i) for i in data["joystick"]])
    executed = np.array([eval(i) for i in data["executed"]])
    data = np.concatenate((joystick, executed), axis = 1)
    model = IKDModel(2, 1)
    epochs = 50
    optimizer = torch.optim.Adam(model.parameters(), lr = 1e-5, weight_decay = 1e-3)
    for epoch in range(epochs):

        model.train()
        ep_loss = 0
        for i, row in enumerate(data):
            v = row[0]
            av = row[1]
            true_av = row[2]
            input = torch.FloatTensor([v, true_av])
            label = torch.FloatTensor([av])
            model.zero_grad()
            output = model(input)
            weights = torch.abs(label)
            weighted_loss = (F.mse_loss(output, label, reduction='none') * weights).mean()
            weighted_loss.backward()
            optimizer.step()
            ep_loss += weighted_loss.item()
        print("[INFO] epoch {} | loss {:10.4f}".format(epoch, ep_loss / (len(data))))
        torch.save(model.state_dict(), "ikddata2.pt")
    print("[INFO] Training complete. Model saved as 'ikddata2.pt")
            