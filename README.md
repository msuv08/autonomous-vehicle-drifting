# Learning Inverse Kinodynamics for Autonomous Vehicle Drifting

## Introduction

In this work, we seek to learn an inverse kinodynamic function that can be used to predict corrected (tightened) angular velocity control inputs from the slip of a drift. We leverage on-board intertial readings from a vectornav IMU and a simple compact neural network as our learned function approximator for the inverse kinodynamic model at test-time. 

## Model Architecture 

<img width="801" alt="Screenshot 2023-04-23 at 11 36 41 PM" src="https://user-images.githubusercontent.com/61725820/233925087-17a5153a-759e-42a5-8d68-762efa5245d2.png">

In the original IKD paper, they feed the IMU readings into a 600-dimensional vector, pass it through 256 neurons as a sequential auto-encoder and then feed it into a simple neural network representing the learned function approximator. In the case of our training data, we only leverage the one reading from the IMU data, making the need of a sequential auto-encoder unnecessary. Below, we outline a description of our model, and show case a graphic of our architecture:

Since our extracted IMU reading is 1 dimensional, we feed this value directly into the function approximator alongside the joystick velocity value, making the input to our inverse kinodynamic model a 2 dimensional tensor. The model itself is a quite simple and compact neural network implemented in PyTorch. It consists of 3 fully connected Linear layers, with the first 2 layers having 32 hidden neurons. The correction layer outputs a 1 dimensional prediction of the joystick angular velocity. In our forward pass, we leverage the ReLU activation function, and avoid a sigmoid or tanh activation on the correction layer. This is because we do not normalize our values during training, and want to avoid our predicted angular velocity being bounded within 0 and 1 or -1 and 1. Since our formulation and data is quite simple (purely float values), we did not need to really worry about normalizing to see an improvement in training loss. Generally, it is important to note that angular velocity can range from -4 to 4, in the most extreme cases. 


## Hardware

<img width="268" alt="Screenshot 2023-04-24 at 2 16 05 AM" src="https://user-images.githubusercontent.com/61725820/233925246-1dc71d10-8617-4fdc-a32a-24faeaa9e41b.png">

We leverage the UT AUTOmata outfitted with a LiDAR sensor, an NVIDIA Jetson TX2, and a Vectornav VN-100 IMU. These vehicles use an Ackerman steering system, are four-wheel drive, and have a TRAXXAS Titan 550 motor controlled by a Flipsky VESC 4.12 50A motor controller. Onboard each F1/10 vehicle is copy of ROS (Robot Operating System), which is used to control and launch nodes for operating all core functionalities within the car

## Presentation Access Link: https://docs.google.com/presentation/d/1UIhE7_GrrWaCUh5Ybc6QZLqAd55wXRPtd-C7zXoDhDI/edit?usp=sharing

