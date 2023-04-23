import pandas as pd
import matplotlib.pyplot as plt

# read the CSV file
df = pd.read_csv('ikddata2.csv')

# extract the joystick values
joystick = df['executed'].apply(lambda x: x.strip('[]').split(', ')).tolist()

# extract the linear velocity and angular velocity values
linear_velocity = [float(j[0]) for j in joystick]
# angular_velocity = [float(j[1]) for j in joystick]

# generate a time array based on the number of records
time = range(len(joystick))

# plot the linear velocity and angular velocity against time
fig, ax = plt.subplots(2, 1, figsize=(8, 8))
ax[0].plot(time, linear_velocity)
ax[0].set_xlabel('Time')
ax[0].set_ylabel('IMU Angular Velocity')
ax[1].plot(time, linear_velocity)
ax[1].set_xlabel('Time')
ax[1].set_ylabel('Angular Velocity')
plt.show()