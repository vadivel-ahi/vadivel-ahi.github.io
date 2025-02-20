import rosbag
import numpy as np
#import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Load the rosbag
bag = rosbag.Bag(r"/home/ahilesh/catkin_ws/src/imu/data/IMU_data_10mins.bag")

# Initialize lists to store data
timestamps = []
gyro_x, gyro_y, gyro_z = [], [], []
accel_x, accel_y, accel_z = [], [], []
rot_x, rot_y, rot_z = [], [], []

# Extract data from the bag
for topic, msg, t in bag.read_messages(topics=['/imu']):
    timestamps.append(t.to_sec())

    # Gyro data (convert to degrees/s)
    gyro_x.append(np.degrees(msg.angular_velocity.x))
    gyro_y.append(np.degrees(msg.angular_velocity.y))
    gyro_z.append(np.degrees(msg.angular_velocity.z))
    
    # Accelerometer data
    accel_x.append(msg.linear_acceleration.x)
    accel_y.append(msg.linear_acceleration.y)
    accel_z.append(msg.linear_acceleration.z)
    
    # Convert quaternion to Euler angles (in degrees)
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    euler = euler_from_quaternion(q)
    rot_x.append(np.degrees(euler[0]))
    rot_y.append(np.degrees(euler[1]))
    rot_z.append(np.degrees(euler[2]))

bag.close()

# Convert to numpy arrays and normalize time
timestamps = np.array(timestamps) - timestamps[0]

# Figure 0: Gyro data
fig, (ax1,ax2,ax3) = plt.subplots(3, 1, figsize=(15,12))
fig.suptitle("Figure 0: Gyroscope Data", fontsize=16)

ax1.plot(timestamps, gyro_x, 'r-', label='X')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('X Direction Rotational Rate (deg/s)')
ax1.grid()

ax2.plot(timestamps, gyro_y, 'g-', label='Y')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Y Direction Rotational Rate (deg/s)')
ax2.grid()

ax3.plot(timestamps, gyro_z, 'b-', label='Z')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Z Direction Rotational Rate (deg/s)')
ax3.grid()

plt.legend()
plt.tight_layout()
plt.savefig('gyro_data.png')
plt.close()

# Figure 1: Accelerometer data
fig, (ax1,ax2,ax3) = plt.subplots(3, 1, figsize=(15,12))
fig.suptitle("Figure 1: Accelerometer Data", fontsize=16)

ax1.plot(timestamps, accel_x, 'r-', label='X')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Acceleration along x dir (m/s²)')
ax1.grid()

ax2.plot(timestamps, accel_y, 'g-', label='Y')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Acceleration along y dir (m/s²)')
ax2.grid()

ax3.plot(timestamps, accel_z, 'b-', label='Z')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Acceleration along z dir (m/s²)')
ax3.grid()

plt.legend()
plt.tight_layout()
plt.savefig('accel_data.png')
plt.close()

# Figure 2: Rotation data
fig, (ax1,ax2,ax3) = plt.subplots(3, 1, figsize=(15,12))
fig.suptitle("Figure 2: Rotation Data", fontsize=16)

ax1.plot(timestamps, rot_x, 'r-', label='X')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Rotation along x dir (deg)')
ax1.grid()

ax2.plot(timestamps, rot_y, 'g-', label='Y')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Rotation along y dir (deg)')
ax2.grid()

ax3.plot(timestamps, rot_z, 'b-', label='Z')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Rotation along z dir (deg)')
ax3.grid()

plt.legend()
plt.tight_layout()
plt.savefig('rotation_data.png')
plt.close()

# Figure 3: Histograms of rotation
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))
#fig.suptitle("Figure 3: Rotation Data Histogram", fontsize=16)
ax1.hist(rot_x, bins=50, color='r', alpha=0.7)
ax1.set_title('X Rotation')
ax1.set_xlabel('Degrees')
ax2.hist(rot_y, bins=50, color='g', alpha=0.7)
ax2.set_title('Y Rotation')
ax2.set_xlabel('Degrees')
ax3.hist(rot_z, bins=50, color='b', alpha=0.7)
ax3.set_title('Z Rotation')
ax3.set_xlabel('Degrees')
plt.tight_layout()
plt.savefig('rotation_histograms.png')
plt.close()
