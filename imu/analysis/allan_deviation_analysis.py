import rosbag
import numpy as np
import matplotlib.pyplot as plt
from allantools import oadev

fs = 20
ts = 1/fs

# Read the bag file
bag = rosbag.Bag(r"/home/ahilesh/catkin_ws/src/imu/data/LocationB.bag")

# Lists to store gyro data
gyro_x, gyro_y, gyro_z = [], [], []
accel_x, accel_y, accel_z = [], [], []
timestamps = []

for topic, msg, t in bag.read_messages(topics=['/vectornav']):
    try:
        # Parse the VNYMR string
        data = msg.data.split(',')
        if data[0] == '$VNYMR':
            gyro_x.append(float(data[10]))
            gyro_y.append(float(data[11]))
            gyro_z.append(float(data[12].split('*')[0]))  # Remove checksum

            accel_x.append(float(data[7]))
            accel_y.append(float(data[8]))
            accel_z.append(float(data[9]))

            timestamps.append(t.to_sec())
    except (ValueError, IndexError):
        # Handle bad strings by skipping them
        continue

bag.close()

# Convert lists to numpy arrays
gyro_x, gyro_y, gyro_z = map(np.array, [gyro_x, gyro_y, gyro_z])
accel_x, accel_y, accel_z = map(np.array, [accel_x, accel_y, accel_z])
timestamps = np.array(timestamps)


# Calculate time differences
dt = np.diff(timestamps).mean()

# Calculate Allan Variance for each axis (gyro and accelerometer)
t, adev_gyro_x, _, _ = oadev(gyro_x, rate=20, data_type="phase", taus="all")
t, adev_gyro_y, _, _ = oadev(gyro_y, rate=20, data_type="phase", taus="all")
t, adev_gyro_z, _, _ = oadev(gyro_z, rate=20, data_type="phase", taus="all")
t, adev_accel_x, _, _ = oadev(accel_x, rate=20, data_type="phase", taus="all")
t, adev_accel_y, _, _ = oadev(accel_y, rate=20, data_type="phase", taus="all")
t, adev_accel_z, _, _ = oadev(accel_z, rate=20, data_type="phase", taus="all")


def extract_parameters(t, adev):
    # Angle Random Walk (N) at tau = 1s
    N = adev[np.argmin(np.abs(t - 1))]
    
    # Bias Instability (B) at the minimum point
    B = np.min(adev)
    
    # Rate Random Walk (K) from the slope after the minimum
    min_idx = np.argmin(adev)
    if min_idx < len(adev) - 1:
        slope = np.diff(np.log(adev[min_idx:])) / np.diff(np.log(t[min_idx:]))
        if len(slope) > 0:
            K = np.sqrt(slope[0] / 2)
        else:
            K = np.nan
    else:
        K = np.nan
    
    return N, B, K

N_x, B_x, K_x = extract_parameters(t, adev_gyro_x)
N_y, B_y, K_y = extract_parameters(t, adev_gyro_y)
N_z, B_z, K_z = extract_parameters(t, adev_gyro_z)

print(f"Gyro X - N: {N_x}, B: {B_x}, K: {K_x}")
print(f"Gyro Y - N: {N_y}, B: {B_y}, K: {K_y}")
print(f"Gyro Z - N: {N_z}, B: {B_z}, K: {K_z}")

N_accel_x, B_accel_x, K_accel_x = extract_parameters(t, adev_accel_x)
N_accel_y, B_accel_y, K_accel_y = extract_parameters(t, adev_accel_y)
N_accel_z, B_accel_z, K_accel_z = extract_parameters(t, adev_accel_z)

print(f"Accel X - N: {N_accel_x}, B: {B_accel_x}, K: {K_accel_x}")
print(f"Accel Y - N: {N_accel_y}, B: {B_accel_y}, K: {K_accel_y}")
print(f"Accel Z - N: {N_accel_z}, B: {B_accel_z}, K: {K_accel_z}")


import matplotlib.pyplot as plt

# Plot gyro data
plt.figure(figsize=(12, 6))
plt.loglog(t, adev_gyro_x, label='Gyro X')
plt.loglog(t, adev_gyro_y, label='Gyro Y')
plt.loglog(t, adev_gyro_z, label='Gyro Z')
plt.xlabel('Tau (s)')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Allan Deviation Gyroscope Data')
plt.legend()
plt.grid(True)

# Add text with B, N, K values
text = f"Gyro X - N: {N_x:.2e}, B: {B_x:.2e}, K: {K_x:.2e}\n"
text += f"Gyro Y - N: {N_y:.2e}, B: {B_y:.2e}, K: {K_y:.2e}\n"
text += f"Gyro Z - N: {N_z:.2e}, B: {B_z:.2e}, K: {K_z:.2e}"
plt.text(0.05, 0.05, text, transform=plt.gca().transAxes, fontsize=8, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.7))

plt.savefig('allan_dev_gyro.png')
plt.show()

# Plot accelerometer data
plt.figure(figsize=(12, 6))
plt.loglog(t, adev_accel_x, label='Accel X')
plt.loglog(t, adev_accel_y, label='Accel Y')
plt.loglog(t, adev_accel_z, label='Accel Z')
plt.xlabel('Tau (s)')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Allan Deviation Accelerometer Data')
plt.legend()
plt.grid(True)

# Add text with B, N, K values
text = f"Accel X - N: {N_accel_x:.2e}, B: {B_accel_x:.2e}, K: {K_accel_x:.2e}\n"
text += f"Accel Y - N: {N_accel_y:.2e}, B: {B_accel_y:.2e}, K: {K_accel_y:.2e}\n"
text += f"Accel Z - N: {N_accel_z:.2e}, B: {B_accel_z:.2e}, K: {K_accel_z:.2e}"
plt.text(0.05, 0.05, text, transform=plt.gca().transAxes, fontsize=8, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.7))

plt.savefig('allan_dev_accel.png')
plt.show()


def integrate_gyro(gyro_data, timestamps):
    dt = np.diff(timestamps)
    integrated = np.cumsum(gyro_data[1:] * dt)
    return np.concatenate(([0], integrated))  # Add initial value of 0

def find_bias_stability_point(t, adev):
    min_idx = np.argmin(adev)
    return t[min_idx], adev[min_idx]

# Integrate gyro data
integrated_x = integrate_gyro(gyro_x, timestamps)
integrated_y = integrate_gyro(gyro_y, timestamps)
integrated_z = integrate_gyro(gyro_z, timestamps)

# Integrate accelerometer data (velocity)
velocity_x = integrate_gyro(accel_x, timestamps)
velocity_y = integrate_gyro(accel_y, timestamps)
velocity_z = integrate_gyro(accel_z, timestamps)

# Adjust timestamps for plotting (remove the first point to match integrated data)
plot_timestamps = timestamps[1:]

int_t, int_adev_gyro_x, _, _ = oadev(integrated_x, rate=20, data_type="phase", taus="all")
int_t, int_adev_gyro_y, _, _ = oadev(integrated_y, rate=20, data_type="phase", taus="all")
int_t, int_adev_gyro_z, _, _ = oadev(integrated_z, rate=20, data_type="phase", taus="all")
int_t, int_adev_accel_x, _, _ = oadev(velocity_x, rate=20, data_type="phase", taus="all")
int_t, int_adev_accel_y, _, _ = oadev(velocity_y, rate=20, data_type="phase", taus="all")
int_t, int_adev_accel_z, _, _ = oadev(velocity_z, rate=20, data_type="phase", taus="all")

# Plot integrated gyro data
plt.figure(figsize=(12, 8))
plt.loglog(int_t, int_adev_gyro_x, label='Integrated Gyro X')
plt.loglog(int_t, int_adev_gyro_y, label='Integrated Gyro Y')
plt.loglog(int_t, int_adev_gyro_z, label='Integrated Gyro Z')

# Find and plot bias stability points
tau_x, B_x = find_bias_stability_point(int_t, int_adev_gyro_x)
tau_y, B_y = find_bias_stability_point(int_t, int_adev_gyro_y)
tau_z, B_z = find_bias_stability_point(int_t, int_adev_gyro_z)

plt.scatter(tau_x, B_x, color='red', s=100, marker='o', label='Bias Stability X')
plt.scatter(tau_y, B_y, color='green', s=100, marker='o', label='Bias Stability Y')
plt.scatter(tau_z, B_z, color='blue', s=100, marker='o', label='Bias Stability Z')

plt.xlabel('Time (s)')
plt.ylabel('Angular Position (rad)')
plt.title('Integrated Gyro Data')
plt.legend()
plt.grid(True)

# Add text with B, N, K values
text = f"Gyro X - N: {N_x:.2e}, B: {B_x:.2e}, K: {K_x:.2e}\n"
text += f"Gyro Y - N: {N_y:.2e}, B: {B_y:.2e}, K: {K_y:.2e}\n"
text += f"Gyro Z - N: {N_z:.2e}, B: {B_z:.2e}, K: {K_z:.2e}"
plt.text(0.05, 0.05, text, transform=plt.gca().transAxes, fontsize=8, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.7))

plt.savefig('allan_dev_inte_gyro.png')
plt.show()

# Plot integrated accelerometer data (velocity)
plt.figure(figsize=(12, 8))
plt.loglog(int_t, int_adev_accel_x, label='Velocity X')
plt.loglog(int_t, int_adev_accel_y, label='Velocity Y')
plt.loglog(int_t, int_adev_accel_z, label='Velocity Z')

# Find and plot bias stability points
tau_accel_x, B_accel_x = find_bias_stability_point(int_t, int_adev_accel_x)
tau_accel_y, B_accel_y = find_bias_stability_point(int_t, int_adev_accel_y)
tau_accel_z, B_accel_z = find_bias_stability_point(int_t, int_adev_accel_z)

plt.scatter(tau_accel_x, B_accel_x, color='red', s=100, marker='o', label='Bias Stability X')
plt.scatter(tau_accel_y, B_accel_y, color='green', s=100, marker='o', label='Bias Stability Y')
plt.scatter(tau_accel_z, B_accel_z, color='blue', s=100, marker='o', label='Bias Stability Z')

plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Integrated Accelerometer Data (Velocity)')
plt.legend()
plt.grid(True)

# Add text with B, N, K values
text = f"Accel X - N: {N_accel_x:.2e}, B: {B_accel_x:.2e}, K: {K_accel_x:.2e}\n"
text += f"Accel Y - N: {N_accel_y:.2e}, B: {B_accel_y:.2e}, K: {K_accel_y:.2e}\n"
text += f"Accel Z - N: {N_accel_z:.2e}, B: {B_accel_z:.2e}, K: {K_accel_z:.2e}"
plt.text(0.05, 0.05, text, transform=plt.gca().transAxes, fontsize=8, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.7))

plt.savefig('allan_dev_inte_accel.png')
plt.show()