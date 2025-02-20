import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from scipy import signal
from scipy.signal import butter, lfilter
from scipy.integrate import cumtrapz
from scipy import linalg
import matplotlib.patches as patches

#Function to create low-pass filter
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cut_off, fs, order=5):
    b, a = butter_lowpass(cut_off, fs, order=order)
    y = lfilter(b, a, data)
    return y

#Function to create high-pass filter
def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Load the rosbag
bag = rosbag.Bag(r"/home/ahilesh/catkin_ws/src/LAB5/src/data/drive.bag")

# Initialize lists to store data
timestamps = []
gps_timestamps = []

gyro_x, gyro_y, gyro_z = [], [], []
accel_x, accel_y, accel_z = [], [], []
rot_x, rot_y, rot_z = [], [], []
mag_x, mag_y, mag_z = [], [], []
gps_x, gps_y, gps_z = [], [], []

# Extract data from the bag
for topic, msg, t in bag.read_messages(topics=['/imu', '/gps_data']):
    if topic == '/imu':
        timestamps.append(t.to_sec())
    
        # Extract magnetic field data
        mag_x.append(msg.mag_field.magnetic_field.x)
        mag_y.append(msg.mag_field.magnetic_field.y)
        mag_z.append(msg.mag_field.magnetic_field.z)

        # Gyro data (convert to degrees/s)
        gyro_x.append(np.degrees(msg.imu.angular_velocity.x))
        gyro_y.append(np.degrees(msg.imu.angular_velocity.y))
        gyro_z.append(np.degrees(msg.imu.angular_velocity.z))
        
        # Accelerometer data
        accel_x.append(msg.imu.linear_acceleration.x)
        accel_y.append(msg.imu.linear_acceleration.y)
        accel_z.append(msg.imu.linear_acceleration.z)
        
        # Convert quaternion to Euler angles (in degrees)
        q = [msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w]
        euler = euler_from_quaternion(q)
        rot_x.append(np.degrees(euler[0]))
        rot_y.append(np.degrees(euler[1]))
        rot_z.append(np.degrees(euler[2]))
    
    elif topic == '/gps_data':
        gps_timestamps.append(t.to_sec())
        gps_x.append(msg.utm_easting)
        gps_y.append(msg.utm_northing)
        gps_z.append(msg.altitude)
    
bag.close()

# Convert to numpy arrays
mag_x = np.array(mag_x)
mag_y = np.array(mag_y)
mag_z = np.array(mag_z)

accel_x = np.array(accel_x)
accel_y = np.array(accel_y)
accel_z = np.array(accel_z)

gps_x = np.array(gps_x)
gps_y = np.array(gps_y)


# Calculate offsets and scale factors
offset_x = (min(mag_x) + max(mag_x)) / 2
offset_y = (min(mag_y) + max(mag_y)) / 2

avg_delta = (max(mag_x) - min(mag_x) + max(mag_y) - min(mag_y)) / 4
scale_x = avg_delta / (max(mag_x) - min(mag_x))
scale_y = avg_delta / (max(mag_y) - min(mag_y))

mag_x_cal = (mag_x - offset_x) * scale_x
mag_y_cal = (mag_y - offset_y) * scale_y

# Convert to numpy arrays and normalize time
timestamps = np.array(timestamps) - timestamps[0]
gps_timestamps = np.array(gps_timestamps) - gps_timestamps[0]

#calculate yaw angle from raw magnetometer data
yaw_mag_raw = np.arctan2(mag_y, mag_x) 

#calculate yaw angle from calibrated magnetometer data
yaw_mag_corrected = np.arctan2(mag_y_cal, mag_x_cal)

#integrating yaw rate from gyro using cumulative trapezoidal integration
yaw_gyro_integrated = np.cumsum(np.radians(gyro_z)) * (timestamps[1] - timestamps[0])

#Wrap angles between -pi and pi
yaw_gyro_integrated_wrap = np.arctan2(np.sin(yaw_gyro_integrated), np.cos(yaw_gyro_integrated))

#sampling frequency
fs = 1 / timestamps[1] - timestamps[0] #Hz

#Low-pass filter for magnetometer data
cutoff_mag = 0.1 #cutoff freq for LPF
mag_filtered = butter_lowpass_filter(mag_x_cal + mag_y_cal * 1j, cutoff_mag, fs)

#High-pass filter for gyro data
cutoff_gyro = 0.1
gyro_filtered = butter_highpass_filter(yaw_gyro_integrated_wrap, cutoff_gyro, fs)

#Complementary filter parameters
alpha = 0.1 #tunning parameter

#initialize arrays for filtered results
yaw_filtered = np.zeros_like(mag_filtered)

#Complimentary filter
for i in range(1, len(timestamps)):
    yaw_filtered[i] = alpha * (yaw_filtered[i-1] + np.radians(gyro_filtered[i]) * (timestamps[i] - timestamps[i - 1])) + (1 - alpha) * mag_filtered[i]


# Calculate forward acceleration
forward_accel = accel_x
forward_accel_corrected = forward_accel - np.mean(forward_accel)
# Integrate acceleration to get velocity
velocity_from_accel_corrected = cumtrapz(forward_accel_corrected, timestamps, initial=0)
velocity_from_accel = cumtrapz(forward_accel, timestamps, initial=0)
# Adjust initial velocity and remove bias
initial_velocity = 0  # Adjust this based on your knowledge of the initial conditions
velocity_bias_corrected = np.mean(velocity_from_accel_corrected[:100])  # Assuming the first 100 samples represent stationary period
velocity_bias = np.mean(velocity_from_accel[:100])
velocity_from_accel_corrected = velocity_from_accel_corrected - velocity_bias_corrected + initial_velocity
velocity_from_accel = velocity_from_accel - velocity_bias + initial_velocity

# Calculate velocity from GPS
gps_velocity = np.sqrt(np.diff(gps_x)**2 + np.diff(gps_y)**2) / np.diff(gps_timestamps)
gps_velocity = np.insert(gps_velocity, 0, 0)  # Add initial velocity



# Plot N vs. E components before and after calibration
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
fig.suptitle("Figure 1: Magnetic X vs Magnetic Y Components", fontsize=14)

ax1.plot(mag_x, mag_y, 'b-', linewidth = 1, label = 'Before Calibration')
ax1.set_title("Before Calibration")
ax1.set_xlabel("Magnetometer x component")
ax1.set_ylabel("Magnetometer y component")
ax1.grid(True)
ax1.legend()
ax1.axis("equal")

ax2.plot(mag_x_cal, mag_y_cal, 'r-', linewidth = 1, label = 'After Calibration')
ax2.set_title("After Calibration")
ax2.set_xlabel("Magnetometer x component")
ax2.set_ylabel("Magnetometer y component")
ax2.grid(True)
ax2.legend()
ax2.axis("equal")

plt.tight_layout()
plt.savefig('Circle_calibration_plot.png')
plt.close()


#Yaw data plot from Magnetometer
plt.figure(figsize=(15, 10))
#plt.plot(timestamps, yaw_mag_raw, label='Raw Magnetometer Yaw', color='blue')
plt.plot(timestamps, yaw_mag_corrected, label='Calibrated Magnetometer Yaw', color='red')
plt.plot(timestamps, np.angle(mag_filtered), label='Low-pass filter of magnetometer data', color='blue')
plt.title('Yaw from Magnetometer')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('yaw_from_mag_with_LPF.png')
plt.close()


#Yaw data from integrating gyro data
plt.figure(figsize=(15,10))
#plt.plot(timestamps, yaw_gyro_integrated, label='Integrated Gyro Yaw', color='green')
plt.plot(timestamps, yaw_gyro_integrated_wrap, label='Integrated Gyro Yaw', color='blue')
plt.plot(timestamps, gyro_filtered, label='High-pass filter of gyro data', color='green')
plt.title('Yaw from Integrated Gyro data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('yaw_from_gyro_with_HPF.png')
plt.close()

#Low-pass plot of magnetometer data
plt.figure(figsize=(15, 10))
plt.plot(timestamps, np.angle(mag_filtered), label='Low-pass filter of magnetometer data', color='blue')
plt.title('Low-pass filter plot of magnetometer data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('LPF_mag_data.png')
plt.close()

#High-pass plot of gyro data
plt.figure(figsize=(15, 10))
plt.plot(timestamps, gyro_filtered, label='High-pass filter of gyro data', color='green')
plt.title('High-pass filter plot of gyro data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('HPF_gyro_data.png')
plt.close()

#Complimentary Filter of mag and gyro filtered data
plt.figure(figsize=(15, 10))
plt.plot(timestamps, yaw_filtered, label='Complimentary filter data', color='red')
#plt.plot(timestamps, yaw_mag_corrected, label='Calibrated Magnetometer Yaw', linestyle='dotted', linewidth=2, color='blue')
#plt.plot(timestamps, np.angle(mag_filtered), label='Low-pass filter of magnetometer data', color='blue')
#plt.plot(timestamps, yaw_gyro_integrated_wrap, label='Integrated Gyro Yaw', linestyle='dotted', linewidth=2, color='green')
#plt.plot(timestamps, gyro_filtered, label='High-pass filter of gyro data', color='green')
plt.title('Complimentary filter data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('Complimentary_filter_data.png')
plt.close()

imu_heading_estimate = np.radians(rot_z)
plt.plot(timestamps[:len(imu_heading_estimate)], imu_heading_estimate[:len(timestamps)], label='IMU Heading Estimate', color='orange')
plt.title('IMU Heading Estimate')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)
plt.savefig('IMU_Heading_estimate.png')
plt.close()



# Plotting results
plt.figure(figsize=(15, 10))

plt.subplot(4, 1, 1)
plt.plot(timestamps, np.angle(mag_filtered), label='Low-pass filter of magnetometer data', color='blue')
plt.title('Low-pass filter plot of magnetometer data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(timestamps, gyro_filtered, label='High-pass filter of gyro data', color='green')
plt.title('High-pass filter plot of gyro data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)

plt.subplot(4, 1, 3)
plt.plot(timestamps, yaw_filtered, label='Complimentary filter data', color='red')
plt.title('Complimentary filter data')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)

plt.subplot(4, 1, 4)
# Assuming you have IMU heading estimate from previous steps
imu_heading_estimate = np.radians(rot_z)  # Replace with actual IMU heading if available
plt.plot(timestamps[:len(imu_heading_estimate)], imu_heading_estimate[:len(timestamps)], label='IMU Heading Estimate', color='orange')
plt.title('IMU Heading Estimate')
plt.xlabel('Time (s)')
plt.ylabel('Yaw (radians)')
plt.grid(True)

plt.tight_layout()
plt.savefig('Filtered_Yaw_Estimates.png')
plt.close()

# Plot Forward Velocity results
plt.figure(figsize=(12, 8))
#plt.plot(timestamps, velocity_from_accel_corrected, label='Corrected Velocity from Accelerometer')
#plt.plot(timestamps, velocity_from_accel, label='Velocity from Accelerometer')
plt.plot(gps_timestamps, gps_velocity, label='Velocity from GPS')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Forward Velocity Estimation')
plt.legend()
plt.grid(True)
plt.savefig('Forward_Velocity_Estimation_from_GPS.png')
plt.close()

# Print some statistics
print(f"Mean velocity from accelerometer: {np.mean(velocity_from_accel * 3.6):.2f} km/h")
print(f"Mean velocity from GPS: {np.mean(gps_velocity * 3.6):.2f} km/h")
print(f"Max velocity from accelerometer: {np.max(velocity_from_accel * 3.6):.2f} km/h")
print(f"Max velocity from GPS: {np.max(gps_velocity * 3.6):.2f} km/h")



# Assuming velocity_from_accel is already calculated
displacement_imu = cumtrapz(velocity_from_accel_corrected, timestamps, initial=0)

#calculate GPS displacement
gps_displacement = np.sqrt(np.cumsum(np.diff(gps_x)**2 + np.diff(gps_y)**2))
gps_displacement = np.insert(gps_displacement, 0, 0)

plt.figure(figsize=(12, 6))
plt.plot(timestamps, displacement_imu, label='IMU Displacement')
plt.plot(gps_timestamps, gps_displacement, label='GPS Displacement')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.legend()
plt.title('IMU vs GPS Displacement')
plt.grid(True)
plt.savefig('IMU_vs_GPS_Displacement.png')
plt.close()


omega = gyro_z  # Assuming gyro_z is angular velocity around z-axis
omega_x_prime = omega * velocity_from_accel_corrected
y_double_prime_obs = np.diff(accel_y) / np.diff(timestamps)

plt.figure(figsize=(12, 6))
plt.plot(timestamps[1:], omega_x_prime[1:], label='ωx')
plt.plot(timestamps[1:], y_double_prime_obs, label='y_double_prime_obs')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.title('Comparison of ωx and y double prime obs')
plt.grid(True)
plt.savefig('omega_x_prime_vs_y_double_prime_obs.png')
plt.close()

# Assuming mag_heading is the heading from magnetometer or complementary filter
v_e = velocity_from_accel_corrected * np.sin(yaw_mag_corrected)
v_n = velocity_from_accel_corrected * np.cos(yaw_mag_corrected)

# Integrate to get trajectory
#x_e = cumtrapz(v_e, timestamps, initial=0)
#x_n = cumtrapz(v_n, timestamps, initial=0)

dt = np.mean(np.diff(timestamps))  # Time step (seconds)
x_e = np.cumsum(v_e)*dt
x_n = np.cumsum(v_n)*dt

x_e = np.array(x_e)
x_n = np.array(x_n)

# Adjust starting points
x_e -= x_e[0]
x_n -= x_n[0]
gps_x -= gps_x[0]
gps_y -= gps_y[0]


# Initial heading of GPS and IMU (calculated from the first two points)
gps_heading = np.arctan2(gps_x[3] - gps_x[0], gps_y[3] - gps_y[0])
imu_heading = np.arctan2(x_n[10] - x_n[0], x_e[10] - x_e[0])

# Compute the heading offset
heading_offset = gps_heading - imu_heading

# Apply the heading offset to the IMU trajectory
imu_easting_corrected = x_e * np.cos(heading_offset) - x_n * np.sin(heading_offset)
imu_northing_corrected = x_e * np.sin(heading_offset) + x_n * np.cos(heading_offset)

# Calculate total distances for both GPS and IMU
gps_distance = np.sqrt(np.maximum(0, np.diff(gps_x)**2 + np.diff(gps_y)**2)).sum()
imu_distance = np.sqrt(np.maximum(0, np.diff(imu_easting_corrected)**2 + np.diff(imu_northing_corrected)**2)).sum()

# Find the scaling factor
scaling_factor = gps_distance / imu_distance
print(f"Scaling Factor for East: {scaling_factor}")

# Apply the scaling factor
imu_easting_corrected *= scaling_factor
imu_northing_corrected *= scaling_factor


# Simple 2D trajectory plot GPS
plt.figure(figsize=(15, 9))
plt.plot(gps_x, gps_y, label='GPS Trajectory', color='blue')
plt.scatter(gps_x[0], gps_y[0], color='green', label='Start Point', s=100)  # Mark start point
plt.scatter(gps_x[-1], gps_y[-1], color='red', label='End Point', s=100)    # Mark end point
plt.xlabel('Easting (meters)')
plt.ylabel('Northing (meters)')
plt.title('Trajectory from GPS Data')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.savefig('Trajectory_from_gps_data.png')
plt.close()

# Simple 2D trajectory plot IMU
plt.figure(figsize=(15, 9))
plt.plot(imu_easting_corrected, imu_northing_corrected, label='IMU Trajectory', color='blue')
plt.xlabel('Easting (meters)')
plt.ylabel('Northing (meters)')
plt.title('Trajectory from IMU Data')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.savefig('Trajectory_from_IMU_data.png')
plt.close()

# Plot trajectory
plt.figure(figsize=(15, 9))
plt.plot(gps_x, gps_y, label="GPS Trajectory", color='blue')
plt.plot(imu_easting_corrected, imu_northing_corrected, label="IMU Trajectory", color='green')
plt.xlabel("Easting (meters)")
plt.ylabel("Northing (meters)")
plt.title("IMU vs GPS Trajectory")
plt.axis("equal")
plt.legend()
plt.grid(True)
plt.savefig('Trajectory_from_IMU&GPS_data.png')
plt.close()


# Adjust heading
#initial_heading = np.arctan2(gps_x[1] - gps_x[0], gps_y[1] - gps_y[0])
#rotation_matrix = np.array([[np.cos(initial_heading), -np.sin(initial_heading)],[np.sin(initial_heading), np.cos(initial_heading)]])
#x_e, x_n = np.dot(rotation_matrix, [x_e, x_n])

# Plot trajectories
plt.figure(figsize=(12, 12))
plt.plot(x_e, x_n, label='IMU Trajectory')
plt.plot(gps_x, gps_y, label='GPS Trajectory')
plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.legend()
plt.title('IMU vs GPS Trajectory')
plt.axis('equal')
plt.grid(True)
plt.savefig('IMU_vs_GPS_Trajectory.png')
plt.close()