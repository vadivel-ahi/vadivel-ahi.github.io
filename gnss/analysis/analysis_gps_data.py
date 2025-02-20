import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def load_gps_data(bag_file):
    easting, northing, altitude, timestamps = [], [], [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps_data']):
            easting.append(msg.utm_easting)
            northing.append(msg.utm_northing)
            altitude.append(msg.altitude)
            timestamps.append(t.to_sec())
    return np.array(easting), np.array(northing), np.array(altitude), np.array(timestamps)

def preprocess_data(easting, northing, altitude):
    easting -= easting[0]
    northing -= northing[0]
    centroid_e = np.mean(easting)
    centroid_n = np.mean(northing)
    return easting, northing, altitude, centroid_e, centroid_n

# Load data
open_stationary_e, open_stationary_n, open_stationary_alt, open_stationary_t = load_gps_data('ocopen_gps_data.bag')
occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt, occluded_stationary_t = load_gps_data('occluded_gps_data.bag')
walk_e, walk_n, walk_alt, walk_t = load_gps_data('walking_gps_data.bag')

# Preprocess data
open_stationary_e, open_stationary_n, open_stationary_alt, open_centroid_e, open_centroid_n = preprocess_data(open_stationary_e, open_stationary_n, open_stationary_alt)
occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt, occluded_centroid_e, occluded_centroid_n = preprocess_data(occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt)
walk_e, walk_n, walk_alt, _, _ = preprocess_data(walk_e, walk_n, walk_alt)

# Stationary northing vs. easting scatterplots
plt.figure(figsize=(10, 8))
#plt.scatter(open_stationary_e - open_centroid_e, open_stationary_n - open_centroid_n, label='Open', marker='o')
#plt.scatter(occluded_stationary_e - occluded_centroid_e, occluded_stationary_n - occluded_centroid_n, label='Occluded', marker='x')

easting_open_offset = open_stationary_e - open_stationary_e[0]
northing_open_offset = open_stationary_n - open_stationary_n[0]
centroid_easting_open = np.mean(easting_open_offset)
centroid_northing_open = np.mean(northing_open_offset)

easting_occluded_offset = occluded_stationary_e - occluded_stationary_e[0]
northing_occluded_offset = occluded_stationary_n - occluded_stationary_n[0]
centroid_easting_occluded = np.mean(easting_occluded_offset)
centroid_northing_occluded = np.mean(northing_occluded_offset)

plt.scatter(easting_open_offset, northing_open_offset, label='Open', marker='o', color='blue')
plt.scatter(easting_occluded_offset, northing_occluded_offset, label='Occluded', marker='x', color='orange')

plt.scatter(centroid_easting_open, centroid_northing_open, color='red', label='Centroid (Open)', marker='+')
plt.scatter(centroid_easting_occluded, centroid_northing_occluded, color='purple', label='Centroid (Occluded)', marker='*')

plt.text(centroid_easting_open, centroid_northing_open, f'  Centroid (Open): ({centroid_easting_open:.2f}, {centroid_northing_open:.2f})', 
         verticalalignment='bottom', horizontalalignment='right', color='red')
plt.text(centroid_easting_occluded, centroid_northing_occluded, f'  Centroid (Occluded): ({centroid_easting_occluded:.2f}, {centroid_northing_occluded:.2f})', 
         verticalalignment='bottom', horizontalalignment='right', color='purple')


plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('Stationary Northing vs. Easting (Centroid Subtracted)')
plt.legend()
plt.text(0.05, 0.95, f'Open Centroid: ({open_centroid_e:.2f}, {open_centroid_n:.2f})', transform=plt.gca().transAxes)
plt.text(0.05, 0.90, f'Occluded Centroid: ({occluded_centroid_e:.2f}, {occluded_centroid_n:.2f})', transform=plt.gca().transAxes)
plt.grid(True)
plt.savefig('stationary_scatter.png')

# Stationary altitude vs. time plot
plt.figure(figsize=(10, 6))
plt.plot(open_stationary_t - open_stationary_t[0], open_stationary_alt, label='Open')
plt.plot(occluded_stationary_t - occluded_stationary_t[0], occluded_stationary_alt, label='Occluded')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Stationary Altitude vs. Time')
plt.legend()
plt.grid(True)
plt.savefig('stationary_altitude.png')

# Stationary histogram plots
def plot_histogram(data, title, filename):
    plt.figure(figsize=(8, 6))
    plt.hist(data, bins=30, edgecolor='black')
    plt.xlabel('Distance from Centroid (m)')
    plt.ylabel('Frequency')
    plt.title(title)
    plt.grid(True)
    plt.savefig(filename)

open_distances = np.sqrt((open_stationary_e - open_centroid_e)**2 + (open_stationary_n - open_centroid_n)**2)
occluded_distances = np.sqrt((occluded_stationary_e - occluded_centroid_e)**2 + (occluded_stationary_n - occluded_centroid_n)**2)

plot_histogram(open_distances, 'Open Stationary Position Histogram', 'open_histogram.png')
plot_histogram(occluded_distances, 'Occluded Stationary Position Histogram', 'occluded_histogram.png')

# 200-meter walk northing vs. easting scatterplot with line of best fit
plt.figure(figsize=(10, 8))
plt.scatter(walk_e, walk_n, label='200m Walk', marker='o')

walk_slope, walk_intercept, _, _, _ = stats.linregress(walk_e, walk_n)

plt.plot(walk_e, walk_slope * walk_e + walk_intercept, color='r', label='Best Fit')

plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('200-meter Walk: Northing vs. Easting')
plt.legend()
plt.grid(True)
plt.savefig('walk_scatter.png')

# 200-meter walk altitude vs. time plot
plt.figure(figsize=(10, 6))
plt.plot(walk_t - walk_t[0], walk_alt, label='200m Walk')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('200-meter Walk: Altitude vs. Time')
plt.legend()
plt.grid(True)
plt.savefig('walk_altitude.png')

# Calculate deviations
open_easting_deviation = np.std(open_stationary_e)
open_northing_deviation = np.std(open_stationary_n)
occluded_easting_deviation = np.std(occluded_stationary_e)
occluded_northing_deviation = np.std(occluded_stationary_n)

print(f"Open Easting Deviation: {open_easting_deviation:.2f} m")
print(f"Open Northing Deviation: {open_northing_deviation:.2f} m")
print(f"Occluded Easting Deviation: {occluded_easting_deviation:.2f} m")
print(f"Occluded Northing Deviation: {occluded_northing_deviation:.2f} m")

plt.show()

