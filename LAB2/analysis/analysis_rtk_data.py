import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def load_gps_data(bag_file):
    easting, northing, altitude, timestamps, fix_type, num_sat = [], [], [], [], [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gps_data']):
            easting.append(msg.utm_easting)
            northing.append(msg.utm_northing)
            altitude.append(msg.altitude)
            timestamps.append(t.to_sec())
            fix_type.append(msg.pos_stat)
            num_sat.append(msg.no_sat)
    return np.array(easting), np.array(northing), np.array(altitude), np.array(timestamps), np.array(fix_type), np.array(num_sat)

def preprocess_data(easting, northing, altitude):
    easting -= easting[0]
    northing -= northing[0]
    centroid_e = np.mean(easting)
    centroid_n = np.mean(northing)
    return easting, northing, altitude, centroid_e, centroid_n

# Load data
open_stationary_e, open_stationary_n, open_stationary_alt, open_stationary_t, open_stationary_fixtype, open_stationary_numsat = load_gps_data('/home/ahilesh/catkin_ws/src/data/LAB2_Stat_Open.bag')
occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt, occluded_stationary_t, occluded_stationary_fixtype, occluded_stationary_numsat = load_gps_data('/home/ahilesh/catkin_ws/src/data/LAB2_Stat_Occ.bag')
open_walk_e, open_walk_n, open_walk_alt, open_walk_t, open_walk_fixtype, open_walk_numsat = load_gps_data('/home/ahilesh/catkin_ws/src/data/LAB2_Walking_Open.bag')
occluded_walk_e, occluded_walk_n, occluded_walk_alt, occluded_walk_t, occluded_walk_fixtype, occluded_walk_numsat = load_gps_data('/home/ahilesh/catkin_ws/src/data/LAB2_Walking_Occ.bag')

# Preprocess data
open_stationary_e, open_stationary_n, open_stationary_alt, open_centroid_e, open_centroid_n = preprocess_data(open_stationary_e, open_stationary_n, open_stationary_alt)
occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt, occluded_centroid_e, occluded_centroid_n = preprocess_data(occluded_stationary_e, occluded_stationary_n, occluded_stationary_alt)
open_walk_e, open_walk_n, open_walk_alt, _, _ = preprocess_data(open_walk_e, open_walk_n, open_walk_alt)
occluded_walk_e, occluded_walk_n, occluded_walk_alt, _, _ = preprocess_data(occluded_walk_e, occluded_walk_n, occluded_walk_alt)

# Calculate deviations
open_easting_deviation = np.std(open_stationary_e)
open_northing_deviation = np.std(open_stationary_n)
occluded_easting_deviation = np.std(occluded_stationary_e)
occluded_northing_deviation = np.std(occluded_stationary_n)

# Stationary northing vs. easting scatterplots
plt.figure(figsize=(10, 8))
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

plt.text(0.65, 0.2, f'Open Easting Deviation: {open_easting_deviation:.2f} m', transform=plt.gca().transAxes, color='blue')
plt.text(0.65, 0.15, f'Open Northing Deviation: {open_northing_deviation:.2f} m', transform=plt.gca().transAxes, color='blue')
plt.text(0.65, 0.10, f'Occluded Easting Deviation: {occluded_easting_deviation:.2f} m', transform=plt.gca().transAxes, color='orange')
plt.text(0.65, 0.05, f'Occluded Northing Deviation: {occluded_northing_deviation:.2f} m', transform=plt.gca().transAxes, color='orange')

plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('Stationary Northing vs. Easting (Centroid Subtracted)')
plt.legend()
plt.text(0.05, 0.75, f'Open Centroid: ({open_centroid_e:.2f}, {open_centroid_n:.2f})', transform=plt.gca().transAxes)
plt.text(0.05, 0.70, f'Occluded Centroid: ({occluded_centroid_e:.2f}, {occluded_centroid_n:.2f})', transform=plt.gca().transAxes)
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

# open and occluded walk northing vs. easting scatterplot with line of best fit
plt.figure(figsize=(10, 8))
plt.scatter(open_walk_e, open_walk_n, label='open Walk', marker='o')
plt.scatter(occluded_walk_e, occluded_walk_n, label='occluded Walk', marker='x')

open_walk_slope, open_walk_intercept, _, _, _ = stats.linregress(open_walk_e, open_walk_n)
occluded_walk_slope, occluded_walk_intercept, _, _, _ = stats.linregress(occluded_walk_e, occluded_walk_n)

plt.plot(open_walk_e, open_walk_slope * open_walk_e + open_walk_intercept, color='r', label='open path Best Fit')
plt.plot(occluded_walk_e, occluded_walk_slope * occluded_walk_e + occluded_walk_intercept, color='b', label='occluded path Best Fit')

plt.xlabel('Easting (m)')
plt.ylabel('Northing (m)')
plt.title('open and occluded Walk: Northing vs. Easting')
plt.legend()
plt.grid(True)
plt.savefig('walk_scatter.png')

# open and occluded walk altitude vs. time plot
plt.figure(figsize=(10, 6))
plt.plot(open_walk_t - open_walk_t[0], open_walk_alt, label='open Walk')
plt.plot(occluded_walk_t - occluded_walk_t[0], occluded_walk_alt, label='occluded Walk')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('open and occluded Walk: Altitude vs. Time')
plt.legend()
plt.grid(True)
plt.savefig('walk_altitude.png')

#Number of satelites over time
plt.figure(figsize=(10,5))
plt.plot(open_walk_t - open_walk_t[0], open_walk_numsat, label='number of satlite while open walk')
plt.plot(occluded_walk_t - occluded_walk_t[0], occluded_walk_numsat, label='number of satlite while occluded walk')
plt.plot(open_stationary_t - open_stationary_t[0], open_stationary_numsat, label='number of satelite while open stationary')
plt.plot(occluded_stationary_t - occluded_stationary_t[0], occluded_stationary_numsat, label='number of satelite while open occluded')
plt.xlabel('Time (s)')
plt.ylabel('Number of satelites')
plt.title('Number of satelites for all the cases')
plt.legend()
plt.grid(True)
plt.savefig('num_sat.png')

#Fix type displayed in graph
def calculate_deviations(easting, northing, centroid_e, centroid_n):
    return np.sqrt((easting - centroid_e) ** 2 + (northing - centroid_n) ** 2)

def calculate_average_deviation_per_fix_type(deviations, fix_types):
    unique_fix_types = np.unique(fix_types)
    average_deviations = [np.mean(deviations[fix_types == fix_type]) for fix_type in unique_fix_types]
    return unique_fix_types, average_deviations

open_stationary_deviations = calculate_deviations(open_stationary_e, open_stationary_n, open_centroid_e, open_centroid_n)
occluded_stationary_deviations = calculate_deviations(occluded_stationary_e, occluded_stationary_n, occluded_centroid_e, occluded_centroid_n)

open_fix_types, open_avg_deviation = calculate_average_deviation_per_fix_type(open_stationary_deviations, open_stationary_fixtype)
occluded_fix_types, occluded_avg_deviation = calculate_average_deviation_per_fix_type(occluded_stationary_deviations, occluded_stationary_fixtype)

plt.figure(figsize=(12,6))
plt.scatter(open_stationary_fixtype, open_stationary_deviations, color='blue', label='Open', alpha=0.6)
plt.scatter(occluded_stationary_fixtype, occluded_stationary_deviations, color='orange', label='Occluded', alpha=0.6)
#categories = ['open_stat','occluded_stat','open_walking','occluded_walking']
#fix_values = [max(open_stationary_fixtype),max(occluded_stationary_fixtype),max(open_walk_fixtype),max(occluded_walk_fixtype)]
#plt.plot(open_stationary_t - open_stationary_t[0], open_stationary_fixtype, label='fix type at stationary open')
#plt.plot(occluded_stationary_t - occluded_stationary_t[0], occluded_stationary_fixtype, label='fix type at stationary occluded')
#plt.plot(open_walk_t - open_walk_t[0], open_walk_fixtype, label='fix type at walking open')
#plt.plot(occluded_walk_t - occluded_walk_t[0], occluded_walk_fixtype, label='fix type at walking occluded')
#plt.bar(categories,fix_values)
plt.xlabel('Fix Type')
plt.ylabel('Centroid deviation')
plt.title('Fix type for all cases of analysis')
plt.legend()
plt.grid(True)
plt.savefig('Centroid_deviation_vs_Fix_type.png')

plt.figure(figsize=(12,6))
plt.plot(open_fix_types, open_avg_deviation, 'o-', color='blue', label='Open', markersize=8)
plt.plot(occluded_fix_types, occluded_avg_deviation, 'x-', color='orange', label='Occluded', markersize=8)
plt.xlabel('Fix Type')
plt.ylabel('Average Centroid deviation')
plt.title('Fix type for all cases of analysis')
plt.legend()
plt.grid(True)
plt.savefig('Avg_Centroid_Deviation_vs_Fix_type.png')


#Analysing fix type distribution
def det_fix_type(fix):
    if (fix == 4).any():
        info = "RTK Fixed Solution (Highest Precision)"
        return info
    elif (fix == 5).any():
        info = "RTK Float Solution"
        return info
    elif (fix == 2).any():
        info = "Differential GPS Fixed Solution"
        return info
    elif (fix == 1).any():
        info = "Single Point Positioning (Least Precision)"
        return info
    else:
        info = "no fix"
        return info
    
#print("Fix Type of Open Stationary: ", open_stationary_fixtype, det_fix_type(open_stationary_fixtype))
#print("Fix Type of Occluded Stationary: ", occluded_stationary_fixtype, det_fix_type(occluded_stationary_fixtype))
#print("Fix Type of Open Walking: ", open_walk_fixtype, det_fix_type(open_walk_fixtype))
#print("Fix Type of Occluded Walking: ", occluded_walk_fixtype, det_fix_type(occluded_walk_fixtype))

print(f"Open Easting Deviation: {open_easting_deviation:.2f} m")
print(f"Open Northing Deviation: {open_northing_deviation:.2f} m")
print(f"Occluded Easting Deviation: {occluded_easting_deviation:.2f} m")
print(f"Occluded Northing Deviation: {occluded_northing_deviation:.2f} m")

plt.show()