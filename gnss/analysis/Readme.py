#This text file provides the details about where the data has been collected and basic explaination about the graphs.
#----------
#Info about data collection locations:
#
#Open space data has been recorded on the middle of the  bridge to EXP and ISEC.
#Occluded space data has been recorded at the space infront of snell library under some trees.
#Walking data has been recorded at the Columbus Ave walking for about 200 meters.
#
#---------- 
#Info about the procedure and the plots:
#
#Recording of the rosbag files for the 3 cases using the GPS module was done and stored in the data folder in the gps_drivers package.
#Ran the analysis script to plot the obtained data.
#
#1. 'Stationary Northing Vs. Easting (Centroid Subtracted)' plot shows the northing and easting gps data point for both open and occluded areas after centroid subtraction along with the centroid of the data set collected.
#2. 'Open Stationary Position Histogram' plot shows the frequency of the deviation from the centroid for open space.
#3. 'Occluded Stationary Position Histogram' plot shows the frequency of the deviation from centroid for occluded space.
#4. 'Stationary Altitude vs. Time' plot shows the altitude data about the open and occluded data, here we can observe that the open space data has higher altitude as we took the data on the bridge which is comparitively higher than the location we took the occluded space data. 
#5. '200-meter Walk: Northing vs. Easting' plot shows the gps data point of the path taken in a straight line and the best fit for the path has been plotted along with it.
#6. '200-meter Walk: Altitude vs. Time' plot shows the altitude data recorded while walking.
#
#----------