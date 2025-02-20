import rospy
import serial
import time
import os
import rosbag
#from sensor_msgs.msg import Imu, MagneticField
from vn_driver.msg import Vectornav
from std_msgs.msg import Header
from math import pi, cos, sin

def euler_to_quaternion(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]

def clean_value(value):
    if '*' in value:
        return value.split('*')[0]
    return value

def create_set_rate_command(rate):
    command = b'\xFA\x0A\x03\x08\x00'
    rate_bytes = (800 // rate).to_bytes(4, byteorder='little')
    checksum = 0
    for byte in command[1:] + rate_bytes:
        checksum ^= byte
    return command + rate_bytes + bytes([checksum])

def main():
    # Initialize ROS node
    rospy.init_node('vectornav_driver')
    imu_pub = rospy.Publisher('/imu', Vectornav, queue_size=10)
    #mag_pub = rospy.Publisher('imu/mag', Vectornav, queue_size=10)

    bag_dir = os.path.expanduser('~/catkin_ws/src/imu/data/')
    if not os.path.exists(bag_dir):
        os.makedirs(bag_dir)
    bag_file = f'{bag_dir}IMU_data_10mins.bag'
    bag = rosbag.Bag(bag_file, 'w')

    # Open serial port
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust port as needed

    # Set output rate to 40 Hz
    set_rate_command = create_set_rate_command(40)
    ser.write(set_rate_command)
    
    # Wait for and read the response
    response = ser.read(8)
    if response[0] == 0xFA and response[1] == 0x0A and response[2] == 0x03:
        rospy.loginfo("Successfully set output rate to 40 Hz")
    else:
        rospy.logerr("Failed to set output rate")

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('ascii').strip()
            try:
                if line.startswith('$VNYMR'):
                    data = line.split(',')
                    
                    # Parse data
                    yaw, pitch, roll = map(float, map(clean_value, data[1:4]))
                    mag_x, mag_y, mag_z = map(float, map(clean_value, data[4:7]))
                    accel_x, accel_y, accel_z = map(float, map(clean_value, data[7:10]))
                    gyro_x, gyro_y, gyro_z = map(float, map(clean_value, data[10:13]))

                    # Create and publish IMU message
                    imu_msg = Vectornav()
                    imu_msg.header = Header()
                    cur_time = rospy.Time.now()
                    imu_msg.header.frame_id = "imu1_frame"
                    imu_msg.header.stamp.secs = cur_time.secs
                    imu_msg.header.stamp.nsecs = cur_time.nsecs
                    
                    # Convert Euler angles to quaternion
                    qx, qy, qz, qw = euler_to_quaternion(roll * pi/180, pitch * pi/180, yaw * pi/180)
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    
                    imu_msg.linear_acceleration.x = accel_x
                    imu_msg.linear_acceleration.y = accel_y
                    imu_msg.linear_acceleration.z = accel_z
                    
                    imu_msg.angular_velocity.x = gyro_x 
                    imu_msg.angular_velocity.y = gyro_y 
                    imu_msg.angular_velocity.z = gyro_z

                    imu_msg.magnetic_field.x = mag_x
                    imu_msg.magnetic_field.y = mag_y
                    imu_msg.magnetic_field.z = mag_z
                    
                    imu_msg.raw_imu_string = line

                    imu_pub.publish(imu_msg)
                    bag.write('/imu',imu_msg)

                    # Create and publish MagneticField message
                    '''mag_msg = Vectornav()
                    mag_msg.header = imu_msg.header
                    mag_msg.magnetic_field.x = mag_x
                    mag_msg.magnetic_field.y = mag_y
                    mag_msg.magnetic_field.z = mag_z

                    mag_pub.publish(mag_msg)
                    bag.write('/imu/mag',mag_msg)'''

                    # Print data to terminal
                    print(f"Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")
                    print(f"Magnetic field: {mag_x}, {mag_y}, {mag_z}")
                    print(f"Acceleration: {accel_x}, {accel_y}, {accel_z}")
                    print(f"Angular Rate: {gyro_x}, {gyro_y}, {gyro_z}")
                    print("---")

            except ValueError as e:
                rospy.logerr(f"Error parsing VNYMR data: {e}")
                continue

        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            break
        except KeyboardInterrupt:
            break

    bag.close()
    ser.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
