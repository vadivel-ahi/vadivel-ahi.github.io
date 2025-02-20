#!/usr/bin/env python3
import rospy
from gps_driver.msg import Customgps
from std_msgs.msg import Header
import utm
import time 
import serial


port = str(rospy.get_param('~port', '/dev/ttyUSB0'))
print(port)
def isGPGGAinString(inputString):
    if inputString.startswith("$GPGGA") : #replace 1 == 1 with condition to be checked for inputString
        print('Great Success!')
    else:
        print('GPGGA not found in the string')

def degMinstoDegDec(LatOrLong):
    deg = int(LatOrLong // 100)
    mins = LatOrLong % 100
    degDec = (mins/60)
    print(deg+degDec)
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir in ["S","W"]: 
        LatOrLong = -abs(LatOrLong)
        print(LatOrLong)
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0]
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    print(UTMVals)
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(UTC):
    current_date = time.gmtime()
    today_midnight = time.struct_time((current_date.tm_year, current_date.tm_mon, current_date.tm_mday, 0, 0, 0, 0, 0, 0))
    TimeSinceEpoch = int(time.mktime(today_midnight) - time.timezone)
    CurrentTime = TimeSinceEpoch + UTC
    CurrentTimeSec = int(CurrentTime)
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1e9)
    print(CurrentTime)
    return [CurrentTimeSec, CurrentTimeNsec]

def ReadFromSerial(serialPort):
    rospy.loginfo("Attempting to read from serial port")
    while not rospy.is_shutdown():
        try:
            serialPortt = serial.Serial(serialPort, baudrate=4800, timeout=1 )
            line = serialPortt.readline().decode('ascii', errors='ignore').strip()
            print("here")
            rospy.logdebug(f"Read line: {line}")
            serialPortt.close()
            if line.startswith('$GPGGA'):
                rospy.loginfo("Found GPGGA string")
                return line
        except serial.SerialException as e:
            rospy.logerr(f"Serial port error: {e}")
            return None
    return None
gpgga_string = ReadFromSerial(port)

def process_gps_data(gpgga_string):
    gpggaSplit = gpgga_string.split(",")
    UTC = float(gpggaSplit[1])
    Latitude = float(gpggaSplit[2])
    LatitudeDir = str(gpggaSplit[3])
    Longitude = float(gpggaSplit[4])
    LongitudeDir = str(gpggaSplit[5])
    HDOP = float(gpggaSplit[8])

    Latitude = degMinstoDegDec(Latitude)
    Longitude = degMinstoDegDec(Longitude)

    LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
    LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)

    UTM = convertToUTM(LatitudeSigned,LongitudeSigned)

    CurrentTime = UTCtoUTCEpoch(UTC)

    msg = Customgps()
    msg.header = Header()
    msg.header.stamp.secs = CurrentTime[0]
    msg.header.stamp.nsecs = CurrentTime[1]
    msg.header.frame_id = 'GPS1_Frame'
    msg.latitude = LatitudeSigned
    msg.longitude = LongitudeSigned
    msg.altitude = float(gpggaSplit[9])
    msg.utm_easting = UTM[0]
    msg.utm_northing = UTM[1]
    msg.zone = UTM[2]
    msg.letter = UTM[3]
    msg.hdop = HDOP
    msg.gpgga_read = gpgga_string

    rospy.loginfo(f"Processed GPS data: Lat {LatitudeSigned}, Lon {LongitudeSigned}")
    return msg


def main():
    rospy.init_node('gps_driver', anonymous=True)
    pub = rospy.Publisher('gps_data', Customgps, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    port = rospy.get_param('~port', '/dev/ttyUSB0')
    try:
        serialPort = serial.Serial(port, 9600, timeout=1)
        rospy.loginfo(f"Successfully opened serial port {port}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port {port}: {e}")
        return

    while not rospy.is_shutdown():
        gpgga_string = ReadFromSerial(port)
        if gpgga_string:
            rospy.loginfo(f"Received GPGGA string: {gpgga_string}")
            try:
                msg = process_gps_data(gpgga_string)
                pub.publish(msg)
                print("running")
                rospy.loginfo(f"Published GPS data: {msg}")
            except ValueError as e:
                rospy.logerr(f"Error processing GPS data: {e}")
        else:
            rospy.logwarn("No GPS data received")
        rate.sleep()

    serialPort.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

