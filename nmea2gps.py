import argparse
import utm
from sensor_msgs.msg import NavSatFix, NavSatStatus


def get_gps_data_fromGGA(line):
    timestamp = line.split(' ')[0] + "000"  # nanoseconds precision
    sentencesData = line.split(',')

    #get status and service
    gps_qual = int(sentencesData[6])
    if gps_qual == 0:
        status = NavSatStatus.STATUS_NO_FIX
    elif gps_qual == 1:
        status = NavSatStatus.STATUS_FIX
    elif gps_qual == 2:
        status = NavSatStatus.STATUS_SBAS_FIX
    elif gps_qual in (4, 5):
        status = NavSatStatus.STATUS_GBAS_FIX
    elif gps_qual == 9:
        # Support specifically for NOVATEL OEM4 recievers which report WAAS fix as 9
        # http://www.novatel.com/support/known-solutions/which-novatel-position-types-correspond-to-the-gga-quality-indicator/
        status = NavSatStatus.STATUS_SBAS_FIX
    else:
        status = NavSatStatus.STATUS_NO_FIX
    service = NavSatStatus.SERVICE_GPS

    # get latitude in degrees
    latitudeRaw = float(sentencesData[2])
    latitudeRawDegrees = latitudeRaw // 100 # int type division
    latitudeRawMinutes = latitudeRaw % 100
    # Get the sign of the latitude. It depends if latitude is North or South
    latitudeSign = 1
    latitudeCartidnalDirection = sentencesData[3]
    if latitudeCartidnalDirection == 'S':
        latitudeSign = -1

    latitude = latitudeSign * (latitudeRawDegrees + latitudeRawMinutes / 60.0)

    # get longitude in degrees
    longitudeRaw = float( sentencesData[4] )
    longitudeRawDegrees = longitudeRaw // 100 # division entera
    longitudeRawMinutes = longitudeRaw % 100

    # Get the sign of the longitude. It depends if longitude is West or East
    longitudeSign = 1
    longitudeCartidnalDirection = sentencesData[5]

    if longitudeCartidnalDirection == 'W':
        longitudeSign = -1

    longitude = longitudeSign * (longitudeRawDegrees + longitudeRawMinutes / 60.0)

    # get altitude in meters (9 is above sea level, 11 is sea level above ellipsoide) with 0 reference at the ellipsoide
    altitude = float(sentencesData[9]) + float(sentencesData[11])

    # get covariance using Horizontal dilution of position
    hdop = float(sentencesData[8])
    position_covariance = [0,0,0,0,0,0,0,0,0]
    position_covariance[0] = hdop ** 2
    position_covariance[1] = 0.0
    position_covariance[2] = 0.0
    position_covariance[3] = 0.0
    position_covariance[4] = hdop ** 2
    position_covariance[5] = 0.0
    position_covariance[6] = 0.0
    position_covariance[7] = 0.0
    position_covariance[8] = (2 * hdop) ** 2
    position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    return timestamp, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type

def get_position(latitude, longitude, altitude):

    global initialx
    global initialy
    global initialz
    (x, y, zoneNumber, zoneLetter) = utm.from_latlon(latitude,longitude)
    z = altitude 
    if (initialx == 0):
        initialx = x
        initialy = y
        initialz = z

    # compute Position w.r.t the starting position
    gps_x = (x-initialx)
    gps_y = (y-initialy)
    gps_z = (z-initialz)

    return gps_x, gps_y, gps_z

if __name__ == '__main__':


    parser = argparse.ArgumentParser()
    parser.description = "Script that takes the raw gps data and creates a file with timestamp x y z"
    parser.add_argument(
        '-i',
        '--input',
        required=True,
        help="raw gps data with nmea sentences")

    parser.add_argument(
        '-o',
        '--output',
        required=False,
        help="out file in TUM format")
    args = parser.parse_args()

    initialx = 0
    initialy = 0
    initialz = 0

    f_in = open(args.input, 'rw')
    f_out = open(args.output, 'w+')

    seq_GGA = 0
    for line in f_in:
        if "GGA" in line:
            timestamp, status, service, latitude, longitude, altitude, position_covariance, position_covariance_type = get_gps_data_fromGGA(line)
            x, y, z = get_position(longitude, latitude, altitude)
            f_out.write(str(timestamp) + ' ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + "0" + ' ' + "0" + ' ' + "0" + ' ' + "1" + "\r\n")