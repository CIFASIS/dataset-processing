import argparse
import utm
from sensor_msgs.msg import NavSatFix, NavSatStatus


def get_gps_data_fromGGA(line):
    timestamp = line.split(' ')[0] + "000"  # nanoseconds precision
    sentencesData = line.split(',')

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

    # altitude is already in meters
    altitude = float(sentencesData[9])

    return timestamp, latitude, longitude, altitude


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
            timestamp, latitude, longitude, altitude = get_gps_data_fromGGA(line)
            x, y, z = get_position(latitude, longitude, altitude)
            f_out.write(str(timestamp) + ' ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + "0" + ' ' + "0" + ' ' + "0" + ' ' + "1" + "\r\n")

    f_in.close()
    f_out.close()
