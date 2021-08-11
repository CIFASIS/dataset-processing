#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rospy
import math
import numpy as np
from utils import plotHelpers as ph
import matplotlib.pyplot as plt
import utm


# SOURCE: http://www.gpsinformation.org/dale/nmea.htm#GGA

#The most important NMEA sentences include the GGA which provides the current Fix data, the RMC which provides the minimum gps sentences information, and the GSA which provides the Satellite status data.

#GGA - essential fix data which provide 3D location and accuracy data.

# $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

#Where:
#     GGA          Global Positioning System Fix Data
#     123519       Fix taken at 12:35:19 UTC
#     4807.038,N   Latitude 48 deg 07.038' N
#     01131.000,E  Longitude 11 deg 31.000' E
#     1            Fix quality: 0 = invalid
#                               1 = GPS fix (SPS)
#                               2 = DGPS fix
#                               3 = PPS fix
#             4 = Real Time Kinematic
#             5 = Float RTK
#                               6 = estimated (dead reckoning) (2.3 feature)
#             7 = Manual input mode
#             8 = Simulation mode
#     08           Number of satellites being tracked
#     0.9          Horizontal dilution of position
#     545.4,M      Altitude, Meters, above mean sea level
#     46.9,M       Height of geoid (mean sea level) above WGS84
#                      ellipsoid
#     (empty field) time in seconds since last DGPS update
#     (empty field) DGPS station ID number
#     *47          the checksum data, always begins with *



# get LLH values from GGA nmea sentences
def getLLHfromGGASensence( sentence ):

  sentencesData = sentence.split(',')

  # get latitude in degrees
  latitudeRaw = float(sentencesData[2])
  latitudeRawDegrees = latitudeRaw // 100 # division entera
  latitudeRawMinutes = latitudeRaw % 100


  # Get the sign of the latitude. It depends if latitude is North or South
  latitudeSign = 1
  latitudeCartidnalDirection = sentencesData[3]
  if latitudeCartidnalDirection == 'S':
    latitudeSign = -1

  latitude = latitudeSign * (latitudeRawDegrees + latitudeRawMinutes / 60.0)


#  print "latitudeRaw: " + str(latitudeRaw)
#  print "latitudeRawDegrees: " + str(latitudeRawDegrees)
#  print "latitudeRawMinutes: " + str(latitudeRawMinutes)

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

#  print "longitudeRaw: " + str(longitudeRaw)
#  print "longitudeRawDegrees: " + str(longitudeRawDegrees)
#  print "longitudeRawMinutes: " + str(longitudeRawMinutes)


  # altitude is already in meters
  altitude = float( sentencesData[9] )

  return latitude, longitude, altitude


def unit_vector(vector):
  """ Returns the unit vector of the vector.  """
  return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
  """ Returns the angle in radians between vectors 'v1' and 'v2'::

      >>> angle_between((1, 0, 0), (0, 1, 0))
      1.5707963267948966
      >>> angle_between((1, 0, 0), (1, 0, 0))
      0.0
      >>> angle_between((1, 0, 0), (-1, 0, 0))
      3.141592653589793
  """
  v1_u = unit_vector(v1)
  v2_u = unit_vector(v2)
  return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def compute_pose_orientations(positions):

  """ Compute orientation from a path (array of positions)
      Given two consecutive positions, compute the vector direction between them
      Note: it should consider the case where the second position is behind the first one (e.g. robot moving backwards)
  """

  orientations = []
  prev_orientation = [0,0,0]
  for pos1, pos2 in zip(positions, positions[1:]):
    orientation = unit_vector(pos2 - pos1)
    # TODO: We need to consider the first orientation as identity.
    angle_between_orientation_vectors = angle_between(orientation, prev_orientation) * 180.0 / math.pi # convert to degrees

    # TODO: We need to compute the quaternion

    # if the angle between two consecutive orientation is larger than 90 degrees, the robot should be still or moving backwards
    if abs(angle_between_orientation_vectors) > 90:
      print("change moving direction. Robot is still or should be moving backward")
    prev_orientation = orientation

    orientations.append( orientation )

  # repeat the last orientation just to have the same length than positions
  orientations.append( prev_orientation )
  return orientations


if __name__ == "__main__":

  parser = argparse.ArgumentParser(description='Script that create a rosbag containing NMEA ROS sentences')
  parser.add_argument('gps_nmea', help='GPS log file')

  args = parser.parse_args()

  inputFile = open(args.gps_nmea, 'r')

  inputFileName = args.gps_nmea[:-4] # remove the extension (.log) of the input file
  outputFileName = str(inputFileName) + ".output" # add extension .output

  outputFile = open(outputFileName, 'w')

  initialPosition = np.array([])
  pos_grnd = []

  for line in inputFile:  #for each line in raw file, take each part: time,id,msg


    # message data
    sentence = line.split(' ')[2][:-2]  # message from the type of NMEA sentence

    # we process only GGA sentences because they are the most informative
    if "GGA" not in sentence:
      continue


    # get timestamp
    timestamp = line.split(' ')[0]

    # get seconds
    seconds = int(timestamp.split('.')[0])

    # get milliseconds and convert them to nanoseconds
    nanoseconds = int(timestamp.split('.')[1]) * 1000

    # get latitude, longitude and altitud from GGA sentence

    latitude, longitude, altitude = getLLHfromGGASensence( sentence )

    (easting, northing, zoneNumber, zoneLetter) = utm.from_latlon(latitude,longitude)
    x = easting
    y = northing


    # NOTE: we keep the altitude as our Z component

    # check if initial position was set, if not, set it
    if not initialPosition.size:
      initialPosition = np.array((x, y, altitude))

    # compute Position w.r.t the starting position
    position = np.array((x, y, altitude)) - initialPosition


    pos_grnd.append( position )

    outputFile.write(timestamp +' '+ str(position[0]) +' '+ str(position[1]) +' '+ str(position[2]) + '\n')


  # compute orientation for each pose
  orientation_grnd = compute_pose_orientations(pos_grnd)

  ####################################################################
  # Close files
  ####################################################################

  print("Output saved on {}".format(outputFileName))

  inputFile.close()
  outputFile.close()


  ####################################################################
  # General stuffs for plotting
  ####################################################################

  # convert data to numpy arrays
  pos_grnd = np.array( pos_grnd )
  orientation_grnd = np.array(orientation_grnd)


  xy_path = []
  zy_path = []
  lines3D = []
  orientations3D = []

  x_grnd = pos_grnd[:,0]
  y_grnd = pos_grnd[:,1]
  z_grnd = pos_grnd[:,2]

  xy_path.append( (x_grnd, y_grnd) )
  zy_path.append( (z_grnd, y_grnd) )
  lines3D.append( (x_grnd, y_grnd, z_grnd) )

  labels = np.array([ "GPS-RTK" ])
  colors = np.array( ["black"] )
  ph.plotPaths2D( xy_path,  labels, colors)

  ph.plotPaths2D( zy_path, labels, colors)

  ph.plotPaths3D( lines3D, labels, colors )

  colors = np.array( [('r','b','g')] )
  orientations3D.append( (orientation_grnd[:,0], orientation_grnd[:,1], orientation_grnd[:,2]) )
  ph.plotPathWithOrientation3D(lines3D, orientations3D, labels, colors)

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()

  ####################################################################
  # quit script
  ####################################################################

  quit()
