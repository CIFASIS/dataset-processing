import numpy as np
from numpy.linalg import inv
import argparse
import yaml
import cv2
import rospy
from sensor_msgs.msg import CameraInfo


# read the yaml file and get the information for the camera calibration, the camera is parse as arg (cam0 = left , cam1 = right)
def get_camera_info(camera_info, camera):
  with open(camera_info, 'r') as stream:
    try:
      data = yaml.load(stream)
      camera_info = CameraInfo()
      T=[0,0,0]
      camera_info.width = data[camera]['resolution'][0]
      camera_info.height = data[camera]['resolution'][1]
      if data[camera]['distortion_model'] == "radtan":
        camera_info.distortion_model = "plumb_bob"
      else:
        camera_info.distortion_model = data[camera]['distortion_model']

      fx,fy,cx,cy = data[camera]['intrinsics']
      camera_info.K[0:3] = [fx, 0, cx]
      camera_info.K[3:6] = [0, fy, cy]
      camera_info.K[6:9] = [0, 0, 1]

      k1,k2,t1,t2 = data[camera]['distortion_coeffs']
      camera_info.D = [k1,k2,t1,t2,0]
      #if cam0 then it's left camera, so R = identity and T = [0 0 0]
      if camera == "cam0":
        camera_info.R[0:3] = [1, 0, 0]
        camera_info.R[3:6] = [0, 1, 0]
        camera_info.R[6:9] = [0, 0, 1]
      else:
        camera_info.R[0:3] = data[camera]['T_cn_cnm1'][0][:3]
        camera_info.R[3:6] = data[camera]['T_cn_cnm1'][1][:3]
        camera_info.R[6:9] = data[camera]['T_cn_cnm1'][2][:3]
        T[0:3] = [data[camera]['T_cn_cnm1'][0][3], data[camera]['T_cn_cnm1'][1][3], data[camera]['T_cn_cnm1'][2][3]]

    except yaml.YAMLError as exc:
      print(exc)

  return camera_info, T

# get the image from the path and the parameters from the name
def rectify_images(cam0, cam1, T):
  R1_rectified = np.zeros((3,3))
  R2_rectified = np.zeros((3,3))
  P1_rectified = np.zeros((3,4))
  P2_rectified = np.zeros((3,4))
  Q_rectified = np.zeros((4,4))
  flags = cv2.CALIB_ZERO_DISPARITY
  alpha = -1


  R = np.reshape(cam1.R,(3,3))
  t = np.reshape(T,(3,1))

  print("R")
  print(R)
  print("t")
  print(t)

  cv2.stereoRectify(np.reshape(cam0.K,(3,3)), np.reshape(cam0.D,(5,1)), np.reshape(cam1.K,(3,3)), np.reshape(cam1.D,(5,1)), (cam0.width, cam0.height), np.reshape(cam1.R,(3,3)), np.reshape(T,(3,1)), R1_rectified, R2_rectified, P1_rectified, P2_rectified, Q_rectified, flags, alpha)
  cam0.R = list(R1_rectified.flat)
  cam0.P = list(P1_rectified.flat)
  cam1.R = list(R2_rectified.flat)
  cam1.P = list(P2_rectified.flat)


#  print("R1_rectified")
#  print(R1_rectified)
#  print("R2_rectified")
#  print(R2_rectified)

  recoveredR = R1_rectified.dot(inv(R2_rectified))
  print(recoveredR)

  # tengo que obtener el baseline
  # para esto tengo que dividir tx/fx_rectificado
  # luego creo que tengo que multiplicar por la inversa de R2

  fx = P2_rectified[0][0]
  rectifyt = np.array(P2_rectified[:,3]) / fx

  print(rectifyt)
  recoveredt = inv(R2_rectified).dot(rectifyt)
  print(recoveredt)


  return cam0, cam1

if __name__ == "__main__":

  parser = argparse.ArgumentParser()
  parser.description = 'Script that generates the inverse of the stereoRectify OpenCV function'
  parser.add_argument(
    '--calibration',
    required=True,
    help='yaml file with the calibration')
  args = parser.parse_args()

  if args.calibration:
    camera_info = [0, 0]
    camera_info_rect = [0, 0]
    for i in range(0,2):
      camera_info[i], T = get_camera_info(args.calibration, "cam" + str(i))

    camera_info_rect[0], camera_info_rect[1] = rectify_images(camera_info[0], camera_info[1], T)

