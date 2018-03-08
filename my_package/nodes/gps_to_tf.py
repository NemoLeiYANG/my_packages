#! /usr/bin/python
from __future__ import print_function
import sys

import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
from osgeo import gdal, osr

import ros, rospy, rosbag
import tf
from tf.msg import tfMessage
import geometry_msgs

if __name__ == '__main__':
  if len(sys.argv) < 3:
    print('Missing input arg')
    print('Usage: rosrun my_package gps_to_tf INPUT.csv OUTPUT.bag')
    sys.exit()

  fData = pd.read_csv(sys.argv[1])
  bag = rosbag.Bag(sys.argv[2], 'w')

  inSpatialRef = osr.SpatialReference()
  inSpatialRef.SetWellKnownGeogCS('WGS84')
  outSpatialRef = osr.SpatialReference()
  outSpatialRef.ImportFromEPSG(3414)
  transform = osr.CoordinateTransformation(inSpatialRef, outSpatialRef)
 
  # Note that we use ENU coordinate system
  GpsOffsetTime = 315964800 # secs from 1-Jan-1970 to 6-Jan-1980
  for i in range(len(fData['week'])):
    rosTime = rospy.Time(fData['week'][i]*604800 + fData['second'][i])
    geoMsg = geometry_msgs.msg.TransformStamped()
    geoMsg.header.seq = i
    geoMsg.header.stamp = rosTime
    geoMsg.header.frame_id = 'map'
    geoMsg.child_frame_id = 'base_link'

    x_, y_, z_ = transform.TransformPoint(fData['longitude'][i], fData['latitude'][i], fData['height'][i])
    geoMsg.transform.translation.x = x_
    geoMsg.transform.translation.y = y_
    geoMsg.transform.translation.z = z_

    roll_imu_ = float(fData['roll'][i]) * math.pi / 180.0
    pitch_imu_ = float(fData['pitch'][i]) * math.pi / 180.0
    azimuth_imu_ = float(fData['azimuth'][i]) * math.pi / 180.0
    quat1 = tf.transformations.quaternion_about_axis(roll_imu_, (1, 0, 0))
    quat2 = tf.transformations.quaternion_about_axis(pitch_imu_, (0, -1, 0))
    quat3 = tf.transformations.quaternion_about_axis(-azimuth_imu_, (0, 0, 1))
    tf_quat = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(quat1, quat2), quat3)

    geoMsg.transform.rotation.x = tf_quat[0]
    geoMsg.transform.rotation.y = tf_quat[1]
    geoMsg.transform.rotation.z = tf_quat[2]
    geoMsg.transform.rotation.w = tf_quat[3]

    tfMsg = tfMessage()
    tfMsg.transforms.append(geoMsg)
    bag.write('/tf', tfMsg, t=rosTime)
  
  bag.close()
  print('Finished')

