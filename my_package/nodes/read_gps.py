#! /usr/bin/python
from __future__ import print_function
import sys

import numpy
import pandas as pd
import matplotlib.pyplot as plt
from osgeo import gdal, osr

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print('Missing input arg')
    sys.exit()

  fName = sys.argv[1]
  fData = pd.read_csv(fName)

  inSpatialRef = osr.SpatialReference()
  inSpatialRef.SetWellKnownGeogCS('WGS84')

  outSpatialRef = osr.SpatialReference()
  outSpatialRef.ImportFromEPSG(3414)
 
  transform = osr.CoordinateTransformation(inSpatialRef, outSpatialRef)
 
  x = []
  y = []
  z = []
  for i in range(len(fData['week'])):
    x_, y_, z_ = transform.TransformPoint(fData['longitude'][i], fData['latitude'][i], fData['height'][i])
    x.append(x_)
    y.append(y_)
    z.append(z_)

  ax = plt.figure(1)
  plt.axis('equal')
  plt.plot(x, y)
  plt.show()
