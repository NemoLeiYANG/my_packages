#! /usr/bin/python
from __future__ import print_function
import csv
import sys

import numpy as np
import pandas as pd

if __name__ == '__main__':
  if(len(sys.argv) < 2):
    print('Missing input file argument')
    sys.exit()

  fName = sys.argv[1]
  isInsRow = True # to check alternating between INS and POS rows
  fHeader = ['week', 'second', 'latitude', 'longitude', 'height', 'north_vel', 'east_vel', 'up_vel', 'roll', 'pitch', 'azimuth']
  fData = []
  with open(fName, 'r') as f:
    file = csv.reader(f)
    for line in file:
      if line[0] != '%INSPVASA':
        if line[0] == '#BESTPOSA':
          continue
        else:
          print('Unknown line: {}'.format(line))
          break
      addLine = []
      addLine.append(line[1])
      addLine.append(line[3])
      addLine.extend(line[4:13])
      fData.append(addLine)

  fDF = pd.DataFrame(fData, columns=fHeader)
  print(fDF.info())
  fDF.to_csv('gps.csv', index=False)