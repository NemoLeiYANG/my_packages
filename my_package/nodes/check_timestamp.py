#! /usr/bin/env python

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print('Please input an csv file')
    sys.exit(-1)
  
  f_in = sys.argv[1]
  fix_ts = False
  if len(sys.argv) == 3:
    print('Timestamps fixing function enabled')
    fix_ts = True
  # f_in = '/home/zwu/data-0409/round-rgb/1/LidarTimestamp.csv'
  data = pd.read_csv(f_in, header=None, names=['timestamp'])
  
  for i in range(1, data.shape[0]):
    timediff = data.iloc[i]['timestamp'] - data.iloc[i-1]['timestamp']
    if abs(timediff - 1e5) > 5e4:
      print('{}: {} ({} .. {})'.format(i, timediff, data.iloc[i-1]['timestamp'], data.iloc[i]['timestamp']))
    
      if fix_ts:
        if abs(timediff + 1.9e6) < 1e4:
          print('Timestamp is fixed by +2s')
          data.iloc[i]['timestamp'] += 2e6
        elif abs(timediff - 2.1e6) < 1e4:
          print('Timestamp is fixed by -2s')
          data.iloc[i]['timestamp'] -= 2e6
  
  if fix_ts:
    f_out = 'lidarTimestampAdjust.csv'
    data.to_csv(f_out, header=False, index=False)
    print('Wrote adjusted timestamp file to {}'.format(f_out))