#! /usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

if __name__ == '__main__':
  f_in = '/home/zwu/14dec_data/LidarTimestamp.csv'
  data = pd.read_csv(f_in, header=None, names=['timestamp'])
  
  for i in range(1, data.shape[0]):
    timediff = data.iloc[i]['timestamp'] - data.iloc[i-1]['timestamp']
    if abs(timediff - 1e5) > 5e4:
      print('{}: {}'.format(i, timediff))
