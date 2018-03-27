#! /usr/bin/env python
from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def img_to_id(name):
  # Note: default name is: Bw_%08d.png
  id = name[3:-4]
  return id

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
  file = ['/home/zwu/12mar_data/bw-lidar-2/allignment.csv', 'r', 'new', '-', 1]
  data = pd.read_csv(file[0])

  # data['x'] = data['image']
  # print(img_to_id(data['image'].iloc[0]))
  print(data['omag'])

	# plt.figure('2d-plot')
	# plt.scatter(x, y,color=file[1], label=file[2], s=0.5)
	# plt.gca().set_aspect('equal', adjustable='datalim')
	# plt.xlabel('x')
	# plt.ylabel('y')
	# plt.legend()
	# plt.show()

  print(data.info())