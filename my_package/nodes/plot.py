#! /usr/bin/env python
from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
	file = ['/home/zwu/12mar_data/rgb-lidar/round1/ndt_matching_20180319_153314.csv', 'r', 'new', '-', 1]
	data = pd.read_csv(file[0], header=None)

	seq = data[0]
	
	x = data[4]
	y = data[5]
	z = data[6]
	roll = data[7]
	pitch = data[8]
	yaw = data[9]

	out_data = {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
	out_data = pd.DataFrame(out_data)
	out_data = out_data[['x', 'y', 'z', 'roll', 'pitch', 'yaw']]
	# out_data.to_csv('/home/zwu/12mar_data/bw-lidar-2/localized_pose.csv', index=False, float_format='%.6f')
	# plt.figure('2d-plot')
	# plt.scatter(x, y,color=file[1], label=file[2], s=0.5)
	# plt.gca().set_aspect('equal', adjustable='datalim')
	# plt.xlabel('x')
	# plt.ylabel('y')
	# plt.legend()
	# plt.show()