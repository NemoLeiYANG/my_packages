from mpl_toolkits.mplot3d import Axes3D # ALWAYS IMPORT Axes3D BEFORE pandas AND PLT
from math import sin, cos, sqrt
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
	file = ['/home/echo/ndt_custom/map_pose_1.csv', 'r', 'ongoing', '-', 1]
	data = pd.read_csv(file[0])

	# added-scan-only filtering
	# data = [item[item['key'] > 0] for item in data]

	# off-set
	# data[0]['z'] -= 2
	# data[0] = data[0][7:]
	# data[1] = data[1][120:600]

	# Process
	local_list = []
	localized_poses = []
	# for i in range(1, 1000):
	for i in range(1, len(data['x'])):
		dist_x = data['x'][i] - data['x'][i-1]
		dist_y = data['y'][i] - data['y'][i-1]
		distance = sqrt(dist_x * dist_x + dist_y * dist_y)
		if distance < 0.05:
			local_list.append(i)
		else:
			if(len(local_list) > 15):
				localized_poses.append(local_list[int(len(local_list) / 2)])
			local_list = []

		print('Distance: {}, local size: {}'.format(distance, len(local_list)))
	print('Found {} positions'.format(len(localized_poses)))
	result_df = data.iloc[localized_poses]

	ax = plt.figure('3d-plot').add_subplot(111, projection='3d')
	plt.axis('equal')
	mpl.rcParams['legend.fontsize'] = 10

	#############################
	# 3D Plotting
	# Plot scatter (points)
	ax.scatter(result_df['x'], result_df['y'], zs=result_df['z'], 
						 color=file[1], label=file[2], s=file[4])
	# ax.set_xlim3d(-20,50)
	# ax.set_ylim3d(-100,500)
	# ax.set_zlim3d(-100,100)
	# ax.autoscale(enable=True, tight=True)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	ax.legend()
	# bagfile_read.main(ax) # add pose plotting from a bag file, see bagfile_read.py

	# #############################
	# # 2D plotting
	plt.figure('2d-plot')
	plt.scatter(result_df['x'], result_df['y'], 
							color=file[1], label=file[2], s=0.5)
	plt.gca().set_aspect('equal', adjustable='datalim')
	plt.xlabel('x')
	plt.ylabel('y')
	plt.legend()
	
	# #############################
	# # 1D plotting
	# plt.figure('z-plot')
	# travel_dist = [0]*len(data['x'])
	# for j in range(1, len(data['x'])):
	# 	dx = data.iloc[j]['x'] - data.iloc[j-1]['x']
	# 	dy = data.iloc[j]['y'] - data.iloc[j-1]['y']
	# 	dz = data.iloc[j]['z'] - data.iloc[j-1]['z']
	# 	travel_dist[j] = travel_dist[j-1] + np.sqrt(dx**2 + dy**2 + dz**2)

	# plt.scatter(travel_dist, data['z'], 
	# 						color=file[1], label=file[2], s=0.5)
	# plt.legend()
	# plt.xlabel('Travel distance')
	# plt.ylabel('z')

	plt.show()