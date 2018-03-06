#! /usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D # ALWAYS IMPORT Axes3D BEFORE pandas AND PLT
from math import sin, cos, sqrt
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
	file = ['/home/zwu/9feb-datacollection/localizing_pose.csv', 'r', 'pose', '-', 1]
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
	previous_cluster = [0, 0]
	distance_array = []
	# for i in range(1, 1000):
	for i in range(1, len(data['x'])):
		dist_x = data['x'][i] - data['x'][i-1]
		dist_y = data['y'][i] - data['y'][i-1]
		distance = sqrt(dist_x * dist_x + dist_y * dist_y)
		if distance < 0.05:
			local_list.append(i)
		else:
			if(len(local_list) > 15): #new cluster!
				new_index = local_list[int(len(local_list) / 2)]
				cluster_dist_x = data['x'][new_index] - previous_cluster[0]
				cluster_dist_y = data['y'][new_index] - previous_cluster[1]
				cluster_dist = sqrt(cluster_dist_x * cluster_dist_x + cluster_dist_y * cluster_dist_y)
				if(len(local_list) <= 20):
					print('Discarding cluster @ index {}/{} since its too small ({})'.format(new_index, len(localized_poses) + 1, len(local_list)))
				elif(cluster_dist < 0.5):
					print('Discarding cluster @ index {}/{} since its too close to the prev cluster'.format(new_index, len(localized_poses) + 1))
				else:
					localized_poses.append(new_index)
					distance_array.append(cluster_dist)
				previous_cluster = [data['x'][new_index], data['y'][new_index]]
				# print('Distance: {}, \tlocal size: {}, \tindex: {}'.format(cluster_dist, len(local_list), new_index))
			local_list = []

	print('Found {} positions'.format(len(localized_poses)))
	# result_df = data.iloc[localized_poses]
	data.loc[localized_poses, 'key'] = 1
	result_df = data
	# result_df['timestamp'] = (result_df['sec'] * 1e6 + result_df['nsec'] / 1e3).astype(int)
	columns_to_write = ['key', 'sequence', 'sec', 'nsec', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
	result_df.to_csv('/home/zwu/9feb-datacollection/maxima_pose.csv', cols=columns_to_write, index=False)

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
	plt.figure('dist hist')
	plt.hist(distance_array, bins=100)

	plt.figure('time diff hist')
	timediff_array = []
	for i in range(1, len(result_df['sec'])):
		timediff = float(result_df.iloc[i]['sec'] - result_df.iloc[i-1]['sec']) + (result_df.iloc[i]['nsec'] - result_df.iloc[i-1]['nsec']) * 1e-9
		timediff_array.append(timediff)

	plt.plot(range(len(timediff_array)), timediff_array)
	# plt.hist(timediff_array, bins = 100)

	plt.show()