from mpl_toolkits.mplot3d import Axes3D # ALWAYS IMPORT Axes3D BEFORE pandas AND PLT
from math import sin, cos
import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
	file = [
					['/home/echo/ndt_custom/map_pose.csv', 'r', 'ongoing', '-', 1],
				 ]
	data = [pd.read_csv(element[0]) for element in file]

	# added-scan-only filtering
	# data = [item[item['key'] > 0] for item in data]

	# off-set
	# data[0]['z'] -= 2
	data[0] = data[0][7:]
	# data[1] = data[1][120:600]

	ax = plt.figure('3d-plot').add_subplot(111, projection='3d')
	# plt.axis('equal')
	mpl.rcParams['legend.fontsize'] = 10
	for i in range(len(file)):
		#############################
		# Pre-processing data
		# Process euler angles to visualize
		# total_size = data[i].shape[0]
		# arrow = np.empty(shape=[total_size, 3, 3])
		
		# arrow[i][0] = np.zeros(shape=[total_size, 3])
		# arrow[i][1] = np.zeros(shape=[total_size, 3])
		# arrow[i][2] = np.zeros(shape=[total_size, 3])
		# for j in range(total_size):
		# 	rot_mat = R(data[i]['roll'][j], data[i]['pitch'][j], data[i]['yaw'][j])
		# 	arrow[j][0] = np.matrix([1, 0, 0])*rot_mat
		# 	arrow[j][1] = np.matrix([0, 1, 0])*rot_mat
		# 	arrow[j][2] = np.matrix([0, 0, 1])*rot_mat

		#############################
		# 3D Plotting
		# Plot with lines
		ax.plot(data[i]['x'], data[i]['y'], zs=data[i]['z'], 
						color=file[i][1], label=file[i][2], ls=file[i][3], lw=file[i][4])

		# Plot scatter (points)
		# ax.scatter(data[i]['x'], data[i]['y'], zs=data[i]['z'], 
		# 					 color=file[i][1], label=file[i][2], s=file[i][4])

		# Plot pose (visualized as Oxyz)
		# ax.quiver(data[i]['x'], data[i]['y'], data[i]['z'], 
		# 					arrow[:,0,0], arrow[:,0,1], arrow[:,0,2],
		# 					pivot='tail', length=0.5, arrow_length_ratio=0.1, color='r', label=file[i][2])
		# ax.quiver(data[i]['x'], data[i]['y'], data[i]['z'], 
		# 					arrow[:,1,0], arrow[:,1,1], arrow[:,1,2],
		# 					pivot='tail', length=0.5, arrow_length_ratio=0.1, color='g', label=file[i][2])
		# ax.quiver(data[i]['x'], data[i]['y'], data[i]['z'], 
		# 					arrow[:,2,0], arrow[:,2,1], arrow[:,2,2],
		#					  pivot='tail', length=0.1, arrow_length_ratio=0.3, color='b', label=file[i][2])
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
		plt.scatter(data[i]['x'], data[i]['y'], 
								color=file[i][1], label=file[i][2], s=0.5)
		plt.gca().set_aspect('equal', adjustable='datalim')
		plt.xlabel('x')
		plt.ylabel('y')
		plt.legend()
		
		# #############################
		# # 1D plotting
		plt.figure('z-plot')
		travel_dist = [0]*len(data[i]['x'])
		for j in range(1, len(data[i]['x'])):
			dx = data[i].iloc[j]['x'] - data[i].iloc[j-1]['x']
			dy = data[i].iloc[j]['y'] - data[i].iloc[j-1]['y']
			dz = data[i].iloc[j]['z'] - data[i].iloc[j-1]['z']
			travel_dist[j] = travel_dist[j-1] + np.sqrt(dx**2 + dy**2 + dz**2)

		plt.scatter(travel_dist, data[i]['z'], 
								color=file[i][1], label=file[i][2], s=0.5)
		plt.legend()
		plt.xlabel('Travel distance')
		plt.ylabel('z')

	plt.show()