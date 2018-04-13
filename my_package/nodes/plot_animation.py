#! /usr/bin/env python

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import time

if __name__=='__main__':
	# import data
	# [file name, color, label, linestyle, linewidth]
	file = [
					['/home/zwu/Desktop/vo.csv', 'cyan', 'trajectory', '-', 1],
					['/home/zwu/Desktop/gps.csv', 'r', 'ground-truth', '-', 1],
					['/home/zwu/Desktop/gps.csv', 'b', 'GPS', '-', 1],
				 ]
	data = [pd.read_csv(element[0]) for element in file]
	# off-set
	# data[0]['z'] -= 2
	# data[0] = data[0][7:]
	# data[1] = data[1][120:600]

	# Generate fake noise to simulate GPS data
	for i in range(len(data[2]['x'])):
		data[2]['x'][i] += np.random.normal(0, scale=0.4)
		data[2]['y'][i] += np.random.normal(0, scale=0.4)

	fig = plt.figure('2d-plot')
	fig = plt.gcf()
	fig.show()
	fig.canvas.draw()
	i = 1
	init = False
	while True:
		# 2D plotting
		# fig.clear()
		for j in range(len(file)):
			# plt.scatter(data[j]['x'][i], data[j]['y'][i], color=file[j][1], label=file[j][2], s=0.5)
			plt.plot(data[j]['x'][i-1:i+1], data[j]['y'][i-1:i+1], color=file[j][1], label=file[j][2], lw=1.0)
		plt.gca().set_aspect('equal', adjustable='datalim')
		if init == False:
			plt.xlabel('x')
			plt.ylabel('y')
			# plt.xlim([-100, 500])
			# plt.ylim([-500, 500])
			plt.legend()
			init = True
		fig.canvas.draw()
		if i < len(data[0]['x']):
			i += 1
		else:
			break
		
	plt.show()