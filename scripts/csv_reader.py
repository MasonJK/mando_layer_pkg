#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

file_length = 10            # number of files
pedestrian_length = 16      # number of pedestrians
data_length = 500           # number of data in one file

data_x = np.zeros((file_length, pedestrian_length, data_length))
data_y = np.zeros((file_length, pedestrian_length, data_length))

print(data_x.shape)
for i in range(file_length):
    file = pd.read_csv('~/costmap_ws/src/csv_reader/bagfile/guidance_data/pedsim'+str(i+1)+'.csv')
    for j in range(pedestrian_length):
        for k in range(data_length):
            data_x[i][j][k] = file['field.agent_states'+str(j)+'.pose.position.x'][k]
            data_y[i][j][k] = file['field.agent_states'+str(j)+'.pose.position.y'][k]

print(data_x)
print(data_y)