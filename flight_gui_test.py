# -*- coding: utf-8 -*-
"""
Created on Sat Dec  3 11:31:07 2022

@author: tcyok
"""

import numpy as np
import matplotlib.pyplot as plt
import csv

with open('test_data.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))

data = data[1]

speed = float(data[0])
x_lim = float(data[1])
y_lim = float(data[2])
pattern = data[3]
print(data)

if pattern == 'Square Pattern Single-Unit':
    x_square = [0,0,x_lim,x_lim,x_lim/4,x_lim/4,3*x_lim/4,3*x_lim/4,x_lim/2,x_lim/2]
    y_square = [0,y_lim,y_lim,0,0,3*y_lim/4,3*y_lim/4,y_lim/4,y_lim/4,y_lim/2]


i = 0
while i < len(x_square):
    
    if i == len(x_square)-2:
        break
    
    x_1 = [x_square[i],x_square[i+1]]
    y_1 = [y_square[i],y_square[i+1]]
    plt.plot(x_1,y_1,'r-')
    i = i+1

z = np.zeros(len(x_square))+0.5
plt.xlim([-1,3])
plt.ylim([-1,3])
