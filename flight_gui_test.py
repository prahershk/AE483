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

if pattern == "['Square Pattern Single-Unit']":
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

if pattern == "['Sector Pattern Single-Unit']":
    x_center = x_lim/2
    y_center = y_lim/2
    h = x_lim/2
    k = y_lim/2

    t = np.linspace(0, 2*np.pi, 1000)
    
    x_sector = np.array([h*np.cos(4*np.pi/3),h*np.cos(np.pi),h*np.cos(2*np.pi/3),h*np.cos(np.pi/3),h*np.cos(0),h*np.cos(5*np.pi/3)])
    x_sector = x_sector+x_center
    y_sector = np.array([k*np.sin(4*np.pi/3),k*np.sin(np.pi),k*np.sin(2*np.pi/3),k*np.sin(np.pi/3),k*np.sin(0),k*np.sin(5*np.pi/3)])
    y_sector = y_sector+y_center
    
    
    plt.plot(x_sector,y_sector,'go')
    plt.plot(x_center + h*np.cos(t), y_center + k*np.sin(t), 'r-')
    
    plt.xlim([0,0.5])
    plt.ylim([0,0.5])
    
if pattern == "['Parallel Single-Unit Spiral']":
    x_center = x_lim/2
    y_center = y_lim/2
    count = 20
    h = np.linspace(x_lim/2,0,count)
    k = np.linspace(y_lim/2,0,count)

    t = np.linspace(3*np.pi/2, 11*np.pi/2, count)
    
    x_spiral = []
    y_spiral = []
    
    i = 0
    while i < len(t):
        x_spiral.append(h[i]*np.cos(t[i]))
        y_spiral.append(k[i]*np.sin(t[i]))
        i = i+1
    
    x_spiral = np.array([x_spiral])+x_center
    y_spiral = np.array([y_spiral])+y_center

    plt.plot(x_spiral,y_spiral,'ro')
    
    plt.xlim([0,0.75])
    plt.ylim([0,0.75])

    """
    x_sector = np.array([h*np.cos(4*np.pi/3),h*np.cos(np.pi),h*np.cos(2*np.pi/3),h*np.cos(np.pi/3),h*np.cos(0),h*np.cos(5*np.pi/3)])
    x_sector = x_sector+x_center
    y_sector = np.array([k*np.sin(4*np.pi/3),k*np.sin(np.pi),k*np.sin(2*np.pi/3),k*np.sin(np.pi/3),k*np.sin(0),k*np.sin(5*np.pi/3)])
    y_sector = y_sector+y_center
    
    
    plt.plot(x_sector,y_sector,'go')
    plt.plot(x_center + h*np.cos(t), y_center + k*np.sin(t), 'r-')
    
    plt.xlim([0,0.5])
    plt.ylim([0,0.5])
    """



"""
    ['Sector Pattern Single-Unit']
if pattern == "['Square Pattern Single-Unit']":
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
"""








