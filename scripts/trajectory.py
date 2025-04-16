#!/usr/bin/python3
# coding=utf-8
import time
import numpy as np
import matplotlib.pyplot as plt
import math


def parabolicBlends(point, duration, tb, dt):
    v12 = (point[1]-point[0])/(duration[0]-0.5*tb)
    step = int(sum(duration)/dt)+1
    duration[len(duration)-1] = duration[len(duration)-1]-0.5*tb
    a1 = v12/tb
    x = np.zeros(int(step))
    v = np.zeros(int(step))
    t = np.zeros(int(step))
    count = 0
    x[0] = point[0]
    v[0] = 0
    t[0] = 0
    for i in range(int((duration[0]-0.5*tb)/dt)):
        if t[i] < tb :
            v[i+1] = v[i]+a1*dt
        if t[i]>=tb:
            v[i+1]=v[i]
        x[i+1] = x[i]+v[i]*dt
        t[i+1] = t[i]+dt
        count = count+1      
    for i in range(len(duration)-1):
        vjk = (point[i+2]-point[i+1])/duration[i+1]
        aj = (vjk - v[count-1])/tb       
        startStep = count
        for a in range(int((duration[i+1])/dt)):
            if t[startStep+a] <t[startStep]+tb :
                v[startStep+a+1] = v[startStep+a]+aj*dt
            if t[startStep+a]>=t[startStep]+tb:
                v[startStep+a+1]=v[startStep+a]
            x[startStep+a+1] = x[startStep+a]+v[startStep+a]*dt
            t[startStep+a+1] = t[startStep+a]+dt
            count = count+1
    vn = (point[len(point)-1]-point[len(point)-2])/duration[len(duration)-1]
    an = -vn /tb
    startStep = count
    for b in range(int(tb/dt)):
        v[startStep+b+1] = v[startStep+b]+an*dt
        x[startStep+b+1] = x[startStep+b]+v[startStep+b]*dt
        t[startStep+b+1] = t[startStep+b]+dt
        count = count+1
    if t[step-1] == 0:
        v[step-1] = v[step-2]+an*dt
        x[step-1] = x[step-2]+v[step-2]*dt
        t[step-1] = t[step-2]+dt
        
    return x,v,t
#if __name__ == "__main__":
#   x1,v1, t1 = parabolicBlends([0,300*math.cos(math.radians(234)), 100*math.cos(math.radians(198)),300*math.cos(math.radians(162)), 100*math.cos(math.radians(126)), 300*math.cos(math.radians(90)), 100*math.cos(math.radians(54)), 300*math.cos(math.radians(18)), 100*math.cos(math.radians(342)),300*math.cos(math.radians(306)),0 ], [10,10,10,10,10,10,10,10,10,10], 3, 0.04)
#   y1,vy, t2 = parabolicBlends([-100,300*math.sin(math.radians(234)), 100*math.sin(math.radians(198)),300*math.sin(math.radians(162)), 100*math.sin(math.radians(126)), 300*math.sin(math.radians(90)), 100*math.sin(math.radians(54)), 300*math.sin(math.radians(18)), 100*math.sin(math.radians(342)),300*math.sin(math.radians(306)),-100], [10,10,10,10,10,10,10,10,10,10], 3, 0.04)  

#   trajectory_points = np.zeros([len(x1), 3])       
#   trajectory_points[:,0] = x1
#   trajectory_points[:,1] = y1

#   fig, ax = plt.subplots()
#   ax.plot(x1, y1, 'r-', label='Trajectory')
#   ax.set_aspect('equal')
#   ax.set_xlabel('x_set_position')
#   ax.set_ylabel('y_set_position')
#   ax.legend()
#   plt.title('Parabolic Blend Trajectory')
#   plt.show()

