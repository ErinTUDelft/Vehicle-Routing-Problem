# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 11:36:09 2022

@author: Pietro Deligios
"""

from project import trip_list, steps, Node_coords, x_max, y_max
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# Position Arrays
Time_array = trip_list[0].t
N = np.size(Node_coords,0) 


def animate_func(num):
    ax.clear()  # Clears the figure to update the line, point, title, and axes

    # Plotting the nodes
    ax.plot(Node_coords[0,0], Node_coords[0,1], 'ro')
    ax.plot(Node_coords[:,0], Node_coords[:,1], 'x')
    
    
    for n in range(0,N):
        ax.annotate(str(n), (Node_coords[n,0]+3, Node_coords[n,1]))
    
    for i in range(len(trip_list)):
        trace = np.array([trip_list[i].x, trip_list[i].y])
        clr = plt.cm.tab20(trip_list[i].drone)
        # checking if drone is not over origin node
        if trace[0, num] != Node_coords[0,0] or trace[1, num] != Node_coords[0,1]:
            # plotting the drone position
            ax.scatter(trace[0, num], trace[1, num], color=clr, marker='o')
            
            # plotting drone name
            ax.annotate(f'D{trip_list[i].drone}-T{trip_list[i].trip_n}', (trace[0, num]+3, trace[1, num]-10))
            
            # plotting drone trace
            ax.plot(trace[0, :num+1], trace[1, :num+1], color=clr)
        else:
            # plotting faded and dashed drone drace
            ax.plot(trace[0, :num+1], trace[1, :num+1],':', color=clr, zorder=-1, alpha=0.3)

    # Setting Axes Limits
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)

    # Adding Figure Labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')

# Plotting the Animation
fig = plt.figure(dpi = 200)
ax = plt.axes()
anim = animation.FuncAnimation(fig, animate_func, frames=steps)

f = r"animate_drones.gif"
writergif = animation.PillowWriter(fps = steps/10)
writergif.setup(fig,f)
anim.save(f, writer=writergif)