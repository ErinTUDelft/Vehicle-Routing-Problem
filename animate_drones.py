# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 11:36:09 2022

@author: Pietro Deligios
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

def animate (trip_list, steps, node_list, x_max, y_max, U_max):
    # Position Arrays
    N = len(node_list)
    Node_x = []
    Node_y = []
    c = []
    for n in range(1,N):
        Node_x.append(node_list[n].coord[0])
        Node_y.append(node_list[n].coord[1])
        c.append(plt.cm.RdYlGn((255-node_list[n].urgency*(255/U_max)).astype(int)))
        
    def animate_func(num):
        ax.clear()  # Clears the figure to update the line, point, title, and axes
        s = []
        # Plotting the nodes
        ax.plot(node_list[0].coord[0], node_list[0].coord[1], 'ro')
        for n in range(1,N):
            #ax.plot(node_list[n].coord[0], node_list[n].coord[1], 'x')
            ax.annotate(str(n), (node_list[n].coord[0]+3, node_list[n].coord[1]))
            s.append(np.multiply(node_list[n].rp[num],100))
            
        ax.scatter(Node_x, Node_y, s, color = c)
        
        
        for i in range(len(trip_list)):
            trace = np.array([trip_list[i].x, trip_list[i].y])
            clr = plt.cm.tab20(trip_list[i].drone)
            # checking if drone is not over origin node
            if not(np.array_equal(trace[:,num], node_list[0].coord[:])):
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
    
def plot(trip_list, node_list, x_max, y_max, U_max):
    N = len(node_list)
    Node_x = []
    Node_y = []
    
    max_trips = 0
    for i in range(len(trip_list)):
        if trip_list[i].trip_n>max_trips:
            max_trips = trip_list[i].trip_n
    
    c = []
    for n in range(1,N):
        Node_x.append(node_list[n].coord[0])
        Node_y.append(node_list[n].coord[1])
        c.append(plt.cm.RdYlGn((255-node_list[n].urgency*(255/U_max)).astype(int)))
    
    for h in range(max_trips+1):
        fig = plt.figure(h)
        ax = fig.gca()
        ax.clear()
        # Setting Axes Limits
        ax.set_xlim(0, x_max)
        ax.set_ylim(0, y_max)
    
        # Adding Figure Labels
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        # Plotting the nodes
        ax.plot(node_list[0].coord[0], node_list[0].coord[1], 'ro')
        ax.scatter(Node_x, Node_y, s=None, color = c)
        for n in range(1,N):
            ax.annotate(str(n), (node_list[n].coord[0]+3, node_list[n].coord[1]))            

        for i in range(len(trip_list)):
            if trip_list[i].trip_n==h:
                clr = plt.cm.tab20(trip_list[i].drone)
                offset = 2+np.random.uniform(low=0, high=3)
                for k in range(0, len(trip_list[i].node_X)-1, 2):
                    dx = trip_list[i].node_X[k+1]-trip_list[i].node_X[k]
                    dy = trip_list[i].node_Y[k+1]-trip_list[i].node_Y[k]
                    l = np.sqrt(pow(dx, 2)+pow(dy, 2))
                    plt.arrow(trip_list[i].node_X[k]+offset*dy/l,
                              trip_list[i].node_Y[k]-offset*dx/l,
                              dx+offset*dy/l,
                              dy-offset*dx/l,
                              color=clr, head_width = 15, length_includes_head = True)
        
        name = 'trip_' + str(h) +'.pdf'
        plt.savefig(name, dpi=120, format='pdf', bbox_inches='tight')
        #plt.savefig(name, dpi=500)

def plotmap(node_list, x_max, y_max, U_max):
    N = len(node_list)
    Node_x = []
    Node_y = []
    
    s = []
    c = []
    for n in range(1,N):
        Node_x.append(node_list[n].coord[0])
        Node_y.append(node_list[n].coord[1])
        c.append(plt.cm.RdYlGn((255-node_list[n].urgency*(255/U_max)).astype(int)))
        s.append(np.multiply(node_list[n].rp[1],100))
    
    fig = plt.figure()
    ax = fig.gca()
    ax.clear()
    # Setting Axes Limits
    ax.set_xlim(0, x_max)
    ax.set_ylim(0, y_max)

    # Adding Figure Labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    # Plotting the nodes
    ax.plot(node_list[0].coord[0], node_list[0].coord[1], 'ro')
    ax.scatter(Node_x, Node_y, s, color = c)
    for n in range(1,N):
        ax.annotate(str(n), (node_list[n].coord[0]+3, node_list[n].coord[1]))            

    name = 'map.pdf'
    plt.savefig(name, dpi=120, format='pdf', bbox_inches='tight')
    #plt.savefig(name, dpi=500)