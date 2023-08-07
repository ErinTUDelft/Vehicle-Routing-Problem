import numpy as np
import classes
import pickle
import os
import matplotlib.pyplot as plt
from matplotlib import animation

def main():
    x_max = 1000 # width of the field
    y_max = 1000 # length of the field
    U_max = 2

    seed = 2
    nodes = 5
    pesticide_max_node = 12
    refill_time = 60
    max_num_drone = 3

    path = os.getcwd()+'\\Saved_solutions\\'
    filecode = ('_seed{0}_nodes{1}_maxpest{2}_reft{3}_maxd{4}.pkl'
                .format(seed,nodes,pesticide_max_node,refill_time,max_num_drone))

    with open(path+'node_list'+filecode, 'rb') as file:
        node_list = pickle.load(file)
    with open(path+'trip_list'+filecode, 'rb') as file:
        trip_list = pickle.load(file)
        
    steps = 200
    stop_time = node_list[0].times[-1]+10
    X_pos = np.array([])
    Y_pos = np.array([])
    U  = np.array([])
    rp = np.array([])
    for n in range(nodes):
        node_list[n].calc_amt(steps, stop_time)
        X_pos = np.append(X_pos, node_list[n].coord[0])
        Y_pos = np.append(Y_pos, node_list[n].coord[1])
        U = np.append(U, node_list[n].urgency)
        rp = np.append(rp, node_list[n].request)
                    
    for i in range(len(trip_list)):
        trip_list[i].calc_coord(X_pos, Y_pos, steps, stop_time)
        # trip_list[i].print_trip()

    path = os.getcwd()+'\\Saved_images\\'
    filecode = ('seed{0}_nodes{1}_maxpest{2}_reft{3}_maxd{4}'
                .format(seed,nodes,pesticide_max_node,refill_time,max_num_drone))
    
    print('generating map')
    plotmap(node_list, x_max, y_max, U_max, path, filecode)
    print('generating plots')
    plottrips(trip_list, node_list, x_max, y_max, U_max, path, filecode)
    print('generating gif')
    generategif(trip_list, steps, node_list, x_max, y_max, U_max, path, filecode)


def generategif (trip_list, steps, node_list, x_max, y_max, U_max, path, filecode):
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
        ax.set_aspect('equal', 'box')
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
                ax.annotate(f'D{trip_list[i].drone+1}-T{trip_list[i].trip_n+1}', (trace[0, num]+3, trace[1, num]-10))
                
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
    
    f = path+filecode+'.gif'
    writergif = animation.PillowWriter(fps = steps/10)
    writergif.setup(fig,f)
    anim.save(f, writer=writergif)
  
  
def plottrips(trip_list, node_list, x_max, y_max, U_max, path, filecode):
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
        ax.set_aspect('equal', 'box')
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

        for i in range(len(trip_list)-1,0-1,-1):
            if trip_list[i].trip_n==h:
                clr = plt.cm.tab20(trip_list[i].drone)
                #offset = 2+np.random.uniform(low=0, high=3)
                offset = 3+3*trip_list[i].drone
                for k in range(0, len(trip_list[i].node_X)-1, 2):
                    dx = trip_list[i].node_X[k+1]-trip_list[i].node_X[k]
                    dy = trip_list[i].node_Y[k+1]-trip_list[i].node_Y[k]
                    l = np.sqrt(pow(dx, 2)+pow(dy, 2))
                    plt.arrow(trip_list[i].node_X[k]+offset*dy/l,
                              trip_list[i].node_Y[k]-offset*dx/l,
                              dx,
                              dy,
                              color=clr, head_width = 15, length_includes_head = True)
        
        name = path+filecode+'_trip' + str(h) +'.pdf'
        plt.savefig(name, dpi=120, format='pdf', bbox_inches='tight')
        #plt.savefig(name, dpi=500)

def plotmap(node_list, x_max, y_max, U_max, path, filecode):
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
    ax.set_aspect('equal', 'box')
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

    name = path+filecode+'_map.pdf'
    plt.savefig(name, dpi=120, format='pdf', bbox_inches='tight')

main()