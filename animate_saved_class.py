import numpy as np
import classes
import pickle
import os
import animate_drones

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

print('generating map')
animate_drones.plotmap(node_list, x_max, y_max, U_max)
print('generating plots')
animate_drones.plot(trip_list, node_list, x_max, y_max, U_max)
print('generating gif')
#animate_drones.animate(trip_list, steps, node_list, x_max, y_max, U_max)