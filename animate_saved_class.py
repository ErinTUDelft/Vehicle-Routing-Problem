import numpy as np
import classes
import pickle
import os
import animate_drones

x_max = 1000 # width of the field
y_max = 1000 # length of the field

seed = 4
nodes = 4
pesticide_max_node = 10
refill_time = 60
max_num_drone = 4

PATH = os.getcwd()
PATH += '\\Saved_solutions\\node_list'
PATH += '_seed' + str(seed) + '_nodes' + str(nodes) + '_maxpest' + str(pesticide_max_node) + '_reft' + str(refill_time) + '_maxd' + str(max_num_drone)
PATH += '.pkl'
with open(PATH, 'rb') as file:
    node_list = pickle.load(file)

PATH = os.getcwd()
PATH += '\\Saved_solutions\\trip_list'
PATH += '_seed' + str(seed) + '_nodes' + str(nodes) + '_maxpest' + str(pesticide_max_node) + '_reft' + str(refill_time) + '_maxd' + str(max_num_drone)
PATH += '.pkl'          
with open(PATH, 'rb') as file:
    trip_list = pickle.load(file)
    
steps = 300
stop_time = node_list[0].times[-1]+10
X_pos = np.array([])
Y_pos = np.array([])
for n in range(nodes):
    node_list[n].calc_amt(steps, stop_time)
    X_pos = np.append(X_pos, node_list[n].coord[0])
    Y_pos = np.append(Y_pos, node_list[n].coord[1])
                
for i in range(len(trip_list)):
    trip_list[i].calc_coord(X_pos, Y_pos, steps, stop_time)
    # trip_list[i].print_trip()

print('generating gif')
animate_drones.animate(trip_list, steps, node_list, x_max, y_max, 2)
print('generating plots')
animate_drones.plot(trip_list, node_list, x_max, y_max, 2)