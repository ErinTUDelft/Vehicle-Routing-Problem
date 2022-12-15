# -*- coding: utf-8 -*-
"""
Created on Thu Dec  1 11:59:20 2022
@authors: Riccardo Barbaglia, Pietro Deligios, Erin Lucassen
"""

# Loading packages that are used in the code
import numpy as np
import os
import pandas as pd
import time
import matplotlib.pyplot as plt
from gurobipy import Model,GRB,LinExpr
import pickle
from copy import deepcopy
from scipy.spatial import distance_matrix
from matplotlib import animation

#################
### CONSTANTS ###
#################

N = 10  # number of nodes
x_max = 500 # width of the field
y_max = 500 # length of the field
rp_min = 1  # minimum amount of pesticide per node [l]
rp_max = 13 # maximum amount of pesticide per node [l]
p_max = 10   # maximum amount of pesticide a drone can carry (tank_capacity) [l]
refill_time = 30    # time it takes for a drone to fill up its tank [s]
k_max = 4   # maximum number of drones
flight_time = 1200  # maximum drone flight time [s]
flight_speed = 6    # drone flight speed in [m/s]
drop_rate = 0.1     # rate of pesticide spraying in [l/s]
U_min = 1 #minimum urgency weighting factor
U_max = 1000 #maximum urgency weighting factor


M = flight_time*3

np.random.seed(1)
X_pos = np.random.uniform(low=0, high=x_max, size=(N,))
Y_pos = np.random.uniform(low=0, high=y_max, size=(N,))
NODES = np.column_stack((X_pos,Y_pos))
RP = np.random.uniform(low=rp_min, high=rp_max, size=(N,))
RP[0] = 0
RP_TOT = np.sum(RP)
U = np.random.uniform(low=U_min, high=U_max, size=(N,))
U[0] = 0 #origin has no urgency
h_max = np.ceil((RP_TOT/(p_max*k_max))).astype(int)   # maximum number of trips

d_matrix = distance_matrix(NODES, NODES)
t = d_matrix/flight_speed





# fig, ax = plt.subplots()
# #ax.scatter(X_pos, Y_pos)
# ax.plot(X_pos[0],Y_pos[0], 'ro')
# ax.plot(X_pos[1:N],Y_pos[1:N], 'o')
# plt.xlim((0,x_max))
# plt.ylim((0,y_max))

# for i in range(0,N):
#     ax.annotate(str(i), (X_pos[i]+3, Y_pos[i]))



#################
### VARIABLES ###
#################

x = {}
p = {}
arr = {}
dep = {}
a = {}
Tcomp = {}

model = Model()

T=model.addVar(lb=0, vtype=GRB.CONTINUOUS,name="T") # Total time


for i in range(0,N):
    for j in range(0,N):
        for k in range(0,k_max):
            for h in range(0,h_max):
                x[i,j,k,h]=model.addVar(lb=0, ub=1, vtype=GRB.BINARY,name="x[%s,%s,%s,%s]"%(i,j,k,h)) # does vehicle k travel from i to j in trip h

for i in range(0,N):
    for k in range(0,k_max):
        for h in range(0,h_max):
            p[i,k,h]=model.addVar(lb=0, ub=p_max, vtype=GRB.CONTINUOUS,name="p[%s,%s,%s]"%(i,k,h)) # amount of pesticide drone k drops at node i in trip h
            arr[i,k,h]=model.addVar(lb=0, vtype=GRB.CONTINUOUS,name="arr[%s,%s,%s]"%(i,k,h)) # arrival time of drone k at node i during trip h
            dep[i,k,h]=model.addVar(lb=0, vtype=GRB.CONTINUOUS,name="dep[%s,%s,%s]"%(i,k,h)) # departure time of drone k from node i during trip h

for k in range(0,k_max):
    for h in range(0,h_max):
        a[k,h]=model.addVar(lb=0, ub=1, vtype=GRB.BINARY,name="a[%s,%s]"%(k,h)) # does drone k leave for trip h

for i in range (1,N):
    Tcomp[i] = model.addVar(lb=0, vtype=GRB.CONTINUOUS,name="Tcomp[%s]"%(i)) # Completion time of a node

model.update()



###################
### CONSTRAINTS ###
###################

# the demand of pesticide at each node needs to be satisfied
for i in range(1,N):
    LHS = LinExpr()
    for k in range(0,k_max):
        for h in range(0,h_max):
            LHS += p[i,k,h]
    cnstr_name = 'node_'+str(i)+'_satisfy_pesticide'
    model.addConstr(LHS >= RP[i], name=cnstr_name)
    
# you can only drop pesticide if the drone visits the node (checked if the drone departs from the node)
for i in range(0,N):
    for k in range(0,k_max):
        for h in range(0,h_max):
            LHS = LinExpr()
            for j in range(0,N):
                if i!=j:
                    LHS += -M*x[i,j,k,h]
            LHS += p[i,k,h]
            cnstr_name = 'drone_'+str(k)+'_can_drop_at_'+str(i)+'_in_trip_'+str(h)
            model.addConstr(LHS <= 0, name=cnstr_name)
    
# drone can carry a maximum amount of pesticide in one trip
for k in range(0,k_max):
    for h in range(0,h_max):
        LHS = LinExpr()
        for i in range(0,N):
            LHS += p[i,k,h]
        cnstr_name = 'drone_'+str(k)+'_can_carry_amount_of_pesticide_in_trip_'+str(h)
        model.addConstr(LHS <= p_max, name=cnstr_name)

# drone departs from 0
for k in range(0,k_max):
    cnstr_name = 'drone_'+str(k)+'_starts_at_origin'
    model.addConstr(dep[0,k,0] == 0, name=cnstr_name)

# every trip between two nodes takes a set amount of time (or more)
for i in range(0,N):
    for j in range(0,N):
        if i!=j:
            for k in range(0,k_max):
                for h in range(0,h_max):
                    cnstr_name = 'travel_time_from'+str(i)+'to'+str(j)+'_drone'+str(k)+'_trip'+str(h)
                    model.addConstr(dep[i,k,h]+t[i,j]-arr[j,k,h]-M*(1-x[i,j,k,h])<=0, name=cnstr_name)

# drone must spend enough time at each node to spray the amount of pesticide
for i in range(1,N):
    for k in range(0,k_max):
        for h in range(0,h_max):
            cnstr_name = 'drop_time_node'+str(i)+'_drone'+str(k)+'_trip'+str(h)
            model.addConstr(arr[i,k,h] + p[i,k,h]/drop_rate - dep[i,k,h] == 0, name=cnstr_name)
            
# drone takes an amount of time to refill
for k in range(0,k_max):
    for h in range(1,h_max):
        cnstr_name = 'refill_time_drone'+str(k)+'_trip'+str(h)
        model.addConstr(arr[0,k,h-1] - dep[0,k,h] + refill_time*a[k,h] <= 0, name=cnstr_name)

# for each drone k, for trip h to happen trip h-1 must also have happened
for k in range(0,k_max):
    for h in range(1,h_max):
        cnstr_name = 'for_drone_'+str(k)+'_for_trip_'+str(h)+'_to_happen_trip_'+str(h-1)+'_must_happen'
        model.addConstr(a[k,h-1] - a[k,h]>= 0, name=cnstr_name)

# the end of a trip is always after its beginning (even if it doesn't happen), otherwise the beginning of a non-happening trip could be at time 0
for k in range(0,k_max):
    for h in range(0,h_max):
        cnstr_name = 'drone_'+str(k)+'_must_arrive_at_origin_after_it_leaves_it_in_trip_'+str(h)
        model.addConstr(dep[0,k,h] - arr[0,k,h] <= 0, name=cnstr_name)
        
# if you go to a node you must leave the node
for k in range(0,k_max):
    for h in range(0,h_max):
        for i in range(0,N):
            LHS = LinExpr()
            for j in range(0,N):
                if i!=j:
                    LHS += x[i,j,k,h]-x[j,i,k,h]
            cnstr_name = 'in_trip_'+str(h)+'_drone_'+str(k)+'_must_leave_'+str(i)+'_if_it_travels_there'
            model.addConstr(LHS == 0, name=cnstr_name)
        
# if active during a trip vehicle must leave the depot once
for k in range(0,k_max):
    for h in range(0,h_max):
        LHS = LinExpr()
        for j in range(1,N):
            LHS += x[0,j,k,h]
        LHS -= a[k,h]
        cnstr_name = 'drone_'+str(k)+'_must_leave_depot_in_trip'+str(h)
        model.addConstr(LHS == 0, name=cnstr_name)

# if active during a trip vehicle must go to depot once
for k in range(0,k_max):
    for h in range(0,h_max):
        LHS = LinExpr()
        for i in range(1,N):
            LHS += x[i,0,k,h]
        LHS -= a[k,h]
        cnstr_name = 'drone_'+str(k)+'_must_return_to_depot_in_trip'+str(h)
        model.addConstr(LHS == 0, name=cnstr_name)
        
# total time variable
for k in range(0,k_max):
    model.addConstr(T - arr[0,k,h_max-1] >= 0, name='Total_time_greater_than_final_time_of_drone'+str(k))


#get the time at which a node i is satisfied
for i in range (1,N):
    for h in range(h_max):
        for k in range(k_max):
            model.addConstr(Tcomp[i] >= dep[i,k,h], name='Completion_time_node'+str(i))

model.update()
 

#####################
### COST FUNCTION ###
#####################
obj        = LinExpr() 
for k in range(0,k_max):
    # for h in range(0,h_max):
    #     obj += arr[0,k,h]-dep[0,k,h]
    obj += arr[0,k,h_max-1]

#minimize completion time of node [i], with its urgency U[i] as weighting factor
for i in range(1,N):
    obj += U[i] * Tcomp[i] 
    
obj += T

##################
### TEST START ###
##################

P1_min = 0   # minimum weighting factor for level-1 penalty
P1_max = 50 # maximum weighting factor for level-1 penalty
T1_min = 0 # minimum time limit for level-1 penalty
T1_max = 0 # maximum time limit for level-1 penalty

P2_min = 0   # minimum weighting factor for level-2 penalty
P2_max = 100 # maximum weighting factor for level-2 penalty
T2_min = 0 # minimum time limit for level-2 penalty
T2_max = 0 # maximum time limit for level-2 penalty


#randon generation of level-1 and 2 penalty factors for each node
P1 = np.random.uniform(low=P1_min, high=P1_max, size=(N,))
P1[0] = 0
P2 = np.random.uniform(low=P2_min, high=P2_max, size=(N,))
P2[0] = 0

#random generation of completion time limits for each node
T1 = np.random.uniform(low=T1_min, high=T1_max, size=(N,))
T2 = np.random.uniform(low=T2_min, high=T2_max, size=(N,))

################
### TEST END ###
################

model.setObjective(obj,GRB.MINIMIZE)
model.update()


###############
### SOLVING ###
###############
model.write('model_formulation.lp')  
model.Params.TimeLimit = 30
model.optimize()
endTime   = time.time()

solution = []

'''
# 
h=0
k=0
for i in range(0,N):
    for j in range(0,N):
        if x[i,j,k,h].x>0.9:
            print (x[i,j,k,h])
'''

k=0
D=0
for k in range(0,k_max):
    print ('\n########### DRONE '+str(k)+' ###########')
    h=0
    i=0
    loop = True
    while loop:
        if h == h_max or a[k,h].x < 0.1:
            break
        if i==0:
            print (f'Trip {h}:')
            
        for j in range(0,N):
            if x[i,j,k,h].x>0.9:
                
                print ('\tfrom {depN:2d}: {dept:6.1f}s'.format(dept = dep[i,k,h].x, depN = i))                
                print ('\t  to {arrN:2d}: {arrt:6.1f}s'.format(arrt = arr[j,k,h].x, arrN = j))                
                print ('\t\t\tDrops {amt:2.1f}/{tot:2.1f}\n'.format(amt = p[j,k,h].x, tot = RP[j]))
                D += p[j,k,h].x
                i=j
                if i==0:
                    h += 1
                    print (f'---- Dropped {D:2.1f}/{p_max:2.1f} ----\n')
                    D=0
                    break
                
print ('\n---------- NODE FULFILMENT ----------')            
for i in range(0,N):
    P = 0
    for k in range(0,k_max):
        for h in range(0,h_max):
            P += p[i,k,h].x
    if abs(P-RP[i])>0.05:
        print('!! demand not satisfied !!')
    print (f'Node {i}')
    print ('\tDropped {drp:2.1f}/{tot:2.1f}\n'.format(drp = P , tot = RP[i]))
    # if
    #     print('\tUrgency {urg:2.1f}\n'.format(urg = U[i]))
    #     print('\tCompletion time {comp:6.1f}s\n'.format(comp = Tcomp[i-1]))


'''
stuff Pietro is trying for animations
'''
class Trip:
    def __init__(self,drone,trip_n):
        self.drone = drone
        self.trip_n = trip_n
        self.leg_n = 0
        self.nodes = []
        self.node_t = []
        self.node_X = []
        self.node_Y = []
        self.x = []
        self.y = []
        self.t = []
    def __str__(self):
        return f'Drone {self.drone}, trip {self.trip_n}'
    def add_leg(self, i, j, dep, arr):
        self.nodes.append(i)
        self.nodes.append(j)
        self.node_t.append(dep)
        self.node_t.append(arr)
        self.leg_n += 1
    def print_leg(self, leg):
        depN = self.nodes[leg*2]
        depT = self.node_t[leg*2]
        arrN = self.nodes[leg*2+1]
        arrT = self.node_t[leg*2+1]
        print (f'\tfrom {depN:2d}: {depT:6.1f}s')                
        print (f'\t  to {arrN:2d}: {arrT:6.1f}s')
    def print_trip(self):
        print(self)
        for leg in range(self.leg_n):
            self.print_leg(leg)
            print()
    def calc_coord(self, nodesX, nodesY, steps, stoptime):
        self.node_X = nodesX[self.nodes]
        self.node_Y = nodesY[self.nodes]
        self.t = np.linspace(0,stoptime,steps)
        self.x = np.interp(self.t,self.node_t,self.node_X)
        self.y = np.interp(self.t,self.node_t,self.node_Y)
    
k=0
trip_list = []
steps = 300

for k in range(0,k_max):
    h=0
    i=0
    loop = True
    while loop:
        if h == h_max or a[k,h].x < 0.1:
            break
        if i==0:
            trip_list.append(Trip(k,h))
        for j in range(0,N):
            if x[i,j,k,h].x>0.9:
                trip_list[-1].add_leg(i, j, dep[i,k,h].x, arr[j,k,h].x)
                i=j
                if i==0:
                    trip_list[-1].calc_coord(X_pos, Y_pos, steps, T.x+5)
                    h += 1
                    break
# for i in range(len(trip_list)):
#     trip_list[i].print_trip()