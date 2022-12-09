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

#################
### CONSTANTS ###
#################

N = 5  # number of nodes
x_max = 500 # width of the field
y_max = 500 # length of the field
rp_min = 1  # minimum amount of pesticide per node [l]
rp_max = 20 # maximum amount of pesticide per node [l]
p_max = 10   # maximum amount of pesticide a drone can carry (tank_capacity) [l]
refill_time = 30    # time it takes for a drone to fill up its tank [s]
k_max = 3   # maximum number of drones
flight_time = 1200  # maximum drone flight time [s]
flight_speed = 6    # drone flight speed in [m/s]
drop_rate = 0.1     # rate of pesticide spraying in [l/s]


M = flight_time*3

np.random.seed(1)
X_pos = np.random.uniform(low=0, high=x_max, size=(N,))
Y_pos = np.random.uniform(low=0, high=y_max, size=(N,))
NODES = np.column_stack((X_pos,Y_pos))
RP = np.random.uniform(low=rp_min, high=rp_max, size=(N,))
RP[0] = 0
RP_TOT = np.sum(RP)
h_max = np.ceil((RP_TOT/(p_max*k_max))).astype(int)   # maximum number of trips

d_matrix = distance_matrix(NODES, NODES)
t = d_matrix/flight_speed





fig, ax = plt.subplots()
#ax.scatter(X_pos, Y_pos)
ax.plot(X_pos[0],Y_pos[0], 'ro')
ax.plot(X_pos[1:N],Y_pos[1:N], 'o')
plt.xlim((0,x_max))
plt.ylim((0,y_max))

for i in range(0,N):
    ax.annotate(str(i), (X_pos[i]+3, Y_pos[i]))



#################
### VARIABLES ###
#################

x = {}
p = {}
arr = {}
dep = {}
a = {}

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
    model.addConstr(T - arr[0,k,h_max-1] >= 0, name='Total_time')

model.update()

#####################
### COST FUNCTION ###
#####################
obj        = LinExpr() 
for k in range(0,k_max):
    # for h in range(0,h_max):
    #     obj += arr[0,k,h]-dep[0,k,h]
    obj += arr[0,k,h_max-1]
    
obj += T
model.setObjective(obj,GRB.MINIMIZE)
model.update()


###############
### SOLVING ###
###############
model.write('model_formulation.lp')  
model.Params.TimeLimit = 60
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
