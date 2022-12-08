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
from gurobipy import Model,GRB,LinExpr
import pickle
from copy import deepcopy

#################
### CONSTANTS ###
#################

N = 10  # number of nodes
x_max = 100 # width of the field
y_max = 100 # length of the field
rp_max = 10 # maximum amount of pesticide per node [l]
p_max = 5   # maximum amount of pesticide a drone can carry (tank_capacity) [l]
refill_time = 10    # time it takes for a drone to fill up its tank [s]
k_max = 3   # maximum number of drones
h_max = 3   # maximum number of trips
flight_time = 1200  # maximum drone flight time [s]
flight_speed = 8    # drone flight speed in [m/s]
drop_rate = 0.1     # rate of pesticide spraying in [l/s]

M = flight_time*3

#################
### VARIABLES ###
#################

x = {}
p = {}
arr = {}
dep = {}
a = {}

model = Model()
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
    model.addConstr(LHS >= rp[i], name=cnstr_name)

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
            model.addConstr(arr[i,k,h] + drop_rate*p[i,k,h] - dep[i,k,h] <= 0, name=cnstr_name)
            
# drone takes an amount of time to refill
for k in range(0,k_max):
    for h in range(1,h_max):
        cnstr_name = 'refill_time_drone'+str(k)+'_trip'+str(h)
        model.addConstr(arr[0,k,h-1] - dep[0,k,h] + refill_time -  M*(1-a[k,h]) <= 0, name=cnstr_name)


#####################
### COST FUNCTION ###
#####################
#stuff that needs to be done

