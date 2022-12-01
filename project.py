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

N = 10  # number of nodes
x_max = 100 # width of the field
y_max = 100 # length of the field
rp_max = 10  # maximum amount of pesticide per node
q_max = 5   # maximum amount of pesticide a drone can carry
k_max = 3   # maximum amount of drones


#################
### VARIABLES ###
#################

for i in range(0,N):
  for j in range(0,N):
    for k in range(0,k_max):
      x[i,j,k]=model.addVar(lb=0, ub=1, vtype=GRB.BINARY,name="x[%s,%s,%s]"%(i,j,k)) # does vehicle k travel from i to j
model.update()

for i in range(0,N):
  for k in range(0,k_max):
    q[i,k]=model.addVar(lb=0, ub=q_max, vtype=GRB.CONTINUOUS,name="q[%s,%s]"%(i,k)) # amount of pesticide drone k drops at node i
    t[i,k]=model.addVar(lb=0, ub=inf, vtype=GRB.CONTINUOUS,name="t[%s,%s]"%(i,k)) # time at which drone k gets at node i
model.update()
