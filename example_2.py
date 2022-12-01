# -*- coding: utf-8 -*-
"""
Created on Thu Sep 24 22:22:35 2020

@author: abombelli
"""

# Loading packages that are used in the code
import numpy as np
import os
import pandas as pd
import time
from gurobipy import Model,GRB,LinExpr
import pickle
from copy import deepcopy

# Get path to current folder
cwd = os.getcwd()

# Get all instances
full_list           = os.listdir(cwd)

# instance name
instance_name = 'data_example_2.xlsx'

# Load data for this instance
edges                 = pd.read_excel(os.path.join(cwd,instance_name),sheet_name='Airport data')
#print("edges", edges)
startTimeSetUp = time.time()
model = Model()

#################
### VARIABLES ###
#################
x = {}

"""
Here the model variables are added. x is a dictionary wherein the tuple pair of the coordinates functions as the 
key for the gurobi.Var object (eg <gurobi.Var x[2,3]>). The variables are all added as binaries. 
60 are added for this problem in total. The distances are not yet added in this stage.(distances are added only at the stage
                                                                                       of the objective function, which makes sense)
"""
for i in range(0,len(edges)):
    x[edges['From'][i],edges['To'][i]]=model.addVar(lb=0, ub=1, vtype=GRB.BINARY,name="x[%s,%s]"%(edges['From'][i],edges['To'][i]))


"""
Nothing much is done here, you can even comment this one out without effect
"""
model.update()
#print("x is", x)


source = 25
sink   = 28

"""
Here the constraints are added, with 3 if statements (for the source, sink, and remaining nodes)
"""

for i in range(1,edges['From'][len(edges)-1]):
    """
    i is the amount of unique nodes in the dataframe (0 to 36 for this case)
    np.where checks here which flow flows out of this node, and which flow goes into the node.
    The outgoing flow is given a minus[-] such that the sum is equal to 0
    """
    idx_this_node_out = np.where(edges['From']==i)[0]
    idx_this_node_in  = np.where(edges['To']==i)[0]
    print("out", idx_this_node_out)
    print("in", idx_this_node_in)
    print("i", i)
    
    if i != source and i != sink:
        thisLHS = LinExpr()
        if len(idx_this_node_out) > 0:
            for j in range(0,len(idx_this_node_out)):
                thisLHS += x[i,edges['To'][idx_this_node_out[j]]]
        if len(idx_this_node_in) > 0:
            for j in range(0,len(idx_this_node_in)):
                thisLHS -= x[edges['From'][idx_this_node_in[j]],i]
        print("thisLHS", thisLHS)
        
        """
        So for each of the 34 non-source/sink nodes, the ingoing flow and outgoing flow is set to 0 using the 
        "model.addConstr" method. 
        """
        model.addConstr(lhs=thisLHS, sense=GRB.EQUAL, rhs=0,
                         name='node_'+str(i))
    
    
    
    if i is source:
        thisLHS = LinExpr()
        if len(idx_this_node_out) > 0:
            for j in range(0,len(idx_this_node_out)):
                thisLHS += x[i,edges['To'][idx_this_node_out[j]]]
                model.addConstr(lhs=thisLHS, sense=GRB.EQUAL, rhs=1,
                         name='node_'+str(i)+'_source_out')
        thisLHS = LinExpr()
        if len(idx_this_node_in) > 0:
            for j in range(0,len(idx_this_node_in)):
                thisLHS += x[edges['From'][idx_this_node_in[j]],i]
                model.addConstr(lhs=thisLHS, sense=GRB.EQUAL, rhs=0,
                         name='node_'+str(i)+'_source_in')
                
    if i is sink:
        thisLHS = LinExpr()
        if len(idx_this_node_in) > 0:
            for j in range(0,len(idx_this_node_in)):
                thisLHS += x[edges['From'][idx_this_node_in[j]],i]
                model.addConstr(lhs=thisLHS, sense=GRB.EQUAL, rhs=1,
                         name='node_'+str(i)+'_sink_in')
        thisLHS = LinExpr()
        if len(idx_this_node_out) > 0:
            for j in range(0,len(idx_this_node_out)):
                thisLHS += x[i,edges['To'][idx_this_node_out[j]]]
                model.addConstr(lhs=thisLHS, sense=GRB.EQUAL, rhs=0,
                         name='node_'+str(i)+'_sink_out')
        
"""
This one is also redundant
""" 
model.update()
 

"""
Ok so what you are doing here is setting the objective function. This is actually quite curious. The individual elements are first 
taken from the dataframe, and then "x[edges]" is done. This is only allowed with the gurobi package. Obj now is a "gurobipy.LinExpr" Object
gurobi.LinExpr: 100.0 x[1,2] + 300.0 x[2,3] <-- How part of the final obj looks like. 
Only here the distances are taken into account 
"""
       
obj        = LinExpr() 
for i in range(0,len(edges)):
    obj += edges['Distance'][i]*x[edges['From'][i],edges['To'][i]]


"""
The model.setObjective method takes two arguments:
    1) the desired objective function either linear (LinExpr) or quadratic
    2) the objective sense: GRB.Minimize or GRB.Maximize
Nothing is here actually calculated yet, this is done using the optimize method 
(only the objective function gets added)
"""
model.setObjective(obj,GRB.MINIMIZE)
model.update()
model.write('model_formulation.lp')    


"""
Here the model will be optimized. The method takes no arguments. Instead the model should have variables, 
constraints, and the objective function. Mixed integer problems are always solved using branch and cut, 
continious models are solved using the dual simplex method. 
Branch and bound is automatically used in this case because the (60) variables are all binary
"""
model.optimize()
endTime   = time.time()

solution = []


"""
After the optimizer magic, the final result is a guriby list of Var objects (sixty in total; one for each path possiblity). When extracted with the loop  below
you get for each individual path an assigned value of "1", or "0"
<gurobi.Var x[3,4] (value 0.0)>     <-- This path is not being used
<gurobi.Var x[11,23] (value 1.0)>   <-- This path is in use
"""     
for v in model.getVars():
     solution.append([v.varName,v.x])
print("Solution", solution)
     
route_complete = False
current_node   = source
path           = [source]

"""
Ok so this final loop is just a bit of a cumbersome way of extracting the final path from the calculated solution
"""
     
while route_complete is False:
    # Connections from current node
    idx_this_node_out = np.where(edges['From']==current_node)[0]
    #print(idx_this_node_out)
    for i in range(0,len(idx_this_node_out)):
        if x[current_node,edges['To'][idx_this_node_out[i]]].x >= 0.99:
            path.append(edges['To'][idx_this_node_out[i]])
            current_node = edges['To'][idx_this_node_out[i]]
            
            if current_node == sink:
                route_complete = True
                break
            else:
                break
            
print(path)
            
            
    
    
     


    
