# -*- coding: utf-8 -*-
"""
Created on Thu Dec  1 12:14:45 2022

@author: Erin
"""

import pandas as pd
import random
import numpy as np
import matplotlib.pyplot as plt

num_nodes = 10 #[-]
y_max = 100 #[m]
x_max = 100 #[m]
pesticide_max = 5 #[kg]?
urgency_max = 100 #[min]?

coordinates = []
pesticide_needed =  []
urgency = []

for i in range(num_nodes):
    coordinates.append((random.randint(0,y_max), (random.randint(0,x_max))))
    pesticide_needed.append(random.uniform(0, pesticide_max))
    urgency.append(random.uniform(0,urgency_max))
    
    

d = {
     'Coordinates':  coordinates,
     'Pesticide needed': pesticide_needed ,
     'Urgency': urgency
     }
df_nodes = pd.DataFrame(data = d)
print(df_nodes)






pathstruct = {'From': #from node,
              'To': #To node
              'Distance': 
    }


df_paths = None # extract paths here

 
#print 10 nodes with a random req
def random_node(x_max, y_max, pesticide_max, num_nodes):
    nodes = {}
    for i in range(num_nodes):
        x = random.randrange(0,x_max)
        y = random.randrange(0,y_max)
        pesticide_requirement = random.uniform(0,pesticide_max)
        node = {"coordinates:": (x,y),
                "pesticide requirement" : pesticide_requirement,
                "Node ID" : i   
                }
        #print(node)
    return node
        
    
#print(random_node(x_max, y_max, pesticide_max, num_nodes)  )




    
