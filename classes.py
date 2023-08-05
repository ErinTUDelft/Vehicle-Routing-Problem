# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 12:39:54 2022

@authors: Riccardo Barbaglia, Pietro Deligios, Erin Lucassen
"""
import numpy as np
import bisect

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
    
    
    
    
class Node:
    def __init__(self, ID, coord, request, urgency):
        self.ID = ID
        self.coord = coord
        self.request = request
        self.urgency = urgency
        
        self.required_pesticide = []
        self.times = []
        self.drop_rate_variation = []
        self.drop_rate = []
        
        self.rp = []
        self.t = []
        
    def add_drop(self,p, arr, dep, max_drop_rate):
        dr = 0
        if arr != dep:
            dr = min(p/(dep-arr),max_drop_rate)
            
        
        i_1 = bisect.bisect(self.times,arr)
        self.times.insert(i_1, arr)
        self.drop_rate_variation.insert(i_1, dr)
        
        i_2 = bisect.bisect(self.times,dep)
        self.times.insert(i_2, dep)
        self.drop_rate_variation.insert(i_2, -dr)
        
    def calc_amt(self, steps, stoptime):
        self.required_pesticide = [self.request]
        self.drop_rate = [self.drop_rate_variation[0]]
        
        for i in range(1,len(self.times)):
            self.drop_rate.append(self.drop_rate[-1]+self.drop_rate_variation[i])
            self.required_pesticide.append(self.required_pesticide[-1] - self.drop_rate[i-1]*(self.times[i]-self.times[i-1]))
        self.t  = np.linspace(0,stoptime,steps)
        self.rp = np.interp(self.t,self.times,self.required_pesticide)
        
            