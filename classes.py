# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 12:39:54 2022

@authors: Riccardo Barbaglia, Pietro Deligios, Erin Lucassen
"""
import numpy as np

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