"""Name: Manisha Natarajan (903294595), Siddharth Agarwal (903223797)
   Lab1 STR"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 20:06:35 2018

@author: manisha
"""
import numpy as np
import numpy as np

class Map:
    
    """
    Given data for the map:
        -1  = don't know
       any value in [0;1] is a probability for occupancy:
                 1   = occupied with probability 1
                 0   = unoccupied with probability 1
                 0.5 = occupied with probability 0.5
    """
    def __init__(self, map_x = 800, map_y = 800, map_res =10, maxX =8000, maxY =8000, filename = './data/wean.dat'):#Constructor
        self.map = np.zeros((map_x,map_y)) #Create a matrix of dimension of the map
        #Given:
        self.map_res = map_res 
        self.maxX = maxX
        self.maxY = maxY
        self.autoshiftX = 0
        self.autoshiftY = 0
        self.filename = filename
        return None
    
    def readMap(self):

        n = []
        with open('./data/wean.dat', 'r') as f:
            for i in range(7):
                next(f)
            
            for j in range(800):
                line = f.readline()
                
                content = line.split()
                content = list(map(float, content))
                n.append(content)
                

            return(np.array(n))
