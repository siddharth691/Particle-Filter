"""Name: Manisha Natarajan (903294595), Siddharth Agarwal (903223797)
   Lab1 STR"""


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 23:02:08 2018

@author: manisha
"""
import numpy as np
from math import pi
from numpy.linalg import inv
import matplotlib.pyplot as plt
from random import *



class Sensor_Model:
    """
    Sensor Model
    Inputs:
    """
    def __init__(self, z_hit=0.8, z_rand=0.05, z_short=0.05, z_max = 0.1, lam_short=0.02, sig_hit=50,  max_z= 550):
    #Required Parameters
        #Weights for calculating probabilities
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.z_short = z_short
        self.z_max = z_max
        #Other intrinsic param
        self.lam_short = lam_short
        self.sig_hit = sig_hit
        self.max_z = max_z  #max range measured by laser 
        return None
    
    def plot_prob(self):
        fig = plt.figure(figsize= (10,10))
        prob = []
        z_true = 300
        for z_t in np.arange(0,self.max_z,0.01):

            p = self.z_hit*self.p_hit(z_true,z_t)+self.z_short*self.p_short(z_true,z_t)+self.z_max*self.p_max(z_t)+self.z_rand*self.p_rand(z_t)
            prob.append(p)

        plt.plot(np.arange(0,self.max_z,0.01),prob)
        plt.title("Range finder model for true distance: 300")
        plt.pause(2)
        plt.close('all')

    def p_hit(self, x1, x2):
        num = np.exp(-0.5*np.square((x1-x2)/self.sig_hit))
        p = num/np.sqrt(2*pi*np.square(self.sig_hit))
               
        tot =1
        if(x2<=self.max_z):
            return p*(1/tot)
        else:
            return 0
        
    def p_short(self, x1, x2):
        if(x2<=x1):
            p = self.lam_short*np.exp(-self.lam_short*x2)    
        else:
            p = 0
        return p
    
    def p_rand(self,x):
        if(x<=self.max_z):
            p = 1/self.max_z
        else:
            p = 0
        return p 
    
    def p_max(self,x):
        if(x == self.max_z):
            p =1
        else:
            p = 0 
        return p
    

    def beam_range_finder_model(self, z_t,x_t, x_l_t, x_t_m, m):
        """
        To calculate the probability of K measurements at x_t
        Inputs:
            z_t(numpy array) measurements @ a pos for every 10 degrees -> 36 measurements
            x_t
            x_l_t
            x_t(numpy array) pos obtained from motion model
            m(numpy array) occupancy grid map 800x800
            Output:
                Returns list of probabilities [p(z_t|x_t,m)] which is used as weights in particle filter
        """
        #Transforming laser coordinates to world frame
        # T_l2o = self.transition_matrix_2d(x_l_t)
        # T_r2o = self.transition_matrix_2d(x_t)


        M = len(x_t_m)
        wt = []
        for particle in range(M):
            q = 1
            tot_prob = []
            for theta_deg in range(0,180,8):
                
                # # Coordinates of laser in world frame
                # T_r2w = self.transition_matrix_2d(x_t_m[particle])
                # T_l2w = np.dot(T_r2w, np.dot(inv(T_r2o), T_l2o))
                # X_l_w = [T_l2w[0][2], T_l2w[1][2], np.arctan2(T_l2w[1][0],T_l2w[1][1])]

                theta_rad = (pi/180)*theta_deg
                z_true = self.Ray_Cast(x_t_m[particle][0],x_t_m[particle][1],x_t_m[particle][2], theta_rad, m)
                
                p = self.z_hit*self.p_hit(z_true,z_t[theta_deg])+self.z_short*self.p_short(z_true,z_t[theta_deg])+self.z_max*self.p_max(z_t[theta_deg])+self.z_rand*self.p_rand(z_t[theta_deg])
                tot_prob.append(p + 0.00000000001)

            q = np.exp(np.sum(np.log(tot_prob)))


            wt.append(q)

        return wt

    # def transition_matrix_2d(self,x):
    #     st = np.sin(x[2])
    #     ct = np.cos(x[2])
    #     return [[ct, -st, x[0]],[st, ct, x[1]],[0, 0, 1.]]

        
    # def calc_true_dist(self,x1,y1,theta, angle, m):
    
    #     """
    #     calculates the true obstacle distance from state of the robot using the map for the motion model
    #     Input:
    #     x1,y1,theta (in degrees) : (float) position of the robot
    #     angle (in degrees): (float) direction in which we need to look for
    #     m: (2-d np array of the map) occupancy grid map
    #     Returns:
    #     true_dist: (int) distance of the robot from the obstacle at the angle theta
    #     """
        
        
    #     theta = theta*pi/180 # in radians
    #     angle = angle*pi/180


    #     if((theta + angle) != pi/2):
    
    #         s = np.tan(theta + angle)
    
    
    #     slope_error = 0.001
    
    #     true_dist = 0
    #     x= x1;
    #     y= y1;
    
    #     while True:
    
    #         true_dist+=1;
            
    #         slope_error+=s
            
    #         if(slope_error>=0.5):
    #             y-=1
    #             slope_error-=1.0
            
    #         x+=1
    #         if(int(x)> m.shape[1] - 1 or int(x) < 0 or int(y) > m.shape[0] - 1 or int(y) < 0):
    #             break
                
    #         if(m[int(y),int(x)]== 0):
    #             break
    
    #     return true_dist


    def Ray_Cast(self,xc,yc,thetac,angle,map1):

        """
        To calculate the true distance of obstacle (within laser range) in map
        from a given co-ordinate and heading(xc, yc, thetac)
        Inputs:
            xc, yc, thetac -> coordinates
            angle -> direction to look at obstacle
            map1 -> map
            max_range -> range of sensor
        Output:
            true distance in pixel..
            if out of range or out of map ->max dist
            """
        xc = int(xc)
        yc = int(yc)
        max_range = self.max_z
        for d in range(0, max_range+1):

            cell_x = int(xc + d*np.cos(thetac + angle))
            cell_y = int(yc + d*np.sin(thetac + angle))
            # if cell is occupied
            if ((cell_x < 0)or(cell_y <0)or(cell_x >799)or(cell_y >799)):
                return(max_range)
            if (map1[cell_y, cell_x] >0.5):
                return(d)
            if ((d == max_range)and (map1[cell_y, cell_x]<=0.5)):
                return(d)

