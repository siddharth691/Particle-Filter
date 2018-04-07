"""Name: Manisha Natarajan (903294595), Siddharth Agarwal (903223797)
   Lab1 STR"""
import numpy as np
from numpy.linalg import inv
from math import *  
from random import *

class sampling:

    def __init__(self, alpha1 =0.25, alpha2 = 0.25, alpha3 = 0.25 , alpha4 = 0.25):
        
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4
        return None

    def prior(self, map1):
        """
        Return one prior particle for injecting
        """
        Xt = []
        def fit_func():
            x = randint(0,799)
            y = randint(0,799)
            theta = np.random.uniform(-pi,pi)
            if((map1[y][x] >0.8)):
                return([x,y,theta])
                
            
            else:
                
                return fit_func()
                        
        

        return fit_func()

    def sample_motion_model_odometry(self, ut,x_prev):
        
        """
        Input:
        
        ut is a list of x_bar_(t-1) and x_bar_t
        x_prev a list of x_prev
        
        Output:
        
        new state (list)
        """


        x_b = ut[0][0];
        y_b = ut[0][1];
        theta_b = ut[0][2];
        x_b_t = ut[1][0];
        y_b_t = ut[1][1];
        theta_b_t = ut[1][2];
    
        
        del_rot_1 = np.arctan2(y_b_t - y_b, x_b_t - x_b) - theta_b
        del_trans = np.sqrt(np.square(x_b - x_b_t) + np.square(y_b - y_b_t))
        del_rot_2 = theta_b_t - theta_b - del_rot_1
        
        del_rot_1_op = del_rot_1 - np.random.normal(0,np.sqrt(self.alpha1*abs(del_rot_1) + self.alpha2*abs(del_trans)))
        del_trans_op = del_trans - np.random.normal(0,np.sqrt(self.alpha3*abs(del_trans) + self.alpha4*abs(del_rot_1 + del_rot_2)) )
        del_rot_2_op = del_rot_2 - np.random.normal(0,np.sqrt(self.alpha1*abs(del_rot_2) + self.alpha2*abs(del_trans)))


        diff1 = del_rot_1 - del_rot_1_op
        diff2 = del_rot_2 - del_rot_2_op 

        
        # if(diff1 > pi):
        #     if((diff1 -  ((diff1)//2*pi)*2*pi) <=pi):
        #        diff1 =  (diff1 -  ((diff1)//2*pi)*2*pi)
            
        #     elif((diff1 - ((diff1)//2*pi)*2*pi) > pi):
        #         diff1 =  (diff1 -  (((diff1)//2*pi)+1)*2*pi)
        
        # if(diff2 > pi):
        #     if((diff2 -  ((diff2)//2*pi)*2*pi) <=pi):
        #        diff2 =  (diff2 -  ((diff2)//2*pi)*2*pi)
            
        #     elif((diff2 - ((diff2)//2*pi)*2*pi) > pi):
        #         diff2 =  (diff2 -  (((diff2)//2*pi)+1)*2*pi)
                
        
        del_rot_1_op = del_rot_1 - diff1
        del_rot_2_op = del_rot_2 - diff2
        
        x_t = x_prev[0] + del_trans_op*np.cos(x_prev[2] + del_rot_1_op)
        y_t = x_prev[1] + del_trans_op*np.sin(x_prev[2] + del_rot_1_op)
        theta_t = x_prev[2] + del_rot_1_op + del_rot_2_op

        return [x_t, y_t, theta_t]


    def sample_motion_model_with_map(self,ut, xt_1, m):
        """
        xt_1: list of particles


        """
        particle = 0
        x_t = []


        while(particle!=len(xt_1)):
            x = self.sample_motion_model_odometry(ut,xt_1[particle])

            if((int(floor(x[1])) <0 ) or (int(floor(x[1])) > 799) or (int(floor(x[0])) < 0) or (int(floor(x[0])) > 799)):

                x = self.prior(m)

            elif(x[2] > pi):
                if((x[2] - (x[2]//(2*pi))*2*pi) <= pi):
                    x = [x[0], x[1], (x[2] - (x[2]//(2*pi))*2*pi)]

                elif((x[2] - (x[2]//(2*pi))*2*pi) >pi):
                    x = [x[0], x[1], (x[2] - ((x[2]//(2*pi)) + 1)*2*pi)]



            elif((m[int(floor(x[1])),int(floor(x[0]))] ==0)):
                x = self.prior(m)


            x_t.append(x)


            particle+=1

        # print("No of particles injected: {}, {}, {}".format(count, count1, count2))
            

        #     else:

        #         while(True):


        #             x = self.prior(m)
        #             x1 = self.sample_motion_model_odometry(ut,x)

        #             # print("prior {}".format(x))
        #             # print("check {}".format(x1))
        #             if(m[int(floor(x1[1])),int(floor(x1[0]))] > 0 ):

        #                 x_t.append(x1)
                
        #                 particle+=1

        #                 break



            
        # print("length of xt {}".format(len(x_t)))
        return x_t
