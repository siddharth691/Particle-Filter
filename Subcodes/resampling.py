"""Name: Manisha Natarajan (903294595), Siddharth Agarwal (903223797)
   Lab1 STR"""
from random import *
import numpy as np
from itertools import repeat
from collections import Counter
class resampling:
    def __init__(self):
        return None
   
    def algorithm_low_variance_sampler(self,Xt, Wt):
        """
        Low variance resampling

		Input:
		------
		Xt (list of list): states from sampling algorithm [[x,y,theta (in degrees)], [x, y, theta],.... M such tuples]
		Wt (list): weights for each position

		Returns:
		--------
		X_b_t (list of list): (low variance) resampled positions [[x,y,theta (in degrees)], [x,y, theta], .... M such tuples]

		"""
        X_b_t = []
        M = len(Xt)
       
        if(M==0):
            raise ValueError('Be careful! Number of particles from the sampling process cannot be 0')
        
        r = uniform(0,1/M)
        c = Wt[0]
        print(c)
        i =0
        for m in range(0,M):
            u = r+(m)*(1/M)
            print(u)
            while(u>c):
                i+=1
                c+=Wt[i]
                X_b_t.append(Xt[i])
        return X_b_t


    def normal_resampling(self, Xt,Wt, no_of_particles):
        """
        Resampling using weight factorization and repeating based on the scores
        
        Input:
        ------
        Xt (list of list): states from sampling algorithm [[x,y,theta (in degrees)], [x, y, theta],.... M such tuples]
        Wt (list): weights for each position

        Returns:
        --------
        X_b_t (list of list): resampled positions [[x,y,theta (in degrees)], [x,y, theta], .... M such tuples]

        """
        M = no_of_particles
        
        #Check for convergence and normalize
        Wt = Wt/np.sum(Wt)
        
        
        Wi = []
        Wf = []
        score = np.zeros(len(Xt))
        cumul_f = 0
        cumul_i = 0
        for i in range(len(Wt)):
            Wi.append(cumul_i)
            cumul_i += Wt[i]

            cumul_f += Wt[i]
            Wf.append(cumul_f)

        for particle in range(0,M):
            r = uniform(0,1)
            for m in range(len(Wt)):
                if((r >= float(Wi[m])) & (r < float(Wf[m]))):
                    score[m]+=1
        new_Xt =  [x for index,item in enumerate(Xt) for x in repeat(item, int(score[index]))]
    

        
        return new_Xt
