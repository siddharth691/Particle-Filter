
"""Name: Manisha Natarajan (903294595), Siddharth Agarwal (903223797)
   Lab1 STR"""

#Particle filter
import numpy as np
from random import *
from Subcodes.map import Map
from Subcodes.resampling import resampling
from Subcodes.sensor_model import Sensor_Model
from Subcodes.motion_model import sampling
import matplotlib.pyplot as plt
import time
from math import *

class Particle_filter:
    def __init__(self, no_of_particles = 1000):
        self.M = no_of_particles
        #Reading map\
        m = Map()
        self.map = m.readMap()
        return None
    
    def ret_map(self):
        return self.map
    
    def prior(self):
        """
        Returns randomly generated prior particles on the unocuppied portion of the map
		"""
        Xt = []
        
        def fit_func():
            x = randint(0,799)
            y = randint(0,799)
            theta = np.random.uniform(-pi,pi)
            if((self.map[y][x] > 0.9)):
                return([x,y,theta])
                
            
            else:
                return fit_func()
                        
        for i in range(self.M):
            Xt.append(fit_func())
                
        return Xt
    

def parseLine(f):
    """
    Parse function to parse the data line by line. Each time this function is called it will read the next line
    """
    temp = f.readline()
    c = temp.split()[0]
    t1 = temp.split()

    #To return data in float format - first is chtr O or L then X,Y,theta or X,Y,theta;Xl,Yl,thetal;and r1 to r180
    if(c == 'L'):
        return([c, [float(t1[1])/10.0, float(t1[2])/10.0, float(t1[3])], [float(t1[4])/10.0, float(t1[5])/10.0, float(t1[6])], list(map(float,t1[7:len(t1)-1])), float(t1[len(t1)-1])], temp)
    else:
        return([c, [float(t1[1])/10.0, float(t1[2])/10.0, float(t1[3])], float(t1[len(t1)-1])], temp)
    
def main():

    #Main function to run particle filter
    motion_model = sampling()
    meas_model = Sensor_Model()
    pf = Particle_filter()
    resample = resampling()


    #Plotting the probabilities
    try:
        meas_model.plot_prob()
        
    except NameError: 
        print("Plotted the probabilities")
    
    f, ax = plt.subplots(figsize=(50,50))


    #Prior reading
    pose0 = pf.prior()
    Xt_1 = pose0

    #Data reading
    f = open('./data/ascii-robotdata2.log','r') 
    meas_list, endLine =parseLine(f) #endline is for checking end of data
    Xt_1_b = meas_list[1]  

    #Display map background
    m_print = pf.ret_map()
    m_print[m_print==-1] = 0
    ax.axis([0, 800, 0, 800])
    
    #Inverting the map (map[y,:] = map[800-y,:]) because we are printing in graph
    inv_ind = list(-1*np.array(list(range(1,801))))
    m_print = m_print[inv_ind,:]

    #Printing the prior and the map
    implot = ax.imshow(m_print, cmap ='gray')
    plt.ion()
    prior_print = np.array(Xt_1)
    prior_print[:,1] = 799 - prior_print[:,1]

    p = ax.scatter(prior_print[:,0],prior_print[:,1], s = 5, c = 'r')

    ax.set_title("Current Time stamp %f"%(meas_list[-1]))
    plt.pause(0.001)
    p.remove()

    # #################################################################################################################################
    #Main while loop
    while(True):

        #Initial assignment
        meas_list, endLine = parseLine(f)
        Xt_b = meas_list[1]
        u_t = [Xt_1_b, Xt_b]
        Xt = []

        #Motion model for each particle to get a hypothesized pose in global frame
        Xt = motion_model.sample_motion_model_with_map(u_t,Xt_1,pf.ret_map())

        
        if(meas_list[0] == 'L'):
            #Calculating Weights
            w_t = meas_model.beam_range_finder_model(meas_list[3], np.array(meas_list[1]), np.array(meas_list[2]), np.array(Xt), pf.ret_map())

            Xt = resample.normal_resampling(Xt, w_t, pf.M)
        
        Xt_1 = Xt
        curr_state = np.array(Xt_1)
        

        #Reversing the y coordinate of the state for the purpose of plotting
        curr_state[:,1] = 799 - curr_state[:,1]

        p= ax.scatter(curr_state[:,0],curr_state[:,1], s= 5, c = 'r')
        
        #Plotting average direction using the arrow
        avg_angle = float(np.sum(curr_state[:,2])/float(len(curr_state[:,2])))
        avg_state_x = np.sum(curr_state[:,0])/float(len(curr_state[:,0]))
        avg_state_y = np.sum(curr_state[:,1])/float(len(curr_state[:,1]))
        u_arrow = np.cos(avg_angle)
        v_arrow = np.sin(avg_angle)        

        p1 = ax.quiver(avg_state_x,avg_state_y,u_arrow,v_arrow,color = 'cyan')

        ax.set_title("Current Time stamp %f, average heading %f"%(meas_list[-1],(180/pi)*avg_angle))
        plt.pause(0.001)

        p.remove()
        p1.remove()

  
        # else:

        #     print('converge else')
        #     curr_state[:,1] = 799 - curr_state[:,1]
        #     p= ax.scatter(curr_state[:,0],curr_state[:,1], s= 5, c= 'r')
        #     ax.set_title("Current Time stamp %f"%(meas_list[-1]))
        #     print("Converged or local optima")
        #     break



        if(endLine == ''):

            print('Observations got over')
            break


    while True:
        plt.pause(0.05)

if __name__ == "__main__":
	main()

    
