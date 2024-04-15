import numpy as np
import math as m

# noise varience.
noise_var = 0.1
dt = 0.1

# temporary setting. -> should be changed into some 
center = np.zeros((2,1))

#could append size of this matrix when our observaiton is added.
Measurement = np.zeros((2,1))

#should have prev, pres data in this matrix. Temp storage
# col0 : pres // col1 : prev
Esti_state = np.zeros((2,2))

class KF:
    def __init__(self, vec_size):
        self.pred_state = np.zeros((vec_size,1))
        self.pred_cov = np.zeros((vec_size,vec_size))
        self.KG = np.zeros((vec_size,vec_size))
        self.esti_state = np.zeros((vec_size,1))
        self.esti_cov = np.zeros((vec_size,vec_size))
        self.Theta = 0
        self.H = np.array([[1,0],[0,1]])
        
    #Kalman FIltering
    def TimeUpdate(self,observation,Q,R):
        F = np.array([[m.cos(self.Theta),-1*m.sin(self.Theta)]
                      [m.sin(self.Theta),m.cos(self.Theta)]])
        self.pred_state = np.dot(F,self.esti_state)
        self.pred_cov = np.dot(np.dot(F,self.esti_cov),np.transpose(F))+Q
        
        self.KG = np.dot(np.dot(self.pred_cov,np.transpose(self.H)),
                         np.linalg.inv(np.dot(np.dot(self.H,self.pred_cov),np.transpose(self.H))+R))
        self.esti_state = self.pred_state + np.dot(
            self.KG,(observation-np.dot(self.H,self.pred_state))
            )
        self.esti_cov = np.dot(
            (np.eye(2)-np.dot(self.KG,self.H)),self.pred_cov
        )
        return [self.esti_state, self.esti_cov]
    
def Rad_Ang(state_k, state_k_1):
    r_k = m.sqrt((state_k[0,0]-center[0,0])**2 + 
                 (state_k[1,0]-center[1,0])**2)
    r_k_1 = m.sqrt((state_k_1[0,0]-center[0,0])**2 + 
                 (state_k_1[1,0]-center[1,0])**2)
    l_k = m.sqrt((state_k[0,0]-state_k_1[0,0])**2 + 
                 (state_k[1,0]-state_k_1[1,0])**2)
    Theta = m.acos(
        (r_k**2+r_k_1**2-l_k**2)/(2*r_k*r_k_1)
    )
    Angvel = Theta/dt
    return [r_k, Angvel, Theta]


##Algorithm Starts...
#Q = # need to be get priori
R = noise_var*np.eye(2)
estimation = KF(2)
ALTITUDE = []
esti_COV = np.zeros((2,2))
iternum = 1
while True:
    Esti_state[:,1] = Esti_state[:,0]
    [Esti_state[:,0], esti_COV] = estimation.TimeUpdate(Measurement,Q,R)
    [alti, ANG_VEL, estimation.Theta] = Rad_Ang(Esti_state[:,0],Esti_state[:,1])
    ALTITUDE.append(alti)
    
    print("[Iteration : ",iternum,"]====================")
    print("[Altitude]: ",alti,
          "\n[Ang Velocity]: ",ANG_VEL)   
    
    # break condition is needed
    #if 
    
    iternum = iternum+1

    
    

