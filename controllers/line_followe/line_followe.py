import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import time
import matplotlib as mpl
from controller import *
from funciones import *
robot = Robot()

# get the time step of the current world.
timestep = int(95)# 100 milisegundos equivale a 0.1 segundos

## Parte de declaracion de los motores del robot
wheels = []
wheelsNames = ['right wheel motor', 'left wheel motor']

for i in range(2):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
   
ground=[]
ground_names=['gs0','gs1','gs2']
for i in range(3):
    ground.append(DistanceSensor(ground_names[i]))
    ground[i].enable(timestep)

# Definir el tiempo de sampleo del sistema
t_sample=0.1
t_final=30+t_sample
t=np.arange(0,t_final,t_sample)
t=t.reshape(1,t.shape[0])

#Parametros del robot diferencial
r=0.0205
L=0.074
a=0.074/2

#velocidades generales
u=-0.0*np.ones((t.shape[0],t.shape[1]))
w=0*np.ones((t.shape[0],t.shape[1]))

#velocidades de cada rueda
w_r=0*np.ones((t.shape[0],t.shape[1]))
w_l=0*np.ones((t.shape[0],t.shape[1]))

for k in range(0,t.shape[1]):
    if robot.step(timestep) != -1:
        ## Vector de velocidades del sistema
        v=np.array([[u[0,k]],[w[0,k]]])
        
        ## conversion de velocidades generales a velocidades angulares de cada rueda
        w_r[0,k],w_l[0,k]=conversion(v,r,L)
        
        wheels[0].setVelocity(w_r[0,k])
        wheels[1].setVelocity(w_l[0,k])
        linea_real=conversion_sensor(ground[0].getValue(),ground[1].getValue(),ground[2].getValue())
        
        print(linea_real)    
    
wheels[0].setVelocity(0)
wheels[1].setVelocity(0)


print("FINALIZACION DEL PROGRAMA")
