import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import time
import matplotlib as mpl
from controller import *
from funciones import *
robot = Robot()
robot.__init__
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

distance=[]
distance_names=['ps7','ps0','ps6','ps1']
for i in range(4):
    distance.append(DistanceSensor(distance_names[i]))
    distance[i].enable(timestep)

ruedas=[]
ruedas_names=['right wheel sensor','left wheel sensor']
for i in range(2):
    ruedas.append(PositionSensor(ruedas_names[i]))
    ruedas[i].enable(timestep)

# DEFINCION DEL SENSOR GPS
gps = GPS("gps")
gps.enable(timestep)

# DFINCION DE LA IMU
imu = InertialUnit("inertial unit")
imu.enable(timestep)
# Seccion para visualizar la camara
pantalla=Display("display")
pantalla.setColor(0xFFFFFF)
pantalla.drawLine(0, 50, 100, 50)
pantalla.drawLine(50, 0, 50, 100)
camera_id=[] 
camara=['camera']
for i in range(1):
    camera_id.append(robot.getCamera(camara[i]))
    camera_id[i].enable(timestep)


lidar_id=[]    
lidar=['lidar']
for i in range(1):
    lidar_id.append(robot.getLidar(lidar[i]))
    lidar_id[i].enablePointCloud()
    lidar_id[i].enable(timestep)
 

#display = robot.getDisplay('displayA')
# Definir el tiempo de sampleo del sistema
t_sample=0.1
t_final=20+t_sample
t=np.arange(0,t_final,t_sample)
t=t.reshape(1,t.shape[0])

#Parametros del robot diferencial
r=0.0205
L=0.074
a=0.074/2

#velocidades generales
u=0.0*np.ones((t.shape[0],t.shape[1]))
w=0*np.ones((t.shape[0],t.shape[1]))

u_r=0*np.ones((t.shape[0],t.shape[1]+1))
ww_r=0*np.ones((t.shape[0],t.shape[1]+1))

#velocidades de cada rueda
w_r=0*np.ones((t.shape[0],t.shape[1]))
w_l=0*np.ones((t.shape[0],t.shape[1]))

w_r_r=0*np.ones((t.shape[0],t.shape[1]+1))
w_l_r=0*np.ones((t.shape[0],t.shape[1]+1))

#Definicion de los errores del pid
err_1=0
err_2=0
w_1=0
# deficinion de los vectores vacios del ssistema
#Posiciones del robot
x=np.zeros((t.shape[0],t.shape[1]+1))
y=np.zeros((t.shape[0],t.shape[1]+1))
phi=np.zeros((t.shape[0],t.shape[1]+1))

x_r=np.zeros((t.shape[0],t.shape[1]+1))
y_r=np.zeros((t.shape[0],t.shape[1]+1))
phi_r=np.zeros((t.shape[0],t.shape[1]+1))

if robot.step(timestep) != -1:

    w_r_r[0,0]=wheels[0].getVelocity()
    w_l_r[0,0]=wheels[1].getVelocity()
    W=np.array([[w_r_r[0,0]],[w_l_r[0,0]]])
    u_r[0,0],ww_r[0,0]=conversion_1(W,r,L)

    #posiciones iniciales lectura del robot
    posicion = gps.getValues()
    #Tranformacion de las posiciones reales al sistema de referencia deseado
    x_real,y_real,z_real=tranformacion_cordenadas(posicion[0],posicion[1],posicion[2],-np.pi/2,-np.pi/2)
    
    phi[0,0]=imu.getRollPitchYaw()[2]
    phi_r[0,0]=imu.getRollPitchYaw()[2]

    x[0,0]=x_real+a*np.cos(phi[0,0])
    y[0,0]=y_real+a*np.sin(phi[0,0])

    x_r[0,0]=x_real+a*np.cos(phi_r[0,0])
    y_r[0,0]=y_real+a*np.sin(phi_r[0,0])

lastPostL=0
lastPostR=0

kp=0.5
ki=0
kd=0

for k in range(0,t.shape[1]):
    if robot.step(timestep) != -1:
        
        u[0,k],w[0,k],err=obstaculos(distance,ground,err_1,err_2,w_1,kp,ki,kd,t_sample)



        ## Vector de velocidades del sistema
        v=np.array([[u[0,k]],[w[0,k]]])
        
        ## conversion de velocidades generales a velocidades angulares de cada rueda
        w_r[0,k],w_l[0,k]=conversion(v,r,L)
        
        wheels[0].setVelocity(w_r[0,k])
        wheels[1].setVelocity(w_l[0,k])

       
        #Transformacion para los valores reales del gps
        w_r_r[0,k+1],w_l_r[0,k+1],lastPostL,lastPostR=vel(ruedas,lastPostL,lastPostR,t_sample,k)
        #w_r_r[0,k+1]=wheels[0].getVelocity()
        #w_l_r[0,k+1]=wheels[1].getVelocity()

        W=np.array([[w_r_r[0,k+1]],[w_l_r[0,k+1]]])

        u_r[0,k+1],ww_r[0,k+1]=conversion_1(W,r,L) 

        v_r=np.array([[u_r[0,k+1]],[ww_r[0,k+1]]])

        ## Velocidad del punto modelado
        hp=Jacobiano(v_r,phi[0,k],a)
    
        ## Obtener las velocidades en Cada eje
        xp=hp[0,0]
        yp=hp[1,0]

        x_r[0,k+1]=euler(x_r[0,k],xp,t_sample)
        y_r[0,k+1]=euler(y_r[0,k],yp,t_sample)
        phi_r[0,k+1]=euler(phi[0,k],ww_r[0,k+1],t_sample)
        
        pixel_u,pixel_v=tranformacion_pantalla(x_r[0,k+1]*100,y_r[0,k+1]*100,1)
        print(pixel_v,pixel_u)
        pantalla.drawPixel(int(pixel_v),int(pixel_u))
        posicion = gps.getValues()

        x_real,y_real,z_real=tranformacion_cordenadas(posicion[0],posicion[1],posicion[2],-np.pi/2,-np.pi/2)
        phi[0,k+1]=imu.getRollPitchYaw()[2]
        x[0,k+1]=x_real+a*np.cos(phi[0,k+1])
        y[0,k+1]=y_real+a*np.sin(phi[0,k+1])

        print(x[0,k+1],y[0,k+1],phi[0,k+1])
        # Actualizacion de los valores apra le pid
        err_2=err_1
        err_1=err
        w_1=w[0,k]
        
wheels[0].setVelocity(0)
wheels[1].setVelocity(0)
grafica_c('default','Trayectorias',x[0,:],y[0,:],'$h$','$x[m]$','$y[m]$','g',x_r[0,:],y_r[0,:],'$h_r$','r--')
grafica_c('default','Velocidad derecha',t[0,:],w_r[0,:],'$w_r$','$t[s]$','$rad/s$','g',t[0,:],w_r_r[0,0:t.shape[1]],'$w_{rr}$','r--')
grafica_c('default','Velocidad Izquierda',t[0,:],w_l[0,:],'$w_r$','$t[s]$','$rad/s$','g',t[0,:],w_l_r[0,0:t.shape[1]],'$w_{rr}$','r--')
grafica_c('default','Velocidad lineal',t[0,:],u[0,:],'$u$','$t[s]$','$m/s$','g',t[0,:],u_r[0,0:t.shape[1]],'$u_r$','r--')
grafica_c('default','Velocidad angular',t[0,:],w[0,:],'$w$','$t[s]$','$rad/s$','g',t[0,:],ww_r[0,0:t.shape[1]],'$w_r$','r--')
print("FINALIZACION DEL PROGRAMA")
