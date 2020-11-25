import numpy as np
def conversion(v,r,L):
    T=np.matrix([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]

def pid(sp,real,err_1,err_2,u_1,kp,ki,kd,ts):
    err=np.tanh(sp-real)
    proporcional=kp*(err-err_1)
    integral=ki*err*ts
    derivativo=kd*(err-2*err_1+err_2)/ts
    u=u_1+proporcional+integral+derivativo
    return u,err
def conversion_sensor(sensor1,sensor2,sensor3):
    if(sensor1<400):
        conversion0=1
    else:
        conversion0=0
        
    if(sensor2<400):
        conversion1=1
    else:
        conversion1=0
        
    if(sensor3<400):
        conversion2=1
    else:
        conversion2=0   
    dato=angulo_sensores(conversion0,conversion1,conversion2)
 
    return dato
    
def angulo_sensores(sensor1,sensor2,sensor3):
    if(sensor1==1 and sensor2==0 and sensor3==0):
        valor=-1
        
    elif(sensor1==1 and sensor2==1 and sensor3==0):
        valor=-0.5   
         
    elif(sensor1==0 and sensor2==1 and sensor3==0):
        valor=0
        
    elif(sensor1==0 and sensor2==1 and sensor3==1):
        valor=0.5   
         
    elif(sensor1==0 and sensor2==0 and sensor3==1):
        valor=1
        
    elif(sensor1==1 and sensor2==1 and sensor3==1):
        valor=2
        
    else:
        valor=3
        
    return valor