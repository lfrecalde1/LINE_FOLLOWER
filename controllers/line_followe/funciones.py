import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
def conversion(v,r,L):
    T=np.matrix([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]

def conversion_1(W,r,L):
    T=np.matrix([[r/2,r/2],[r/L,-r/L]])
    generales=T@W
    return generales[0,0],generales[1,0]

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
    auxiliar=1
    if(sensor1==1 and sensor2==0 and sensor3==0):
        valor=-1
        auxiliar=valor
        
    elif(sensor1==1 and sensor2==1 and sensor3==0):
        valor=-0.5
         
    elif(sensor1==0 and sensor2==1 and sensor3==0):
        valor=0
        
    elif(sensor1==0 and sensor2==1 and sensor3==1):
        valor=0.5   
         
    elif(sensor1==0 and sensor2==0 and sensor3==1):
        valor=1
        auxiliar=valor
        
    elif(sensor1==1 and sensor2==1 and sensor3==1):
        valor=2
        
    else:
        valor=auxiliar
        
    return valor
def tranformacion_cordenadas(x,y,z,phi,teta):
    T1=np.matrix([[np.cos(phi),0,np.sin(phi)],[0,1,0],[-np.sin(phi),0,np.cos(phi)]])
    T2=np.matrix([[np.cos(teta),-np.sin(teta),0],[np.sin(teta),np.cos(teta),0],[0,0,1]])
    relativo=np.array([[x],[y],[z]])
    real=T1@T2@relativo
    return real[0,0],real[1,0],real[2,0]

def grafica(sty,titulo,x,y,etiqueta,ejex,ejey,color):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color, label=etiqueta)
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()

def grafica_c(sty,titulo,x,y,etiqueta,ejex,ejey,color,x_1,y_1,etiqueta_1,color_1):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color,label=etiqueta)
    ax.plot(x_1,y_1,color_1,label=etiqueta_1)
    ax.plot()
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()

def obstaculos(distance,ground,err_1,err_2,w_1,kp,ki,kd,t_sample):
    linea_real=conversion_sensor(ground[0].getValue(),ground[1].getValue(),ground[2].getValue())
    pepito,err=pid(0,linea_real,err_1,err_2,w_1,kp,ki,kd,t_sample)
    if distance[1].getValue() > 100.0:
        w=1
        u=0.01
        
    elif distance[0].getValue() > 100.0:
        w=-1
        u=u=0.01
        
    elif distance[3].getValue() > 100.0:
        w=1
        u=u=0.01
       
    elif distance[2].getValue() > 100.0:
        w=-1
        u=u=0.01
        
    else:
        #w,err=pid(0,linea_real,err_1,err_2,w_1,kp,ki,kd,t_sample)
        w=0
        u=0.02
    return u,w,err

def Jacobiano(v,phi,a):
    J=np.matrix([[np.cos(phi),-a*np.sin(phi)],[np.sin(phi),a*np.cos(phi)]])
    hp=J*v
    return hp

def euler(z,zp,t_sample):
    z=z+zp*t_sample
    return z

def vel(ruedas,lastPostL,lastPostR,t_sample):
    newPostR = ruedas[0].getValue()
    velPosR = (newPostR - lastPostR)/t_sample
    lastPostR = newPostR

    newPostL = ruedas[1].getValue()
    velPosL = (newPostL - lastPostL)/t_sample
    lastPostL = newPostL
    return velPosR,velPosL,lastPostL,lastPostR