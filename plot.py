import matplotlib.pyplot as plt
import numpy as np

accel_z = open("/home/pi/Documents/ae456final/data/z").read().splitlines()
accel_y = open("/home/pi/Documents/ae456final/data/y").read().splitlines()
accel_x = open("/home/pi/Documents/ae456final/data/x").read().splitlines()

dt = 1.0/100.0
times = np.arange(0,dt*len(accel_z),dt)

d_z = []
d_z_total = 0.0

d_y = []
d_y_total = 0.0

d_x = []
d_x_total = 0.0
for i in range(0,len(accel_z)):
    d_z_total = d_z_total + -0.5*((float(accel_z[i])+0.905)*9.81)*dt**2
    d_z.append(d_z_total)

    d_y_total = d_y_total + 0.5*((float(accel_y[i]))*9.81)*dt**2
    d_y.append(d_y_total)

    d_x_total = d_x_total + 0.5*((float(accel_x[i])-0.2)*9.81)*dt**2
    d_x.append(d_x_total)

plt.figure()
plt.plot(times,d_z,'r.',label='z')
#plt.plot(times,d_y,'b.',label='y')
#plt.plot(times,d_x,'g.',label='x')
plt.legend()
plt.ylabel('Vertical position [m]')
plt.xlabel('Time [s]')
plt.show()

    
    
