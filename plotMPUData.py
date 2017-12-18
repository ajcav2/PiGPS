import matplotlib.pyplot as plt
import numpy as np

# Read in data from files
path = "/home/pi/Documents/ae456final/data/"
accel_z = open(path+"raw/z_accel_MPU").read().splitlines()
accel_y = open(path+"raw/y_accel_MPU").read().splitlines()
accel_x = open(path+"raw/x_accel_MPU").read().splitlines()
rot_y = open(path+"raw/rot_y").read().splitlines()
rot_x = open(path+"raw/rot_x").read().splitlines()
gyro_x = open(path+"raw/gyro_x").read().splitlines()
gyro_y = open(path+"raw/gyro_y").read().splitlines()
gyro_z = open(path+"raw/gyro_z").read().splitlines()
accel_z_rot = open(path+"z_accel_rotated").read().splitlines()
accel_y_rot = open(path+"y_accel_rotated").read().splitlines()
accel_x_rot = open(path+"x_accel_rotated").read().splitlines()
v_x = open(path+"v_x").read().splitlines()
v_y = open(path+"v_y").read().splitlines()
v_z = open(path+"v_z").read().splitlines()
d_x = open(path+"dx").read().splitlines()
d_y = open(path+"dy").read().splitlines()
d_z = open(path+"dz").read().splitlines()
x_angle = open(path+"x_angle").read().splitlines()
y_angle = open(path+"y_angle").read().splitlines()
z_angle = open(path+"z_angle").read().splitlines()
p = open(path+"filtered_position").read().splitlines()
v = open(path+"filtered_velocity").read().splitlines()
GPS_alt = open(path+"GPS_interpolated").read().splitlines()

# Sampling frequency of MPU6050
dt = 1.0/163.0

# Number of data points
times = np.arange(0,dt*len(accel_z),dt)

# Plot gyro data
plt.figure()
plt.plot(times[0:len(gyro_x)],gyro_x,'r.',label='x')
plt.plot(times[0:len(gyro_y)],gyro_y,'g.',label='y')
plt.plot(times[0:len(gyro_z)],gyro_z,'b.',label='z')
plt.title('Gyro data')
plt.legend()
plt.ylabel('Rate [deg/sec]')
plt.xlabel('Time [s]')


# Plot rotation data
plt.figure()
plt.plot(times,rot_x,'r.',label='x rotation')
plt.plot(times,rot_y,'g.',label='y rotation')
plt.title('Rotation data')
plt.legend()
plt.ylabel('Angle [deg]')
plt.xlabel('Time [s]')

# Plot acceleration data
plt.figure()
plt.plot(times[0:len(accel_x_rot)],accel_x_rot,'r.',label='x')
plt.plot(times[0:len(accel_y_rot)],accel_y_rot,'g.',label='y')
plt.plot(times[0:len(accel_z_rot)],accel_z_rot,'b.',label='z')
plt.title('Acceleration data')
plt.legend()
plt.ylabel('Acceleration [m/s^2]')
plt.xlabel('Time [s]')

# Plot integrated gyro data
plt.figure()
plt.plot(times[0:len(x_angle)],x_angle,'r.',label='x')
plt.plot(times[0:len(y_angle)],y_angle,'g.',label='y')
plt.plot(times[0:len(z_angle)],z_angle,'b.',label='z')
plt.title('Integrated gyro')
plt.legend()
plt.ylabel('Angle [deg]')
plt.xlabel('Time [s]')

# Plot integrated velocity
plt.figure()
plt.plot(times[0:len(v_x)],v_x,'r.',label='x')
plt.plot(times[0:len(v_y)],v_y,'g.',label='y')
plt.plot(times[0:len(v_z)],v_z,'b.',label='z')
plt.title('Integrated velocity')
plt.legend()
plt.ylabel('Velocity [m/s]')
plt.xlabel('Time [s]')

# Plot position with Kalman Filter
GPS_alt = np.asarray(GPS_alt)
GPS_alt = GPS_alt.astype(np.float)
plt.figure()
plt.plot(times[0:len(p)],p,'r.',label='Kalman')
plt.plot(times[0:len(d_z)],d_z,'g.',label='MPU')
plt.plot(times[0:len(GPS_alt)],GPS_alt,'b.',label='GPS')
plt.title('Altitude')
plt.legend(bbox_to_anchor=(0.35,0.35),
           bbox_transform=plt.gcf().transFigure)
plt.ylabel('Altitude [m]')
plt.xlabel('Time [s]')
plt.grid(color='k',linestyle='-', linewidth=2)

# Plot velocity with Kalman Filter
plt.figure()
plt.plot(times[0:len(v)],v,'r.',label='Kalman')
plt.plot(times[0:len(v_z)],v_z,'g.',label="MPU")
plt.title('Velocity')
plt.legend()
plt.ylabel('Velocity [m/s]')
plt.xlabel('Time [s]')


# Plot position
plt.figure()
plt.plot(times[0:len(d_x)],d_x,'r.',label='x pos')
plt.plot(times[0:len(d_y)],d_y,'g.',label='y pos')
plt.plot(times[0:len(d_z)],d_z,'b.',label='z pos')
plt.title('Integrated position')
plt.legend()
plt.ylabel('Position [m]')
plt.xlabel('Time [s]')
plt.show()
