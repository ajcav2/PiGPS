import numpy as np

# Read in data from files
path = "/home/pi/Documents/ae456final/data/raw/"
accel_z = open(path+"z_accel_MPU").read().splitlines()
accel_y = open(path+"/y_accel_MPU").read().splitlines()
accel_x = open(path+"x_accel_MPU").read().splitlines()
rot_y = open(path+"rot_y").read().splitlines()
rot_x = open(path+"rot_x").read().splitlines()
gyro_x = open(path+"gyro_x").read().splitlines()
gyro_y = open(path+"gyro_y").read().splitlines()
gyro_z = open(path+"gyro_z").read().splitlines()

# Correction Lerp
def correctGyro(x_corr,y_corr,z_corr):
    global gyro_x
    global gyro_y
    global gyro_z
    
    for i in range(0,len(gyro_x)):
        gyro_x[i] = float(gyro_x[i])+x_corr
    for i in range(0,len(gyro_y)):
        gyro_y[i] = float(gyro_y[i])+y_corr
    for i in range(0,len(gyro_z)):
        gyro_z[i] = float(gyro_z[i])+z_corr

def write(lst,fname):
    with open("/home/pi/Documents/ae456final/data/"+fname,"w") as f:
        for val in lst:
            f.write("%s\n" % val)
        f.close()

        

# Generation rotated coordinates to calculate acceleration
def getRotatedCoordinates(theta_x,theta_y,theta_z):
    theta_x = (np.pi/180.0)*theta_x
    theta_y = (np.pi/180.0)*theta_y
    theta_z = (np.pi/180.0)*theta_z
    
    Rz = np.array([[np.cos(theta_z), np.sin(theta_z), 0],[-np.sin(theta_z), np.cos(theta_z), 0],[0,0,1]])
    Ry = np.array([[np.cos(theta_y), 0, -np.sin(theta_y)],[0,1,0],[np.sin(theta_y), 0, np.cos(theta_y)]])
    Rx = np.array([[1,0,0],[0, np.cos(theta_x), np.sin(theta_x)],[0, -np.sin(theta_x), np.cos(theta_x)]])
    return Rx,Ry,Rz

correctGyro(3.5,1.5,2.0)

# Sampling frequency of MPU6050
dt = 1.0/163.0

# Number of data points
times = np.arange(0,dt*len(accel_z),dt)

# Initialize position values
d_z = []
d_z_total = 0.0
d_y = []
d_y_total = 0.0
d_x = []
d_x_total = 0.0

# Initialize angular values
x_angle = []
x_angle_total = 0.0
y_angle = []
y_angle_total = 0.0
z_angle = []
z_angle_total = 0.0

# Initialize velocities
v_z_total = 0.0
v_z = []
v_y_total = 0.0
v_y = []
v_x_total = 0.0
v_x = []

acceleration = np.full((3,len(accel_z)),0)
for i in range(0,len(accel_z)):
    # Calculate current angle by integrating gyro
    x_angle_total = x_angle_total + dt*(float(gyro_x[i])+(8.0/60.0+13.0/56.0)+(1.4/51.9))
    x_angle.append(x_angle_total)

    y_angle_total = y_angle_total + dt*(float(gyro_y[i])+(1.0/6.0)-(1.2/51.2))
    y_angle.append(y_angle_total)

    z_angle_total = z_angle_total + dt*(float(gyro_z[i]))
    z_angle.append(z_angle_total)

    # Rotate coordinated before applying acceleration integration
    Rx,Ry,Rz = getRotatedCoordinates(x_angle_total, y_angle_total, z_angle_total)
    
    inertialAcceleration = np.array([float(accel_x[i])-0.09, float(accel_y[i]), float(accel_z[i])+0.948]).T
    acceleration[:,i] = Rx.dot(Ry.dot(Rz.dot(inertialAcceleration)))

    # Calculate velocity
    v_z_total = v_z_total - (acceleration[2,i])*dt
    v_z.append(v_z_total)

    v_y_total = v_y_total + (acceleration[1,i])*dt
    v_y.append(v_y_total)

    v_x_total = v_x_total + (acceleration[0,i])*dt
    v_x.append(v_x_total)

    # Get position data from velocity                        
    d_z_total = d_z_total +  v_z_total*dt
    d_z.append(d_z_total)

    d_y_total = d_y_total +  v_y_total*dt
    d_y.append(d_y_total)

    d_x_total = d_x_total +  v_x_total*dt
    d_x.append(d_x_total)

write(v_z,"v_z")
write(acceleration[2,:],"z_accel_rotated")
write(v_y,"v_y")
write(acceleration[1,:],"y_accel_rotated")
write(v_x,"v_x")
write(acceleration[0,:],"x_accel_rotated")
write(x_angle,"x_angle")
write(y_angle,"y_angle")
write(z_angle,"z_angle")
write(d_x,"dx")
write(d_y,"dy")
write(d_z,"dz")

