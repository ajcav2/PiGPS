import numpy as np
import interpolation
# Learn about Kalman filters here: https://drive.google.com/file/d/0By_SW19c1BfhSVFzNHc0SjduNzg/view

def write(lst,fname):
    with open("/home/pi/Documents/ae456final/data/"+fname,"w") as f:
        for val in lst:
            f.write("%s\n" % val)
        f.close()

if __name__ == "__main__":
    # Read acceleration data from sensor
    a_z = open("/home/pi/Documents/ae456final/data/z_accel_rotated").read().splitlines()
    GPS_alt_corrected = open("/home/pi/Documents/ae456final/data/GPS_alt_corrected").read().splitlines()

    # Frequency of MPU 6050
    dt = 1.0/172.0

    # Initialize variables
    P_cov = 1.0 # Process
    R_cov = 16.0 # GPS
    Q_cov = 0.0001 # State noise

    x0_hat = np.array([[float(GPS_alt_corrected[0])],[0.0]]) 
    P0 = np.array([[P_cov, 0],[0,P_cov]])
    F1 = np.array([[1.0, dt],[0.0, 1.0]])
    B1 = np.array([[dt**2/2],[dt]])
    H1 = np.array([[1.0,0.0],[0.0,0.0]])
    R1 = np.array([[R_cov, 0.0],[0.0,R_cov]])
    Q1 = np.array([[Q_cov, 0.0],[0.0,Q_cov]])
    
    # To record position and velocity
    p=[]
    v=[]

    # Interpolate acceleration and GPS measurement lists to be the same length
    a_z,z1,length = interpolation.interpolation(a_z,GPS_alt_corrected,len(a_z)*dt)

    # Correct sign for z acceleration (IMU is mounted upside down)
    for i in range(len(a_z)):
        a_z[i] = float(a_z[i])*(-1.0)

    # Apply Kalman filter
    for i in range(0,len(a_z)):
        # Prediction
        u1 = float(a_z[i])
        x1_hat = F1.dot(x0_hat) + B1.dot(u1)
        P1 = F1.dot(P0.dot(F1.T)) + Q1

        # Update
        m = np.linalg.inv(H1.dot(P1.dot(H1.T))+R1)
        K = P1.dot(H1.T.dot(m))
        P1_p = P1 - K.dot(H1.dot(P1))
        x1_hat_p = x1_hat + K.dot(np.array([[z1[i]],[0.0]])-H1.dot(x1_hat))
        

        # Iterate
        P0 = P1_p
        x0_hat = x1_hat_p

        # Record
        p.append(x0_hat[0,0])
        v.append(x0_hat[1,0])

    # Write data to file
    write(p,"filtered_position")
    write(v,"filtered_velocity")
        
