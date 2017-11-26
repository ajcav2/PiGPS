import numpy as np


if __name__ == "__main__":
    a_z = open("/home/pi/Documents/ae456final/data/z_accel_rotated").read().splitlines()
    dt = 1.0/163.0

    x0_hat = np.array([[0],[0]])
    P0 = np.array([[0.63, 0],[0,0.63]]) # need 2 calculate this
    F1 = np.array([[1, dt],[0, 1]])
    B1 = np.array([[dt**2/2],[dt]])
    H1 = np.array([[1,0],[0,0]])
    R1 = np.array([[0.63, 0],[0,0.63]])
    Q1 = np.array([[0.63, 0],[0,0.63]])

    p=[]
    v=[]
    
    z1 = 0
    for i in range(0,len(a_z)):
        # Prediction
        u1 = float(a_z[i])
        x1_hat = F1.dot(x0_hat) + B1.dot(u1)
        P1 = F1.dot(P0.dot(F1.T)) + Q1

        # Update
        m = np.linalg.inv(H1.dot(P1.dot(H1.T))+R1)
        K = P1.dot(H1.T.dot(m))
        P1_p = P1 - K.dot(H1.dot(P1))
        x1_hat_p = x1_hat + K.dot(z1-H1.dot(x1_hat))

        P0 = P1_p
        x0_hat = x1_hat_p
        
        p.append(x0_hat[0,0])
        v.append(x0_hat[1,0])

        
    print(p)

        
