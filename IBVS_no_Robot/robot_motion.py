import numpy as np
import cv2 as cv
from output_det import servoing1

def getJacobian(u, v, f=1, z=1):
    J = [[-f/z, 0, u/z, u*v/z, -(f + u*u)/f, v], [0, -f/z, v/z, (f+v*v)/f, -u*v/f, -u]]
    return np.array(J)

fr = cv.imread("reference_image_2.png") # the final output we want  
requiredPos = servoing1(fr) # gets the reqd position 


def robotControl(points,depth ,  k=None): # points = coordinates of 4 corners
    global requiredPos 
    print(requiredPos, points)
    error = []
    for i in range(3): # we need 3 points -> DOF = 6 
        for j in range(2):  
            error.append(requiredPos[i][j] - points[i][j])
    print(f"error = {error}")

    J = np.zeros((0, 6)) # Initialize as empty matrix
    # J1 = np.zeros((0, 6)) # Initialize as empty matrix
    z = [depth/10, depth/10, depth/10]

    max_abs_error = max(abs(val) for val in error)

    if max_abs_error <= 15: # keep changing it 
        if all(val >= 0 for val in error):
            print("All error values are positive and less than +/- 15. Stopping execution.")
            exit()

    # if k is not None:
    #     for i in range(3):
    #         z[i] = k[points[i][1] * 800 + points[i][0]]

    J = np.vstack((J, getJacobian(*points[0], z[0])))
    J = np.vstack((J, getJacobian(*points[1], z[1])))
    J = np.vstack((J, getJacobian(*points[2], z[2])))

    # J1 = np.vstack((J1, getJacobian(*requiredPos[0], z[0])))
    # J1 = np.vstack((J1, getJacobian(*requiredPos[1], z[1])))
    # J1 = np.vstack((J1, getJacobian(*requiredPos[2], z[2])))

    J_fin = J 
    # # print(J_fin/)
    # J_fin *= 0.5 # Scale the combined Jacobian
    # print(J_fin)

    J_FIN = np.linalg.pinv(J_fin)
    velocity = np.matmul(J_FIN, error)
    
    return velocity
