import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np

import cv2 as cv
from cv2 import aruco

from detector import servoing
from robot_motion import robotControl
from save_image_2 import save_image
import time 

# from output_detector import servoing1


# Use p.DIRECT for non Graphical interface 

i = 25



def imageConversion(arr, h, w):
# a is alpha -> refers to the degree of transparency of a pixel
# A value of 0 indicates full transparency (completely see-through), while a value of 1 indicates full opacity (completely solid). 
      
    width, height = w, h
    global i
    pixels = []
    for x in range(height):
        for y in range(width):
            r, g, b, a = arr[x][y][0], arr[x][y][1], arr[x][y][2], arr[x][y][3]
            pixels.append((r, g, b, a))

    # //__  Create a new image from the pixel values  __//
    img = Image.new('RGBA', (width, height))
    img.putdata(pixels)
    # img.save(f"output_reference.png")
    i += 1
    return np.array(img)



def main():

    # boiler basic code 
    physicsClient = p.connect(p.DIRECT)
    dt = 0.0003 # don't increase it too much 
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10) # create a gravity 

    # setting up a ground plane 
    planeId = p.loadURDF("plane.urdf") # from pybullet data 


    # setting the robot up 

    # move the robot by 1 unit along z and shift its orientation by -45 in z axis 
    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0-(np.pi/4)]) # enter in radians 

    # loads the robot 
    boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId) # of the robot 
    # print(f"robot : POs , Ori = {cubePos,cubeOrn}")
 
    
    textureId = p.loadTexture("qrcode.png")
    orn = [0, 0, 0]
    
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    # increase the size of the obstacle by x20  

    # pos of obstcale = [0,5,1]   -> where the QR code is put 
    obstacleId = p.loadURDF("cube_small.urdf", [cubeStartPos[0], cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)

    # remaining obstacles to make the wall - around the qr code 
    obstacleId1 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]+1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]-1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]+1, cubeStartPos[1]+5, cubeStartPos[2]+5], cubeStartOrientation, globalScaling = 20)
    obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]-1, cubeStartPos[1]+5, cubeStartPos[2]+5], cubeStartOrientation, globalScaling = 20)
    obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0], cubeStartPos[1]+5, cubeStartPos[2]+5], cubeStartOrientation, globalScaling = 20)

    # cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId) # of the robot 
    # print(f"POs , Ori = {cubePos,cubeOrn}")
    

    # add that qrcode to the obstacle 
    p.changeVisualShape(objectUniqueId=obstacleId,
                     linkIndex=-1,  # full block put -1
                     textureUniqueId=textureId)
    useRealTimeSimulation = 1 # same pace as real time 


    base_link_id = p.getBodyInfo(boxId)[0]
    #print(base_link_id)
    if (useRealTimeSimulation):
        p.setRealTimeSimulation(1)

    sleep(1)
    i = 0

    

    while 1:
        i += 1
        if (useRealTimeSimulation):
        

            basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
            cubePos, cubeOrn = p.getBasePositionAndOrientation(obstacleId)

            # print(baseOrn)
            # convert to rotation matrox = 3x3 
            rot_matrix = p.getMatrixFromQuaternion(baseOrn) # shape of this = (9,1)
            # print(rot_matrix)

            # # Assuming rot_matrix is a list or array
            rot_matrix = np.array(rot_matrix)
            # print("Shape of the matrix:", rot_matrix.shape)

            # print(rot_matrix)
            # print(rot_matrix)
            rot_matrix = np.array(rot_matrix).reshape(3, 3) # reshaped to (3,3)
            # print(rot_matrix)


            # Initial vectors
            init_camera_vector = (0, 1, 0) # pointing along y axis 
            init_up_vector = (0, 0, 1) # up to camera = pos z 

            # camera is placed 0.7 units above the robot to capture better (vertically -> x and y are same )            
            basePos2 = [basePos[0], basePos[1], basePos[2]+0.7]
            # rotate the camera according to the robot 
            camera_vector = rot_matrix.dot(init_camera_vector) # where camera is looking 
            up_vector = rot_matrix.dot(init_up_vector)

            view_matrix = p.computeViewMatrix(basePos2, basePos + 2 * camera_vector, up_vector) # world to camera 


            p.stepSimulation()


            width, height = 800, 600
            fov, aspect, nearVal, farVal = 90, width/height, 0.01, 100
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearVal, farVal) # 3D to 2D 


            k = p.getCameraImage(width, height,view_matrix,projection_matrix)
            arr = np.array(k[2])
            # print(f"Cmera is {arr}")
            arr = imageConversion(k[2], height, width)
            # print(f"Cmera is {arr}")
            save_image(arr, "Images", f"image_{i}.png")

            # Get depth values using the OpenGL renderer
            # images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            depth_buffer_opengl = np.reshape(k[3], [width, height])
            # print(f"depth is {depth_buffer_opengl}")
            depth_opengl = farVal * nearVal / (farVal - (farVal - nearVal) * depth_buffer_opengl)
            # print(f"depth is {depth_opengl}")
            depth_opengl = np.array(depth_opengl)
            # print(depth_opengl.shape)

            points= servoing(arr)
            # points = servoing1(arr)

            # print(f"depth :{depth}")

            if not points:
                print("Done")
                _, baseOrn = p.getBasePositionAndOrientation(boxId)
                baseOrn = p.getEulerFromQuaternion(baseOrn)
                p.resetBasePositionAndOrientation(boxId, basePos, p.getQuaternionFromEuler([baseOrn[0], baseOrn[1], baseOrn[2]+np.pi/18]))
                continue
            else:
                velocity  = robotControl(points)
                
            basePos = list(basePos) # model 
            baseOrn = list(p.getEulerFromQuaternion(baseOrn))


            transform = np.zeros((4, 4))
            
            for kl in range(3):
                for mn in range(3):
                    transform[kl][mn] = rot_matrix[kl][mn]

            transform[0][3] = -basePos[0]
            transform[1][3] = -basePos[1]
            transform[2][3] = -basePos[2]
            transform[3][3] = 1

            # print(f"transform{transform}")
            velocity[3] = 1
            # print(f'cube is {cubePos,cubeOrn}')
            # print(f"POs , Ori = {cubePos,cubeOrn}")


            delPos = np.matmul(transform, velocity[:4])
            # print(f"velocity: {velocity}, delpos:{delPos}", sep='\n')
            basePos[0] += delPos[0]*dt
            basePos[1] += delPos[1]*dt
            basePos[2] += delPos[2]*dt
            baseOrn[2] += velocity[5]*dt*75
            baseOrn = p.getQuaternionFromEuler(baseOrn)
            p.resetBasePositionAndOrientation(boxId, basePos, baseOrn)
            sleep(.01)
            # print("X = {:+f}, Y = {:+f}, Z = {:+f}, Roll = {:+f}, Pitch = {:+f}, Yaw = {:+f}".format(*pos,*rpy))

            # # Display frame
            # cv.imshow('Frame', arr)
            # cv.waitKey(1)

if __name__ == "__main__":
    main()
