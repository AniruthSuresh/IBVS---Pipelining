import pybullet as p
import cv2
import numpy as np
import pickle

# Function to create a virtual circular grid pattern in PyBullet
def create_circle_grid():
    # Load a circular grid model (you may need to create or download this URDF file)
    circle_grid_id = p.loadURDF("/home/aniruth/Desktop/RRC/VisualServoing/IBVS-pybullet/visual-servoing/IBVS/circle_grid.urdf", [0, 0, 0], useFixedBase=True)
    return circle_grid_id

# Function to render a synthetic image with the virtual camera in PyBullet
def render_image(camera_position, camera_target, width, height, fov):
    # Compute view matrix and projection matrix
    view_matrix = p.computeViewMatrix(camera_position, camera_target, [0, 1, 0])
    projection_matrix = p.computeProjectionMatrixFOV(fov, width / height, 0.1, 100)

    # Render image
    img = p.getCameraImage(width, height, view_matrix, projection_matrix)
    return img

# Main function
def main():
    # Connect to the PyBullet simulator
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)

    # Set up camera parameters
    width = 640
    height = 480
    fov = 60

    # Create a virtual circular grid pattern in the PyBullet environment
    circle_grid_id = create_circle_grid()

    # Define camera positions and targets
    camera_positions = [[2, 2, 2], [2, -2, 2], [-2, 2, 2], [-2, -2, 2]]  # Adjusted camera positions
    camera_targets = [[0, 0, 0]] * len(camera_positions)

    # Capture images and store them for camera calibration
    images = []
    for i, (camera_position, camera_target) in enumerate(zip(camera_positions, camera_targets)):
        # Render synthetic image
        img = render_image(camera_position, camera_target, width, height, fov)
        images.append(img[2])

        # # Display the captured image
        # cv2.imshow(f"Image {i+1}", img[2])
        # cv2.waitKey(0)

    # Perform camera calibration using OpenCV
    pattern_size = (8, 6)  # Number of circles in the circular grid pattern
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, circles = cv2.findCirclesGrid(gray, pattern_size, flags=cv2.CALIB_CB_SYMMETRIC_GRID)
        print(ret)
        print(circles)

        if ret:
            obj_points.append(objp)
            img_points.append(circles)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    calibration_data = {
        "camera_matrix": mtx,
        "distortion_coefficients": dist,
        "rotation_vectors": rvecs,
        "translation_vectors": tvecs,
    }

    with open("camera_calibration.pkl", "wb") as f:
        pickle.dump(calibration_data, f)

    print("Camera calibration results saved to 'camera_calibration.pkl'")
    # Print camera matrix and distortion coefficients
    print("Camera Matrix:")
    print(mtx)
    print("Distortion Coefficients:")
    print(dist)

    # Disconnect from the PyBullet simulator
    p.disconnect()

# Run the main function
if __name__ == "__main__":
    main()
