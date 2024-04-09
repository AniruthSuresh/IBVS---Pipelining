# IBVS Pipeline Implementation

This repository contains an implementation of the IBVS (Image-Based Visual Servoing) pipeline using three different methods:

1. **Basic Method**: In this method, a fixed depth level is assumed, and the Jacobian matrix remains constant. The depth is fixed at `z = 1`.

2. **Depth Method**: Depth information is incorporated by using a depth map. The pipeline locates the center of the ArUco marker and considers the depth level at that point in the depth map as the actual depth.

3. **Updated Jacobian Method**: Instead of using the standard Jacobian matrix, this method employs the modified Jacobian, which is calculated as the average of the standard Jacobian and its transpose. This modification accounts for variations in the depth information.

In each of these cases, three coordinates from the ArUco marker are utilized as the degrees of freedom (DOF) for the robot motion. The objective is to control the robot's end-effector to minimize the error between the final position and the required position, bringing it within a tolerance of +/- 10 units.
