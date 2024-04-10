# IBVS Pipeline Implementation

This repository contains an implementation of the IBVS (Image-Based Visual Servoing) pipeline using three different methods:

1. **Basic Method**: In this method, a fixed depth level is assumed, and the Jacobian matrix remains constant. The depth is fixed at `z = 1`.

2. **Depth Method**: Depth information is incorporated by using a depth map. The pipeline locates the center of the ArUco marker and considers the depth level at that point in the depth map as the actual depth.

3. **Updated Jacobian Method**: Instead of using the standard Jacobian matrix, this method employs the modified Jacobian, which is calculated as the average of the standard Jacobian (Jacobian for the output reference i.e the Required Position) and the Current Image . This modification accounts for variations in the depth information.

In each of these cases, three coordinates from the ArUco marker are utilized as the degrees of freedom (DOF) for the robot motion is 6. The objective is to control the robot's end-effector to minimize the error between the final position and the required position, bringing it within a tolerance of +/- 10 units.

## Additional Information

An additional aspect of this repository is the inclusion of a simulation without the robot arms or gripper visible in the video. This simulation is achieved by situating the camera above the robot and capturing the image from that perspective, resulting in the robot's arms being out of view.

In this particular case, depth information is considered, and the motion of the system is halted when all error values are positive and less than a specific threshold. This approach enhances the understanding of the impact of depth information on the IBVS pipeline's performance, particularly in scenarios where the physical appearance of the robot arms may not be directly observable.

Additionally, a simulation video demonstrating this scenario has been added to provide visual context and further insight into the behavior of the IBVS pipeline under these conditions.

## Usage

1. **Clone the Repository**: Clone this repository to your local machine.

2. **Navigate to Each Method Folder**: Navigate to the folder of the desired method implementation. For example, to run the Depth method implementation:

    ```bash
    cd Depth_IBVS/
    ```

3. **Create Images Directory**: Create a directory named "Images" if it does not exist already:

    ```bash
    mkdir Images
    ```

4. **Run the Pipeline**: Execute the Python script for the IBVS pipeline:

    ```bash
    python IBVS.py
    ```

5. **View Results**: After the pipeline execution, view the results stored in the `Images/` directory. Convert these images to Video to observe the robot's motion .


## Results 

Videos demonstrating the performance of all three methods have been uploaded , feel free to watch the videos to observe the behavior and effectiveness of each method and understanding the robot's motion in each of them 
