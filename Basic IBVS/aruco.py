import cv2 as cv
from cv2 import aruco
import numpy as np


## CTRL + Z -> to terminate 
# this returns pixel coordinate 

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
param_markers = aruco.DetectorParameters_create()

# Read the marker image
image_path = "output_reference.png"
frame = cv.imread(image_path)

# Convert the image to grayscale
gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)


# print("wied")
# Detect markers in the image
marker_corners, marker_IDs, reject = aruco.detectMarkers(
    gray_frame, marker_dict, parameters=param_markers
)
# print("iwejf")
print(marker_corners)


# Draw markers and IDs on the image if markers are detected
if marker_corners:
    # print("iwjec")
    for ids, corners in zip(marker_IDs, marker_corners):
        cv.polylines(
            frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
        )

        # print("jewijco")
        corners = corners.reshape(4, 2)
        corners = corners.astype(int)
        top_left = corners[0].ravel()
        top_right= corners[1].ravel()
        bottom_right = corners[2].ravel()
        bottom_left = corners[3].ravel()
        cv.putText(
            frame,
            f"id: {ids[0]}",
            top_left,
            cv.FONT_HERSHEY_PLAIN,
            1.3,
            (200, 100, 0),
            2,
            cv.LINE_AA,
        )

    # Save the frame with polylines drawn to a new PNG file
    # output_file_path = "output_with_polylines.png"
    # cv.imwrite(output_file_path, frame)
        # Save the frame with polylines drawn to a new PNG file
    # output_file_path = "output_with_polylines.png"
    # if cv.imwrite(output_file_path, frame):
    #     print(f"Image with polylines saved to: {output_file_path}")
    # else:
        
    #     print("Error: Failed to save the image")
        
        # print(corners)
        


# Display the resulting image
cv.imshow("Detected Markers", frame)
cv.waitKey(0)
cv.destroyAllWindows()
