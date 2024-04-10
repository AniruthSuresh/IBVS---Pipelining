import cv2 as cv
from cv2 import aruco
import numpy as np


def servoing1(frame):
    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    param_markers = aruco.DetectorParameters_create()

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        ids, corners = marker_IDs[0], marker_corners[0]

        cv.polylines(
            frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
        )

        corners = corners.reshape(4, 2)
        corners = corners.astype(int)

        top_right = list(corners[0].ravel())
        top_left = list(corners[1].ravel())
        bottom_right = list(corners[2].ravel())
        bottom_left = list(corners[3].ravel())

        points = [top_left, top_right, bottom_left, bottom_right]
        points.sort()

        if points[0][1] > points[1][1]:
            points[0][1], points[1][1] = points[1][1], points[0][1]
        if points[2][1] > points[3][1]:
            points[2][1], points[3][1] = points[3][1], points[2][1]

        # centre = [int(round((top_left[0] + top_right[0] + bottom_left[0] + bottom_right[0]) / 4)),
        #           int(round((top_left[1] + top_right[1] + bottom_left[1] + bottom_right[1]) / 4))]
        
        # print(f'centre is {centre}')

        # cv.putText(
        #     frame,
        #     f"Center!!",
        #     tuple(centre),
        #     cv.FONT_HERSHEY_PLAIN,
        #     1.3,
        #     (200, 100, 0),
        #     2,
        #     cv.LINE_AA,
        # )

        # # Draw points
        # for point in points:
        #     cv.circle(frame, tuple(point), 5, (0, 255, 0), -1)  # Green circle at corners
        # cv.circle(frame, tuple(centre), 5, (255, 0, 0), -1)  # Blue circle at center

        # # Display the frame
        # cv.imshow("Frame", frame)
        # cv.waitKey(0)

                # Display the depth value
        # depth_value = depth_map[centre[1], centre[0]]
        # cv.putText(
        #     frame,
        #     f"Depth: {depth_value}",
        #     (10, 30),
        #     cv.FONT_HERSHEY_PLAIN,
        #     1,
        #     (0, 0, 255),
        #     2,
        #     cv.LINE_AA,
        # )

        # # Display the frame
        # cv.imshow("Frame", frame)
        # cv.waitKey(0)

        # print(f"depth is {depth_map[centre[1], centre[0]]} ")

        return points

# Example usage:
# points, depth = servoing(rgb_frame, depth_frame)
