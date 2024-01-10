import cv2
import numpy as np

def detect_aruco(image_path):
    # Load the image
    img = cv2.imread(image_path)

    # ArUco dictionary and parameters (replace with the values from the previous group)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    aruco_parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers
    corners, ids, rejected_corners = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_parameters)

    # Example: Extract x, y, theta from the detected ArUco markers
    if corners and ids:
        # Calculate x, y, theta based on the pose estimation
        x, y, theta = calculate_robot_coordinates(corners, ids)

        return x, y, theta, ids[0]  # Assuming a single ArUco marker, change accordingly

def calculate_robot_coordinates(corners, ids):
    matrix_coeff = np.array([[2.5959453540558084e+03, 0., 1.7015437024875371e+03],
                            [0., 2.5981150581304091e+03, 1.2459640171617973e+03],
                            [0., 0., 1.]])

    distortion_coeff = np.array([2.0203034110644411e-01, -3.8937675520950621e-01,
                                 2.8240913779598222e-03, 5.4936365788275619e-03,
                                 -9.1462806356767012e-02])

    # Estimate the pose of the first detected marker
    rvec, tvec, marker_points = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.02,
                                                                    matrix_coeff,
                                                                    distortion_coeff)

    # Extract x, y, theta from the pose information
    x, y, z = tvec[0][0]
    theta = np.degrees(np.arctan2(rvec[0][0][1], rvec[0][0][0]))

    return x, y, theta
