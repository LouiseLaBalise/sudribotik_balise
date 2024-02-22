#This file is based on this tutorial : https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
import cv2
import numpy as np
import glob
import os
import sys
import json


FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
MEDIA_FOLDER = f"{FILE_PATH.split(FILE_NAME)[0]}media/"
sys.path.insert(1, FILE_PATH.split("_3_TRAITEMENT_d_IMAGES")[0]) #add parent folder to python path
from init import prettify_json


def calibrateFisheyeDistortion(samples_affix = "chess", checkboard_tiles=(6,9)):
    """
    Calibrate the camera to get camera intrasic matrix K and vector of distortion coefficient D 
    to undistort image from a fisheye camera using chessboard pattern.

    Parameters:
        - samples_affix (str): term in the filename of images being sampled.
                               the function will only recognize them in the media/ folder.
        - checkboard_tiles (tuple): number of intersection between black tiles in height then width.

    Returns:
        - bool: function success.

    Note:
        Matrix K and D + height and width of the original image size will be stored in the init/configuration.json file.
        An outputed image is generated in test_24/ to see calibration results.
    """

    subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    objp = np.zeros((1, checkboard_tiles[0]*checkboard_tiles[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:checkboard_tiles[0], 0:checkboard_tiles[1]].T.reshape(-1, 2)
    _img_shape = None
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(f"{MEDIA_FOLDER}*{samples_affix}*.jpg")

    for fname in images:
        img = cv2.imread(fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "Toutes les images doivent être de la même taille"
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, checkboard_tiles, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)


    #Stop function if not enough images
    N_OK = len(objpoints)
    if (N_OK < 15):
        print(f"Log {FILE_NAME} : Il faut au minimum 15 photos de plateau d'échec prises de points de vus différents, "
              +f"de même dimension et comportant le terme '{samples_affix}' dans leur nom pour que la fonction se lance.")
        print(f"Il faut que l'image ait {checkboard_tiles[0]} intersections de cases noires sur la largeur et "
              +f"{checkboard_tiles[1]} intersections sur la longueur.")
        print(f"Seulement {N_OK} image(s) correspond(ent).")
        return False

    print(f"Log {FILE_NAME} : Il y a {N_OK} bonnes photos.")

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

    rms, _, _, _, _ = cv2.fisheye.calibrate(objpoints,
                                            imgpoints,
                                            gray.shape[::-1],
                                            K,
                                            D,
                                            rvecs,
                                            tvecs,
                                            calibration_flags,
                                            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
    

    #Insert matrix and size into the configuration file
    configuration_FILEPATH = FILE_PATH.split("_3_TRAITEMENT_d_IMAGES")[0]+"init/configuration.json"    
    #First Load the config file
    with open (configuration_FILEPATH, "r") as f:
        config = json.load(f)
    
    #Second save new keys
    config["AUTO_ORIGINAL_PHOTO_SIZE"] = list(_img_shape[::-1])
    config["AUTO_K_DISTORTION"] = K.tolist()
    config["AUTO_D_DISTORTION"] = D.tolist()

    #Third upload the new config file
    with open(configuration_FILEPATH, "w") as json_file:
        json.dump(config, json_file)

    #Prettify Json file
    prettify_json.prettify(configuration_FILEPATH)
    
    #Save undistortion of the first sampled image
    frame = cv2.imread(images[-1])
    res, undistorted_frame = undistortImage(frame, K, D, _img_shape[::-1])
    out_filename = "undistortion_result.jpg"
    cv2.imwrite(filename=MEDIA_FOLDER+out_filename, img=undistorted_frame)

    print(f"Log {FILE_NAME} : Résultat de la calibration disponnible dans {out_filename}."+
          f"\nL'image {images[0]} a été utilisé.")
    
    return True
    
    


def undistortImage(frame, K, D, size):
    """
    Undistort an image using a transform matrix.

    Parameters:
        - frame (np.array): inputed image.
        - K (np.array): camera intrasic matrix.
        - D (np.array): vector of distortion coefficients.
        - size (list): width and height of the outputed image.

    Returns:
        - bool: function success.
        - np.array: frame undistorted.

    Note:   The parameters <K>, <D> and <size> are already stored into the configuration file so you may wonder 
            why do we have to pass them by parameters instead of just access the JSON file directly in this function.
            It's mostly because this function will be used for each frame took by the beacon camera.
            In lieu of opening,closing,re-opening,re-closing the config file, opening it just one time on the caller side
            will save a lot of time and ressources. (same as redressImage())
    """

    #Get mapping points for remap() function
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_16SC2)
    
    #Apply a geometrical undistortion to the frame
    undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return True, undistorted_frame


if __name__=="__main__":

    #Just call the function
    calibrateFisheyeDistortion()