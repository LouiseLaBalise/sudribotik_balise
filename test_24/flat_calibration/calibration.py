import numpy as np
import cv2
import glob

print("\n\n\nTHIS SCRIPT CALIBRATE THE FLAT CAMERA. DO NOT USE THIS FOR THE FISH EYE.\n\n\n")

#Criteria is used to determine when to stop an iterative algorithm,
#by count of iteration, by accuracy or by both (stop when one of the two is reached)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


"""Camera calibration
    -dirpath : location of calibrations images
    -prefix : generalisation of filebame
    -image_format : image format for the calibration set
    -square_size : edge square size in meter
    -width : number of intersection in the long size of the calibration board
    -heght : same as width except its for the short size
    """
def calibrate(dirpath="media/calibration/",
              prefix="photo_calibration",
              image_format="jpg",
              square_size=0.015, width=9, height=6):

    print("Start calibration.")

    #Set object point in the form of (0,0,0), (0,1,0), (0,2,0)..
    #for each intersection on the board. z-pos is for now 0 because img is supposed to be flat
    objp = np.zeros((height*width, 3), np.float32)
    #Set all coordinates of our intersection into objp
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    #Set real coordinates including unit length of a square
    objp = objp*square_size

    #Arrays to store 2d (plane image) and 3d (real world) points
    objpoints_3d = []
    imgpoints_2d = []

    #List all path containing a calibration photo
    images = glob.glob(dirpath+prefix+'*.'+image_format)

    #Loop through all images
    for fname in images :

        print(f"Using {fname}...")
        img = cv2.imread(fname) #load an image
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert the image from bgr to gray

        #Find corners
        ret, corners = cv2.findChessboardCorners(img_gray, (width, height), None)

        #If all corners are found
        if ret:
            #Add obj point
            objpoints_3d.append(objp)

            #Once we have approximate location of corners we can strenghten the pos estimation
            #using this function and the criteria to stop the iteration
            corners2 = cv2.cornerSubPix(img_gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints_2d.append(corners2)

            #Draw and display the corners so we drop bad images and get new ones
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

    #Calibration
    print("Start calibration.")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints_3d, imgpoints_2d, img_gray.shape[::-1], None, None)

    print("Calibration done.")
    return [ret, mtx, dist, rvecs, tvecs]



"""Save matrix and distortion coefficients in a given path"""
def saveCoefficients(mtx, dist, path):

    #Mode write
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)

    #Write values
    cv_file.write("K", mtx)
    cv_file.write("D", dist)

    #Release (=close) the FileStorage
    cv_file.release()

    print(f"Results save in {path}")


if __name__ == "__main__":
    ret, mtx, dist, rvecs, tvecs = calibrate()
    saveCoefficients(mtx, dist, "./calibration_results.txt")






