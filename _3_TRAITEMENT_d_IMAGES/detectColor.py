import sys
import cv2
import numpy as np
import takePhoto

def colorDetection(argv):

    #Get image
    src = cv2.imread(takePhoto.takePhoto())

    #Define RGB limits to see
    lower_limit_bgr = np.array([0, 0, 10], dtype='uint8')
    upper_limit_bgr = np.array([0, 0, 255], dtype='uint8')

    #Create mask to only show bgr limits selected
    mask = cv2.inRange(src, lower_limit_bgr, upper_limit_bgr)

    #Add mask uppon the source
    output = cv2.bitwise_and(src, src, mask=mask)

    #Display
    cv2.namedWindow("ouput color detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ouput color detection", 600, 600)
    cv2.imshow("ouput color detection", output)
    cv2.waitKey(0)

if __name__=="__main__":
    colorDetection(sys.argv[1:])
