import sys
import cv2
import numpy as np
import takePhoto

def colorDetection(argv):

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()

        #Get image
        #src = cv2.imread(takePhoto.takePhoto())
        #frame = cv2.imread(argv[0])
        cv2.imshow("rgb", frame)

        #Convert image from original colorspace bgr to hsv
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        cv2.imshow("hsv", hsv_frame)

        #Define RGB limits to see
        lower_limit_rgb = np.array([0, 0, 0], dtype='uint8')
        upper_limit_rgb = np.array([0, 255, 0], dtype='uint8')

        lower_limit_hsv, upper_limit_hsv = get_limits([0, 255, 0])
        #Convert bgr limits to hsv limits
        """lowsrc_rgb = np.uint8([[lower_limit_rgb]])
        lower_limit_hsv = cv2.cvtColor(lowsrc_rgb, cv2.COLOR_RGB2HSV)
        uppsrc_rgb = np.uint8([[upper_limit_rgb]])
        upper_limit_hsv = cv2.cvtColor(uppsrc_rgb, cv2.COLOR_RGB2HSV)"""

        #Create mask to only show bgr limits selected
        mask = cv2.inRange(frame, lower_limit_hsv, upper_limit_hsv)


        cv2.imshow("ouput color detection", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

        

    #Add mask uppon the source
    #output = cv2.bitwise_and(src, src, mask=mask)

    #Display
    """cv2.namedWindow("ouput color detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ouput color detection", 600, 600)
    cv2.imshow("ouput color detection", mask)
    cv2.waitKey(0)"""

def get_limits(color):
    c = np.uint8([[color]])  # RGB values
    hsvC = cv2.cvtColor(c, cv2.COLOR_RGB2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


if __name__=="__main__":
    colorDetection(sys.argv[1:])
