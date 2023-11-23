import sys
import cv2
import numpy as np
from takePhoto import takePhoto  # Import the function

# Constants
LOWER_LIMIT_RGB = np.array([0, 10, 0], dtype='uint8')
UPPER_LIMIT_RGB = np.array([0, 255, 0], dtype='uint8')

def colorDetection():

    # Take a photo
    image_path = takePhoto

    # Read the image
    src = cv2.imread(image_path)

    # Check if the image was successfully read
    if src is None:
        print("Image not detected.")
        sys.exit(0)

    # Create a mask to only show BGR limits selected
    mask = cv2.inRange(src, LOWER_LIMIT_RGB, UPPER_LIMIT_RGB)

    # Apply the mask to the source image
    output = cv2.bitwise_and(src, src, mask=mask)

    # Display the output image
    cv2.namedWindow("Output Color Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Output Color Detection", 600, 600)
    cv2.imshow("Output Color Detection", output)
    cv2.waitKey(0)

if __name__=="__main__":
    colorDetection()
