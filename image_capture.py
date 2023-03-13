#import libraries 
#change
from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time

# CAMERA can be 0 or 1 based on default camera of your computer
camera = Picamera2()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode
camera.start() #must start the camera before taking any images
time.sleep(1)

while True:
    objectName = input("Enter object name: ")
    if objectName == 'q':
        break
    else:
        i = 1
        while True: 
            cmd = input("Press enter to capture an image: ")
            if cmd == 'q':
                break
            else:
                img_name = objectName + str(i) + '.jpg'
                camera.capture_file(img_name) #take image 
                i = i + 1
camera.stop() #stop the picam 
cv.destroyAllWindows()