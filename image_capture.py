#import libraries 
#change
from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time

picam2 = Picamera2()

#configure the picamera
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode

picam2.start() #must start the camera before taking any images
time.sleep(1)

while True:
    objectName = input("Enter object name: ")
    if objectName == 'q':
        break
    else:
        i = 1
        while True: 
            cmd = input("Press enter to capture an image.")
            if cmd == 'q':
                break
            else:
                img_name = objectName + str(i) + '.jpg'
                picam2.capture_file(img_name) #take image 
                i = i + 1
picam2.stop() #stop the picam 