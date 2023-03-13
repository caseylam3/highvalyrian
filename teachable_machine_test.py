
from keras.models import load_model  # TensorFlow is required for Keras to work
import cv2  # Install opencv-python
import numpy as np
from picamera2 import Picamera2 
from libcamera import controls
import time
# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("keras_model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()

# CAMERA can be 0 or 1 based on default camera of your computer
camera = Picamera2()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode
camera.start() #must start the camera before taking any images
time.sleep(1)

while True:
    # Grab the webcamera's image.
    image = camera.capture_file("test.jpg")
    image = cv2.imread("test.jpg") #read image with open cv
    #image = camera.capture_array("main")

    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (int(224), int(224)), interpolation=cv2.INTER_AREA)

    img = cv2.imread("image.jpg") #read image with open cv

    # Show the image in a window
    cv2.imshow("Picam Image", image)

    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

    # Normalize the image array
    image = (image / 127.5) - 1

    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    # Print prediction and confidence score
    print("Class:", class_name[2:], end="")
    print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

    # Listen to the keyboard for presses.
    keyboard_input = cv2.waitKey(1)

    # 27 is the ASCII for the esc key on your keyboard.
    if keyboard_input == 27:
        break

camera.stop() #stop the picam 
cv2.destroyAllWindows()