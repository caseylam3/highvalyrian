import rclpy # imports rclpy client library 
from rclpy.node import Node # imports Node class of rclpy library
from std_msgs.msg import String # imports ROS2 built-in string message type
from geometry_msgs.msg import Twist,Vector3
from rclpy.qos import qos_profile_sensor_data
from picamera2 import Picamera2 
import cv2
import numpy as np
from libcamera import controls
import time
from keras.models import load_model  # TensorFlow is required for Keras to work
import RPi.GPIO as GPIO

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("keras_model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()

GPIO.setmode(GPIO.BOARD)

PIN_TRIGGER = 7
PIN_ECHO = 11

GPIO.setup(PIN_TRIGGER, GPIO.OUT)
GPIO.setup(PIN_ECHO, GPIO.IN)

GPIO.output(PIN_TRIGGER, GPIO.LOW)

print("Waiting for sensor to settle")
time.sleep(1)

camera = Picamera2()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode
camera.start() #must start the camera before taking any images
time.sleep(1)

# Check to see if there is an object within 6 inches of distance
def getDist():
    print("Calculating distance")

    GPIO.output(PIN_TRIGGER, GPIO.HIGH)

    time.sleep(0.00001)

    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    while GPIO.input(PIN_ECHO)==0:
        pulse_start_time = time.time()
    while GPIO.input(PIN_ECHO)==1:
        pulse_end_time = time.time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round((pulse_duration * 17150) / 2.54, 2)
    print("Distance:", distance, "inches")
    return distance

# Use the model from teachable machine to analyze the image
def processImage():

    image = camera.capture_file("test.jpg")
    image = cv2.imread("test.jpg") #read image with open cv


    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (int(224), int(224)), interpolation=cv2.INTER_AREA)

    # Show the image in a window
    #cv2.imshow("Picam Image", image)

    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

    # Normalize the image array
    image = (image / 127.5) - 1

    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    className = class_name[2:]
    confidence = str(np.round(confidence_score * 100))[:-2]

    # Print prediction and confidence score
    print("Class:", className, end="")
    print("Confidence Score:", confidence, "%")
    
    return className

# Based on the image class, proximity, and the turn history, define the next move
def getNextAction(turn_remaining, turn_type):
    nextAction = 'No'
    if turn_remaining > 0:
        nextAction = turn_type
        turn_remaining = turn_remaining - 1

    else:
        if getDist() < 7:
            imageClass = processImage()
            print("Image Class: " + imageClass)

            ## PART TO EDIT IN CLASS
            #Lturn for left turn, Rturn for turn, turn remaining = how many times it should go 30 degrees - 1 (90 degress, turn remaining = 2)
        
            #bear
            if imageClass == class_names[0][2:]:
                nextAction = turn_type = 'Rturn'
                turn_remaining = 2
            #elephant
            elif imageClass == class_names[1][2:]:
                nextAction = turn_type = 'Lturn'
                turn_remaining = 2
            #kiwi
            elif imageClass == class_names[2][2:]:
                nextAction = turn_type = 'Rturn'
                turn_remaining = 2
            #mario
            elif imageClass == class_names[3][2:]:
                nextAction = turn_type = 'Lturn'
                turn_remaining = 2
            #mug
            elif imageClass == class_names[4][2:]:
                nextAction = turn_type = 'Rturn'
                turn_remaining = 2
            #rubik
            elif imageClass == class_names[5][2:]:
                nextAction = turn_type = 'Lturn'
                turn_remaining = 2
            #tractor
            elif imageClass == class_names[6][2:]:
                nextAction = turn_type = 'Rturn'
                turn_remaining = 2
        else:
            nextAction = 'Forward'
    return nextAction, turn_remaining, turn_type

# Creates SimplePublisher class which is a subclass of Node 
class SimplePublisher(Node):

    # Defines class constructor
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_velocities(self, turn_remaining, turn_type):
        twist = Twist()
        nextAction, turn_remaining, turn_type = getNextAction(turn_remaining, turn_type)
        print("next action: " + nextAction)
        print("turn remaining: " + str(turn_remaining))
        print("turn type: " + turn_type)

        # Turn or go forward based on the action
        if nextAction == 'Lturn':
            twist.angular.z = float(1)
        elif nextAction == 'Rturn':
            twist.angular.z = float(-1)
        elif nextAction == 'Forward':
            twist.linear.x = float(0.1)

        print("Current action: " + str(twist.angular.z))   
        self.publisher_.publish(twist)
        # Sleep for a second when turning to give time for turn to complete
        time.sleep(abs(twist.angular.z)*0.65)

        return turn_remaining, turn_type


def main(args=None):
    # Initializes ROS2 communication and allows Nodes to be created
    rclpy.init(args=args)
    node1 = SimplePublisher()

    turn_remaining = 0
    turn_type = 'No'
    processImage()

    while True:
        try:
            # run loop
            turn_remaining, turn_type = node1.publish_velocities(turn_remaining, turn_type)
            time.sleep(0.1)
            
        # Stops the code if CNTL-C is pressed on the keyboard    
        except KeyboardInterrupt:
            print('\nCaught Keyboard Interrupt')

            # Destroys the node that was created
            node1.destroy_node()

            # Shuts down rclpy 
            rclpy.shutdown()


if __name__ == '__main__':
    # Runs the main function
    main()

