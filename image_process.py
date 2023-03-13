''' Code from ROS2 Docs
Modifications by Briana Bouchard
This code creates a publisher node and publishes a string to a topic with a counter every 0.5 seconds.
'''

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

# CAMERA can be 0 or 1 based on default camera of your computer
camera = Picamera2()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode
camera.start() #must start the camera before taking any images
time.sleep(1)

global halfway_turned
halfway_turned = False


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


def processImage():

    image = camera.capture_file("test.jpg")
    image = cv2.imread("test.jpg") #read image with open cv


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

    className = class_name[2:]
    confidence = str(np.round(confidence_score * 100))[:-2]

    # Print prediction and confidence score
    print("Class:", className, end="")
    print("Confidence Score:", confidence, "%")
    
    return className

def getNextAction():
    Angularx= 0
    Angulary= 0 
    Angularz= 0
    Linearx= 0
    Lineary= 0
    Linearz= 0
    
    turn = False

    if getDist() < 6:

        nextAction = processImage()
        print("nextAction: " + nextAction)

        #just testing with this
        if nextAction == class_names[4][2:]:
            Angularz = 0.785
            turn = True
        elif nextAction == class_names[5][2:]:
            Angularz = -0.785
            turn = True
            
    else:
        print("TOO FAR")
        Angularx= 0
        Angulary= 0 
        Angularz= 0
        Linearx= 0.1
        Lineary= 0
        Linearz= 0

    return[Linearx,Lineary,Linearz,Angularx,Angulary,Angularz,turn]

# Creates SimplePublisher class which is a subclass of Node 
class SimplePublisher(Node):

    # Defines class constructor
    def __init__(self):

        # Initializes and gives Node the name simple_publisher and inherits the Node class's attributes by using 'super()'
        super().__init__('simple_publisher')

        # Creates a publisher based on the message type "String" that has been imported from the std_msgs module above
        #* Works if you actaully just put cmd_vel but isnt sending to to the subscriber tho
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set delay in seconds
        timer_period = 0.5  

        # Creates a timer that triggers a callback function after the set timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Sets initial counter to zero
        self.i = 0

    def timer_callback(self):
        # Assigns message type "String" that has been imported from the std_msgs module above
        self.msg = Twist()
        
        # Publishes `msg` to topic 
        # x = input("Start Or Stop:")
        # if(x == 'Start'):
        values = getNextAction()
        halfway_turned = values.pop()

        self.msg.linear.x = float(values[0])
        self.msg.linear.y = float(values[1])
        self.msg.linear.z = float(values[2])
        self.msg.angular.x =float(values[3])
        self.msg.angular.y =float(values[4])
        self.msg.angular.z =float(values[5])

        self.publisher_.publish(self.msg) 
        time.sleep(1)


        if halfway_turned == True:
            self.publisher_.publish(self.msg)

class SubScriberNodes(Node):

     def __init__(self):

        super().__init__('Subscriber_Node')

        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.listener_callback,10)
        self.subscription
    
     def listener_callback(self, msg):

        self.get_logger().info('Publishing: %s' % msg.linear.x) 



def main(args=None):
    # Initializes ROS2 communication and allows Nodes to be created
    rclpy.init(args=args)

    # Creates the SimplePublisher Node
    simple_publisher = SimplePublisher()
    Subscriber_node = SubScriberNodes()


    try:
        # Spins the Node to activate the callbacks
        rclpy.spin(simple_publisher)
        rclpy.spin(Subscriber_node)

        

    # Stops the code if CNTL-C is pressed on the keyboard    
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')

        # Destroys the node that was created
        simple_publisher.destroy_node()

        # Shuts down rclpy 
        rclpy.shutdown()


if __name__ == '__main__':
    # Runs the main function
    main()

