
# Title: Looping Airtable Publisher
# Authors: Kelly MacDonald and Darya Clark
# Purpose: Use data from Airtable at https://airtable.com/invite/l?inviteId=invxMhCvgAZMqpT64&inviteToken=8216393138120dcf2c66adf0885d0d3ab69e43b7f4335c79930c2e24ef079a5c&utm_medium=email&utm_source=product_team&utm_content=transactional-alerts 
# to move the iRobot Create3 robot remotely
# Date: 5 March 2023

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import requests
import json 

# Title: PublisherNode
# Purpose: initialize a node in main that will publish to the cmd_vel topic
# and contain a function to retrieve data from Airtable and correspondingly
# move the robot 
class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        


    def publish_velocities(self,linear_prev, angular_prev):
        twist = Twist()

        # if the value is not the same as the previous and does not have a non
        # readable value, move the robot forward or backwards the given amt
        if linear != linear_prev and linear != {'specialValue': 'NaN'}:
            print("linear prev: ", linear_prev)
            print("linear: ", linear)
            print("Driving distance: ", linear)
            twist.linear.x = float(linear)
            print(twist.linear.x)
            linear_prev = linear
            
        # if the value is not the same as the previous and does not have a non
        # readable value, turn the robot forward or backwards the given amt
        if angular != angular_prev and angular != {'specialValue': 'NaN'}:
            print("Turning angle: ", angular)
            twist.angular.z = float(angular)
            print(twist.angular.z)
            angular_prev = angular

        self.publisher_.publish(twist)

        # if the user sets quit to 1, quit after this iteration
        quitval = data['records'][1]['fields']['quit']
        if quitval == 1:
            isQuit = True
        else:
            isQuit = False

        return linear_prev, angular_prev, isQuit
        
        

def main(args=None):

    rclpy.init(args=args)
    # create instance of publisher node
    node1 = PublisherNode()
    # set initial values to 0
    linear_prev = 0
    angular_prev = 0

    while True:
        try:
            # run until quit is true
            linear_prev, angular_prev, isQuit = node1.publish_velocities(linear_prev, angular_prev)
            time.sleep(1)
            
            if isQuit:
                # restart node1 and rclpy shutdown
                node1.destroy_node()
                rclpy.shutdown()
                return


        except KeyboardInterrupt:
            print('\nCaught Keyboard Interrupt')

            # Destroys the node that was created
            node1.destroy_node()

            rclpy.shutdown()


if __name__ == '__main__':
    main()
