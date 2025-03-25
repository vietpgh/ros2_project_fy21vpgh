
#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal
import random


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)

        # Initialise any flags that signal a colour has been detected (default to false)
        self.blue_found = False
        self.moveBackwardsFlag = False
        self.moveForwardsFlag = False

        self.sensitivity = 10
        
        self.exploration_counter = 0

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.target_area = 3000 # Estimated area when 1m from blue box
        self.margin = 500

    def callback(self, data):

        # Convert the received image into a opencv image
        try: 
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        
        # Set the upper and lower bounds for the colour blue        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but blue using the cv2.inRange() method
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        # Find the contours that appear within the blue mask using the cv2.findContours() method
        contours, _ = cv2.findContours(blue_mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        if len(contours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            x = 500

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if area > x: #<What do you think is a suitable area?>
                # Alter the value of the flag
                self.blue_found = True

                #Check if a flag has been set = colour object detected - follow the colour object
                if area > self.target_area + self.margin:
                    # Too close to object, need to move backwards
                    # Set a flag to tell the robot to move backwards when in the main loop
                    self.moveForwardsFlag = False
                    self.moveBackwardsFlag = True
                    
                elif area < self.target_area - self.margin:
                    # Too far away from object, need to move forwards
                    # Set a flag to tell the robot to move forwards when in the main loop
                    self.moveForwardsFlag = True
                    self.moveBackwardsFlag = False
                else:
                    self.moveForwardsFlag = False
                    self.moveBackwardsFlag = False
                    
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                else:
                    cx = image.shape[1] // 2

                image_centre_x = image.shape[1] // 2
                offset = cx - image_centre_x

                self.angular_z = 0.0
                if offset > 50:
                    self.angular_z = -0.2  # Turn right
                elif offset < -50:
                    self.angular_z = 0.2   # Turn left

                print(f"[INFO] Area: {area}, Offset: {offset}")
            else:
                self.blue_found = False
        else:
            self.blue_found = False
                
            # Be sure to do this for the other colour as well
            # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

    def move(self):
        twist = Twist()

        if self.blue_found:
            twist.linear.x = 0.2 if self.moveForwardsFlag else -0.2 if self.moveBackwardsFlag else 0.0
            twist.angular.z = getattr(self, 'angular_z', 0.0)
        else:
            # Explore (rotate in place)
            self.exploration_counter += 1
            
            if self.exploration_counter % 50 == 0:
                twist.linear.x = 0.0
                twist.angular.z = random.choice([0.5, -0.5])
            else:
                twist.linear.x = 0.1
                twist.linear.z = 0.0
                
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Publish moves
            robot.move()
            time.sleep(0.1)
            pass

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

