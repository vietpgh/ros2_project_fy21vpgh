# Exercise 2 - detecting two colours, and filtering out the third colour and background.



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



class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        self.sensitivity = 10
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

    def callback(self, data):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as error:
            print(error)
            return
        
        
        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_red_lower1 = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper1 = np.array([0 + self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180 + self.sensitivity, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
        red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        rg_mask = cv2.bitwise_or(red_mask, green_mask)
        rb_mask = cv2.bitwise_or(red_mask, blue_mask)
        bg_mask = cv2.bitwise_or(blue_mask, green_mask)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        #bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.
        filtered_img = cv2.bitwise_and(self.image, self.image, mask=bg_mask)


        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('camera_Feed_BlueGreen',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed_BlueGreen', filtered_img)
        cv2.resizeWindow('camera_Feed_BlueGreen',320,240)
        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    
# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
