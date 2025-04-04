import threading
import time, signal
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge, CvBridgeError
from math import sin, cos

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        self.sensitivity = 10

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.subscription # Prevent unused variable warning

        # Detection state
        self.blue_found = False
        self.green_found = False
        self.red_found = False
        self.obstacle_in_front = False
        
        # Distance control parameters
        self.target_distance = 1.0  # 1 meter from blue box
        self.distance_tolerance = 0.1  # 10cm tolerance
        self.min_obstacle_distance = 0.8
        self.estimated_distance = float('inf') # Start with infinite distance
        
        # Movement flags
        self.moveBackwardsFlag = False
        self.moveForwardsFlag = False
        self.at_target_distance = False
        self.rotating = False

        # Navigation state machine
        self.state = "NAVIGATING"
        self.rotation_start_time = None
        self.rotation_duration = 10.0  # 10 seconds

        # Navigation goals
        self.goal_index = 0
        self.goals = [
            (-1.0, -5.0, 0.0),
            (7.0, -5.0, 0.0),
            (-8.0, -8.0, 0.0)
        ]
        self.sending_goal = False

    def laser_callback(self, msg):
        # Process laser scan data to detect obstacles
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]

        if len(ranges) == 0:
            self.obstacle_in_front = False
            return

        # Check the front 30 degrees
        front_ranges = np.concatenate((ranges[0:15], ranges[-15:]))
        min_distance = np.min(front_ranges)
        self.obstacle_in_front = min_distance < self.min_obstacle_distance

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Blue box detection
        lower_blue = np.array([120 - self.sensitivity, 100, 100])
        upper_blue = np.array([120 + self.sensitivity, 255, 255])
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Green box detection
        lower_green = np.array([60 - self.sensitivity, 100, 100])
        upper_green = np.array([60 + self.sensitivity, 255, 255])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Red box detection
        red_lower1 = np.array([0 - self.sensitivity, 100, 100])
        red_upper1 = np.array([0 + self.sensitivity, 255, 255])
        red_lower2 = np.array([180 - self.sensitivity, 100, 100])
        red_upper2 = np.array([180 + self.sensitivity, 255, 255])
        red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Reset detection flags
        self.blue_found = False
        self.green_found = False
        self.red_found = False

        # Handle blue box detection
        if blue_contours:
            c = max(blue_contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:  # Minimum area threshold
                self.blue_found = True
                
                # Draw a bounding circle
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 
                cv2.circle(image, center, radius, (255, 0, 0), 2)
                
                # Estimate distance based on contour area
                self.estimated_distance = (300000 / area) ** 0.5
                
                # Check if robot is at target distance
                self.at_target_distance = abs(self.estimated_distance - self.target_distance) < self.distance_tolerance

        # Handle green box detection
        if green_contours:
            c = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                self.green_found = True
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 
                cv2.circle(image, center, radius, (0, 255, 0), 2)

        # Handle red box detection
        if red_contours:
            c = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                self.red_found = True
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x),int(y)) 
                radius = int(radius) 
                cv2.circle(image, center, radius, (0, 0, 255), 2)
        
        # Display camera feed with detections
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.imshow('camera_Feed', image)
        cv2.waitKey(3)

    def move(self):
        twist = Twist()

        if self.state == "NAVIGATING":
            if self.blue_found:
                self.state = "APPROACHING"
                self.get_logger().info("Blue box found during navigation, switch to APPROACHING.")
            else:
                pass  # Wait for navigation to complete

        elif self.state == "ROTATING":
            if self.blue_found:
                self.state = "APPROACHING"
                self.get_logger().info("Blue box found during rotation, switch to APPROACHING.")
            else:
                twist.angular.z = 0.5
                
                # Timeout rotation after 10 seconds
                if (time.time() - self.rotation_start_time) >= self.rotation_duration:
                    self.state = "NAVIGATING"
                    self.get_logger().info("Rotation complete, no blue box found. Move to next goal.")
                    self.send_next_goal()

        elif self.state == "APPROACHING":
            if self.blue_found:
                if self.at_target_distance:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.state = "STOPPED"
                    self.get_logger().info(f"Reached target distance: {self.estimated_distance:.2f}m.")
                else:
                    distance_error = self.estimated_distance - self.target_distance
                    linear_vel = 0.3 * distance_error
                    linear_vel = np.clip(linear_vel, -0.2, 0.3)
                    
                    if linear_vel > 0 and self.obstacle_in_front:
                        linear_vel = 0.0
                        twist.angular.z = 0.3
                        self.get_logger().warn("Obstacle detected! Rotating to avoid.")
                    else:
                        twist.linear.x = linear_vel
                        twist.angular.z = 0.0
                        self.get_logger().info(f"Approaching blue box. Distance: {self.estimated_distance:.2f}m")
            
            else:
                self.state = "ROTATING"
                self.rotation_start_time = time.time()
                self.get_logger().info("Blue box lost, starting rotation to search.")

        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(Twist())

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("Finished all goals.")
            return

        self.state = "NAVIGATING"
        x, y, yaw = self.goals[self.goal_index]
        self.goal_index += 1
        self.sending_goal = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.nav_action_client.wait_for_server()
        send_future = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self.sending_goal = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Arrived at goal point")
        self.sending_goal = False
        
        # Start rotation to look for blue box
        self.state = "ROTATING"
        self.rotation_start_time = time.time()
        self.get_logger().info("Starting rotation to search for blue box")

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init()
    global robot
    robot = Robot()
    signal.signal(signal.SIGINT, signal_handler)
    threading.Thread(target=rclpy.spin, args=(robot,), daemon=True).start()

    time.sleep(1)
    robot.send_next_goal()

    try:
        while rclpy.ok():
            robot.move()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    