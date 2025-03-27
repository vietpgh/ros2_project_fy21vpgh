import threading
import sys, time, signal, random
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from math import sin, cos

class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Detection state
        self.blue_found = False
        self.green_found = False
        self.red_found = False
        self.obstacle_in_front = False
        self.angular_z = 0.0
        self.sensitivity = 10
        
        # Distance control parameters
        self.target_distance = 1.0  # 1 meter from blue box
        self.distance_tolerance = 0.1  # ±10cm tolerance
        self.min_obstacle_distance = 1.2
        
        # Box tracking
        self.blue_box_area = 0
        self.blue_box_center_x = 0
        self.estimated_distance = float('inf')
        
        # Movement flags
        self.moveBackwardsFlag = False
        self.moveForwardsFlag = False
        self.at_target_distance = False
        self.rotating = False

        # Navigation goal
        self.goal_index = 0
        self.goals = [
            (-8.5, -9.0, 0.0),
            (-1.0, -5.0, 0.0),
            (2.0, -2.0, 0.0),
            (5.0, -4.0, 0.0),
            (3.0, -8.0, 0.0)
        ]
        self.sending_goal = False

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = ranges[np.isfinite(ranges)]

        if len(ranges) == 0:
            self.obstacle_in_front = False
            return

        front_ranges = np.concatenate((ranges[0:15], ranges[-15:]))
        min_distance = np.min(front_ranges)
        self.obstacle_in_front = min_distance < self.min_obstacle_distance

    def image_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))
            return
        
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        
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
                self.blue_box_area = area
                
                # Calculate center and offset
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else image.shape[1] // 2
                self.blue_box_center_x = cx - (image.shape[1] // 2)
                
                # Estimate distance based on area (empirical relationship)
                self.estimated_distance = (300000 / area) ** 0.5
                
                # Check if we're at target distance
                self.at_target_distance = abs(self.estimated_distance - self.target_distance) < self.distance_tolerance

        # Handle green box detection
        if green_contours:
            c = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                self.green_found = True

        # Handle red box detection
        if red_contours:
            c = max(red_contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                self.red_found = True

    def move(self):
        twist = Twist()

        if self.blue_found:
            self.rotating = False
            if self.at_target_distance:
                # Stop when within 1m ± tolerance
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Stopped at target distance: {self.estimated_distance:.2f}m")
            else:
                # Move towards/away from blue box to reach target distance
                distance_error = self.estimated_distance - self.target_distance
                
                # Proportional control for linear velocity
                linear_vel = 0.3 * distance_error
                linear_vel = np.clip(linear_vel, -0.2, 0.3)
                
                angular_vel = 0.0
                    
                # Apply obstacle check
                if linear_vel > 0 and self.obstacle_in_front:
                    linear_vel = 0.0
                    angular_vel = 0.3

                twist.linear.x = linear_vel
                twist.angular.z = angular_vel
                
                self.get_logger().info(f"Approaching blue box. Distance: {self.estimated_distance:.2f}m, Target: {self.target_distance}m")

        # If red or green is detected, ignore them and rotate in place
        elif self.red_found or self.green_found:
            self.get_logger().info("Red/Green box detected. Rotating in place.")
            self.rotate_to_find_blue()

        # Rotate if no box found
        elif self.rotating:
            twist.angular.z = 0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)


    def stop(self):
        self.publisher.publish(Twist())

    def rotate_to_find_blue(self):
        # Rotate in place for 360 degrees to search for the blue box
        self.rotating = True
        threading.Timer(10.0, self.stop_rotation_and_move_on).start()

    def stop_rotation_and_move_on(self):
        self.rotating = False
        if self.blue_found:
            self.get_logger().info("Blue box found, moving towards it.")
            self.move()
        else:
            self.get_logger().info("No blue box found, moving to next goal.")
            self.send_next_goal()

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("Finished all goals.")
            return

        x, y, theta = self.goals[self.goal_index]
        self.goal_index += 1
        self.sending_goal = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(theta / 2)
        goal_msg.pose.pose.orientation.w = cos(theta / 2)

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
        self.get_logger().info("Arrived at goal point. Rotating to scan.")
        self.sending_goal = False

        self.rotating = True
        threading.Timer(10.0, self.stop_rotation_and_move_on).start()

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
