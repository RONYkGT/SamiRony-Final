import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt8
import rclpy
from rclpy.node import Node
import time
import os
import numpy as np
import subprocess

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Declare and get parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('switch_topic', '/switch_to_qr')
        self.declare_parameter('position_topic', '/can_in_view')

        self.declare_parameter('lower_blue_hue', 100)
        self.declare_parameter('lower_blue_saturation', 150)
        self.declare_parameter('lower_blue_value', 0)

        self.declare_parameter('upper_blue_hue', 140)
        self.declare_parameter('upper_blue_saturation', 255)
        self.declare_parameter('upper_blue_value', 255)

        self.declare_parameter('left_boundary_ratio', 0.45)
        self.declare_parameter('right_boundary_ratio', 0.55)

        # Get parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        switch_topic = self.get_parameter('switch_topic').get_parameter_value().string_value
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value

        lower_blue_hue = self.get_parameter('lower_blue_hue').get_parameter_value().integer_value
        lower_blue_saturation = self.get_parameter('lower_blue_saturation').get_parameter_value().integer_value
        lower_blue_value = self.get_parameter('lower_blue_value').get_parameter_value().integer_value

        upper_blue_hue = self.get_parameter('upper_blue_hue').get_parameter_value().integer_value
        upper_blue_saturation = self.get_parameter('upper_blue_saturation').get_parameter_value().integer_value
        upper_blue_value = self.get_parameter('upper_blue_value').get_parameter_value().integer_value

        left_boundary_ratio = self.get_parameter('left_boundary_ratio').get_parameter_value().double_value
        right_boundary_ratio = self.get_parameter('right_boundary_ratio').get_parameter_value().double_value

        # Subscribe to the image topic
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10
        )

        # Subscribe to the switch topic
        self.switch_subscription = self.create_subscription(
            Bool,
            switch_topic,
            self.switch_callback,
            10
        )

        # Create a publisher for the detected object position
        self.position_publisher = self.create_publisher(
            UInt8,
            position_topic,
            10
        )

        self.bridge = CvBridge()
        self.shutdown_flag = False

        # Use parameters for color detection and boundary definitions
        self.lower_blue = np.array([lower_blue_hue, lower_blue_saturation, lower_blue_value])
        self.upper_blue = np.array([upper_blue_hue, upper_blue_saturation, upper_blue_value])
        self.left_boundary_ratio = left_boundary_ratio
        self.right_boundary_ratio = right_boundary_ratio

    def listener_callback(self, data):
        if self.shutdown_flag:
            return

        start_time = time.time()

        try:
            # Convert ROS image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Convert the image to the HSV color space
            hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            # Create a mask for blue color
            mask = cv2.inRange(hsv_frame, self.lower_blue, self.upper_blue)

            # Apply the mask to the image
            blue_objects = cv2.bitwise_and(current_frame, current_frame, mask=mask)

            # Convert the masked image to grayscale
            gray_frame = cv2.cvtColor(blue_objects, cv2.COLOR_BGR2GRAY)

            # Perform edge detection using Canny
            edges = cv2.Canny(gray_frame, 100, 200)

            # Find contours in the edged image
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Initialize detection flag and position
            object_detected = False
            position = 0

            # Find the largest contour based on area
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2

                # Draw the bounding rectangle of the largest contour
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Determine the position of the object
                img_width = current_frame.shape[1]
                left_boundary = img_width * self.left_boundary_ratio
                right_boundary = img_width * self.right_boundary_ratio

                if center_x < left_boundary:
                    position = 1  # Left
                elif center_x > right_boundary:
                    position = 3  # Right
                else:
                    position = 2  # Center

                object_detected = True

            if not object_detected:
                self.get_logger().info("Object not detected")

            position_msg = UInt8()
            position_msg.data = position
            self.position_publisher.publish(position_msg)

            processing_time = time.time() - start_time
            self.get_logger().info(f"Detection status: {'Object detected' if object_detected else 'Object not detected'}")
            self.get_logger().info(f"Processing time: {processing_time:.2f} seconds")

            # Optionally display the resulting frame
            # cv2.imshow("Camera Feed", current_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")


    def switch_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received shutdown signal, running qr_detection.py...")
            self.shutdown_flag = True
            
            # Construct the full path to the qr_detection.py script
            script_directory = os.path.dirname(os.path.abspath(__file__))
            qr_detection_script = os.path.join(script_directory, 'qr_detection.py')
            
            if os.path.exists(qr_detection_script):
                subprocess.Popen(['python3', qr_detection_script])
            else:
                self.get_logger().error(f"qr_detection.py not found at {qr_detection_script}")
            
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        while rclpy.ok() and not image_subscriber.shutdown_flag:
            rclpy.spin_once(image_subscriber)
    except rclpy.executors.ExternalShutdownException:
        image_subscriber.get_logger().info("External shutdown signal received.")
    except Exception as e:
        image_subscriber.get_logger().error(f"Unexpected error: {e}")
    finally:
        image_subscriber.get_logger().info("Cleaning up and shutting down...")
        if rclpy.ok():
            image_subscriber.destroy_node()
            rclpy.shutdown()
        else:
            image_subscriber.get_logger().warning("rclpy was not ok during shutdown.")

if __name__ == '__main__':
    main()
