import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
import rclpy
from rclpy.node import Node
import time
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/robot_interfaces/compressed',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher for the detected object position
        self.position_publisher = self.create_publisher(
            UInt8,
            '/can_in_view',
            10
        )

        self.bridge = CvBridge()

    def listener_callback(self, data):
        start_time = time.time()  # Start timing the processing

        try:
            # Convert ROS image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Convert the image to the HSV color space
            hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            # Define the range for blue color in HSV
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])

            # Create a mask for blue color
            mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

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
            position = 0  # Default to 0 (Object not detected)

            # Image width to determine left, center, right regions
            img_width = current_frame.shape[1]
            left_boundary = img_width * 0.40  # Narrower center region
            right_boundary = img_width * 0.60  # Narrower center region

            for contour in contours:
                # Get bounding box of the contour
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2

                # Draw bounding box around detected object
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Determine position of the object with refined boundaries
                object_detected = True
                if center_x < left_boundary:
                    position = 1  # Left
                elif center_x > right_boundary:
                    position = 3  # Right
                else:
                    position = 2  # Center

            if not object_detected:
                self.get_logger().info("Object not detected")

            # Publish the position
            position_msg = UInt8()
            position_msg.data = position
            self.position_publisher.publish(position_msg)

            # Calculate and log processing time
            processing_time = time.time() - start_time
            self.get_logger().info(f"Detection status: {'Object detected' if object_detected else 'Object not detected'}")
            self.get_logger().info(f"Processing time: {processing_time:.2f} seconds")

            # Optionally display the resulting frame
            #cv2.imshow("Camera Feed", current_frame)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
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