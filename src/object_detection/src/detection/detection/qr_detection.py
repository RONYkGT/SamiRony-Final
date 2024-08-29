import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from cv_bridge import CvBridge

# Define position constants
LEFT = 1
RIGHT = 3
CENTER = 2
NOT_DETECTED = 0

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')

        # Declare parameters
        self.declare_parameter('image_topic', '/robot_interfaces/compressed')
        self.declare_parameter('position_topic', '/qr_in_view')
        self.declare_parameter('left_boundary_ratio', 0.45)
        self.declare_parameter('right_boundary_ratio', 0.55)

        # Get parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        position_topic = self.get_parameter('position_topic').get_parameter_value().string_value
        self.left_boundary_ratio = self.get_parameter('left_boundary_ratio').get_parameter_value().double_value
        self.right_boundary_ratio = self.get_parameter('right_boundary_ratio').get_parameter_value().double_value

        # Create a subscriber to the image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            image_topic, 
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher for the QR code position
        self.position_publisher = self.create_publisher(
            UInt8,
            position_topic,
            10
        )

        self.bridge = CvBridge()
        self.font = cv2.FONT_HERSHEY_PLAIN

    def listener_callback(self, msg):
        try:
            # Convert ROS compressed image message to OpenCV image
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = frame.shape

            decodedObjects = pyzbar.decode(frame)
            
            # Initialize position as NOT_DETECTED
            position = NOT_DETECTED

            for obj in decodedObjects:
                # Get the bounding box coordinates
                points = obj.polygon
                if len(points) == 4:  # Only handle QR codes with 4 points
                    # Draw bounding box
                    cv2.polylines(frame, [np.array(points, dtype=np.int32)], True, (0, 255, 0), 2)
                    
                    # Calculate the center of the bounding box
                    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = points
                    center_x = int((x1 + x2 + x3 + x4) / 4)
                    
                    # Determine position of QR code
                    if center_x < width * self.left_boundary_ratio:
                        position = LEFT
                    elif center_x > width * self.right_boundary_ratio:
                        position = RIGHT
                    else:
                        position = CENTER

                    # Display the decoded text
                    cv2.putText(frame, str(obj.data, 'utf-8'), (50, 50), self.font, 2, (255, 0, 0), 3)
                    
                    # Display the position of the QR code
                    position_text = f"Position: {position}"
                    cv2.putText(frame, position_text, (50, 100), self.font, 1, (0, 255, 0), 2)

            # If no QR codes are detected, set position to NOT_DETECTED
            if not decodedObjects:
                position = NOT_DETECTED
                position_text = f"Position: {position}"
                cv2.putText(frame, position_text, (50, 100), self.font, 1, (0, 255, 0), 2)

            # Publish the position
            position_msg = UInt8()
            position_msg.data = position
            self.position_publisher.publish(position_msg)

            # Display the frame
            # cv2.imshow("QRCode Detector", frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector)
    qr_code_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()