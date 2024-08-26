import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
import cv2
from cv_bridge import CvBridge

# Define constants for QR code positions
NO_QR = 0
RIGHT = 1
CENTER = 2
LEFT = 3

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.qr_decoder = cv2.QRCodeDetector()
        self.bridge = CvBridge()

        # Publisher to publish the position of the detected QR code
        self.publisher_ = self.create_publisher(UInt8, 'qr_in_view', 10)

        # Subscriber to receive images from the robot's camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('QR Code Detector Node has been started.')

    def image_callback(self, msg):
        # Convert the ROS Image message to a CV2 image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect the QR code
        retval_detection, bbox = self.qr_decoder.detect(frame)

        if retval_detection:
            # Draw the bounding box on the frame
            self._draw_bounding_box(frame, bbox)
            
            # Determine the position of the QR code
            position = self._get_qr_position(bbox, frame.shape[1])
            self.get_logger().info(f"QR Code detected! Position: {position}")
        else:
            position = NO_QR  # No QR code detected
            self.get_logger().info("QR Code not detected")

        # Publish the position to the topic
        self.publisher_.publish(UInt8(data=position))

        # Display the results (optional for debugging)
        #cv2.imshow("QR Code Detection", frame)
        #cv2.waitKey(1)

    def _draw_bounding_box(self, frame, bbox):
        """Draws a bounding box around the detected QR code."""
        if bbox is not None:
            for i in range(len(bbox)):
                start_point = tuple(map(int, bbox[i][0]))
                end_point = tuple(map(int, bbox[(i + 1) % len(bbox)][0]))
                cv2.line(frame, start_point, end_point, (255, 255, 0), 3)

    def _get_qr_position(self, bbox, frame_width):
        """Determines the position of the QR code relative to the frame center."""
        if bbox is not None:
            # Compute the center of the bounding box
            x_coords = [point[0][0] for point in bbox]
            x_center = (max(x_coords) + min(x_coords)) / 2

            # Frame center X coordinate
            frame_center_x = frame_width / 2

            # Determine position based on the center of the QR code
            if x_center < frame_center_x - frame_center_x / 10:  # Narrowed left region
                return LEFT
            elif x_center > frame_center_x + frame_center_x / 10:  # Narrowed right region
                return RIGHT
            else:
                return CENTER
        return NO_QR  # No QR code detected

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
