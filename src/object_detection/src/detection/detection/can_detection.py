import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO # type: ignore
NOT_IN_VIEW = 0
LEFT = 1
CENTER = 2
RIGHT = 3
class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
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

        # Load YOLOv8 model
        self.model = YOLO("/root/ros_ws/src/object_detection/src/detection/detection/best.pt")  # Load your YOLOv8 model file
        self.model.fuse()  # Optimize the model if needed

    def listener_callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Perform object detection using YOLOv8
            results = self.model(current_frame)

            # Extract results (bounding boxes)
            detections = results[0].boxes  # Get detection boxes
            class_names = self.model.names  # Get class names

            # Image width to determine left, center, right regions
            img_width = current_frame.shape[1]
            left_boundary = img_width * 0.45  # Narrower center region
            right_boundary = img_width * 0.55  # Narrower center region

            # Initialize detection flag and position
            object_detected = False
            position = NOT_IN_VIEW  # Default to 0 (Object not detected)

            for box in detections:
                # Ensure the box has the required fields: x1, y1, x2, y2, confidence, class_id
                if not hasattr(box, 'xyxy') or not hasattr(box, 'conf') or not hasattr(box, 'cls'):
                    self.get_logger().warn(f"Skipping box with insufficient data: {box}")
                    continue

                # Extract the bounding box coordinates, confidence, and class ID
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = float(box.conf.item())  # Convert confidence to a scalar float
                class_id = int(box.cls.item())  # Convert class ID to a scalar int

                label = class_names[class_id]  # Get class name

                # Object detected
                object_detected = True
                center_x = (x1 + x2) // 2  # Calculate the center x-coordinate of the bounding box

                # Determine position of the object with refined boundaries
                if center_x < left_boundary:
                    position = LEFT  # Left
                elif center_x > right_boundary:
                    position = RIGHT  # Right
                else:
                    position = CENTER  # Center

                # Draw bounding box and label
                cv2.rectangle(current_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(current_frame, f"{label} {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if not object_detected:
                self.get_logger().info("Object not detected")

            # Publish the position
            position_msg = UInt8()
            position_msg.data = position
            self.position_publisher.publish(position_msg)

            # Display the resulting frame
            #cv2.imshow("Camera Feed", current_frame)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
