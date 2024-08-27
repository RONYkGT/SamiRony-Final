import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os


class SequentialRunnerNode(Node):
    def __init__(self):
        super().__init__('sequential_runner')
        self.get_logger().info("Starting sequential execution...")

        # Path to your scripts
        self.can_detection_script = 'can_detection.py'
        self.qr_detection_script = 'qr_detection.py'

        # Initializing the subscriber for the 'switch_to_qr' topic
        self.subscription = self.create_subscription(
            Bool,
            'switch_to_qr',
            self.switch_callback,
            10
        )

        self.running_script1 = True  # Indicates whether script1 is currently running

        # Start the sequence by running the first script
        self.run_script1()

    def run_script1(self):
        if os.path.exists(self.can_detection_script):
            self.get_logger().info(f'Running {self.can_detection_script}')
            self.process1 = subprocess.Popen(['python3', self.can_detection_script], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.get_logger().info('Waiting for switch_to_qr message to stop script1.py...')
        else:
            self.get_logger().error(f"Script not found: {self.can_detection_script}")

    def switch_callback(self, msg):
        if msg.data and self.running_script1:
            self.get_logger().info("Received signal to switch to qr_detection.py")

            # Stop script1.py
            if self.process1:
                self.process1.terminate()
                stdout, stderr = self.process1.communicate()

                self.get_logger().info(stdout.decode())
                if stderr:
                    self.get_logger().error(stderr.decode())

            # After stopping script1, start script2
            self.running_script1 = False
            self.running_script2 = True
            self.run_script2()

    def run_script2(self):
        if os.path.exists(self.qr_detection_script):
            self.get_logger().info(f'Running {self.qr_detection_script}')
            result = subprocess.run(['python3', self.qr_detection_script], capture_output=True)
            self.get_logger().info(result.stdout.decode())
            if result.stderr:
                self.get_logger().error(result.stderr.decode())
        else:
            self.get_logger().error(f"Script not found: {self.qr_detection_script}")

        self.get_logger().info("Sequential execution completed.")


def main(args=None):
    rclpy.init(args=args)
    node = SequentialRunnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
