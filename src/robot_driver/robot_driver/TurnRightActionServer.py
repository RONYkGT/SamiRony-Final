import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from bot_behavior_interfaces.action import TurnRight  # Import the TurnRight action
from geometry_msgs.msg import Twist

class TurnRightActionServer(Node):
    def __init__(self):
        super().__init__('turn_right_action_server')
        
        # Declare and set the turning speed 
        self.turn_speed = -0.5  

        self._action_server = ActionServer(
            self,
            TurnRight,
            'turn_right',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received request to turn right.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request. Stopping the robot.')
        self.stop_robot()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing turn right with speed {self.turn_speed}...')
        
        twist_msg = Twist()
        twist_msg.angular.z = self.turn_speed  # Use the class attribute for angular speed

        # Create a timer to publish twist messages periodically
        self.timer = self.create_timer(0.1, self.publish_twist_msg)
        self.timer_msg = twist_msg

        while not goal_handle.is_cancel_requested:
            rclpy.spin_once(self, timeout_sec=0.1)

        if goal_handle.is_cancel_requested:
            self.get_logger().info('Goal canceled. Stopping the robot.')
            self.stop_robot()
            goal_handle.canceled()
            return TurnRight.Result()

    def publish_twist_msg(self):
        if self.timer_msg:
            self.publisher_.publish(self.timer_msg)

    def stop_robot(self):
        # Publish zero velocity to stop the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        if self.timer:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    turn_right_action_server = TurnRightActionServer()
    rclpy.spin(turn_right_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
