import rclpy
from rclpy.node import Node
from hri_msgs.msg import Expression

from rclpy.action import ActionClient
from tts_msgs.action import TTS  # Make sure tts_msgs is installed and available

class CycleExpressions(Node):
    def __init__(self):
        super().__init__('cycle_expressions')
        self.publisher_ = self.create_publisher(Expression,
                                                '/robot_face/expression', 1)

        self.expressions = ['happy', 'sad', 'angry',
                            'surprised', 'neutral', 'furious']
        self.index = 0

        self.timer_ = self.create_timer(1.0, self.publish_expression)

        # Action client for TTS
        self.tts_client = ActionClient(self, TTS, 'say')
        self.tts_timer_ = self.create_timer(60.0, self.send_tts_goal)  # every 60 seconds

    def publish_expression(self):
        msg = Expression()
        msg.expression = self.expressions[self.index]

        rclpy.logging.get_logger('cycle_expressions').info(
            'Publishing: "%s"' % msg.expression)

        self.publisher_.publish(msg)
        self.index = (self.index + 1) % len(self.expressions)

    def send_tts_goal(self):
        if not self.tts_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('TTS action server not available')
            return

        goal_msg = TTS.Goal()
        text = 'Hello I am Tiago Pro!'
        goal_msg.input = text
        goal_msg.locale = ''
        goal_msg.voice = 'Andreas'

        self.get_logger().info('Sending TTS goal: "%s"' % text)
        self.tts_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CycleExpressions()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()