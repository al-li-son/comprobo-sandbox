import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')

        # Create a subscriber
        self.subscriber = self.create_subscription(String, 'greetings', self.parse_msg, 10)

    def parse_msg(self, msg):
        print(f"Greetings from my friend!: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()