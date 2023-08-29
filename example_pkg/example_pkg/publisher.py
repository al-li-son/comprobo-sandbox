import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')

        # Create a timer for the publisher
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.msg)

        # Create a publisher
        self.publisher = self.create_publisher(String, 'greetings', 10)

    def msg(self):
        """
        Callback function for publisher
        """
        msg = String()
        msg.data = "Hello World"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()