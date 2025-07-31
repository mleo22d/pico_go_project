import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoutePublisher(Node):
    def __init__(self):
        super().__init__('route_publisher')
        self.pub = self.create_publisher(String, '/robot_1/route_assign', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_route)
        self.counter = 0

    def publish_route(self):
        msg = String()
        msg.data = "RRRR"
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.counter += 1
        if self.counter >= 4:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RoutePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
