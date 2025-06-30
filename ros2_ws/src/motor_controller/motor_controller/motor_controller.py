import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Int32,
            'obstacle_distance',
            self.sensor_callback,
            10
        )
        self.publisher = self.create_publisher(Int32, 'motor_control', 10)
        self.get_logger().info('Motor Controller node started')

    def sensor_callback(self, msg):
        obstacle_distance = msg.data
        self.get_logger().info(f'Distancia alerta: {obstacle_distance}')

        # Si está muy cerca de un objeto, detenemos (envía 0), si no, seguimos (envía 1)
        command = Int32()
        if obstacle_distance == 0:  # hay un obstáculo cercano
            command.data = 0  # detener motores
        else:
            command.data = 1  # seguir moviéndose

        self.publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
