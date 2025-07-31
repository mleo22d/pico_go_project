import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml

with open('robots_config.yaml', 'r') as file:
    config = yaml.safe_load(file)

NUM_ROBOTS = config['num_robots']


class ObstacleMonitor(Node):

    def __init__(self):
        super().__init__('obstacle_monitor')

        # Lista de robots a monitorear
        self.robot_ids = [1,2]  
        self.robot_publishers = {}
        self.robot_subscribers = []
        self.obstacle_states = {robot_id: False for robot_id in self.robot_ids}
        self.last_alert_state = False

        for robot_id in self.robot_ids:
            obstacle_alert_topic = f'/robot_{robot_id}/obstacle_alert'
            control_topic = f'/robot_{robot_id}/line_follow_start'

            # Crear publicador para detener el robot
            self.robot_publishers[robot_id] = self.create_publisher(Bool, control_topic, 10)

            # Crear suscriptor para alertas
            sub = self.create_subscription(Bool, obstacle_alert_topic, self.make_callback(robot_id), 10)
            self.robot_subscribers.append(sub)

    def make_callback(self, robot_id):
        
        def callback(msg):
            self.obstacle_states[robot_id] = msg.data

            if msg.data and not self.last_alert_state:
                self.get_logger().warn(f'Obstacle detected by robot_{robot_id}! Stopping.')
                stop_msg = Bool()
                stop_msg.data = False
                for pub in self.robot_publishers.values():
                    pub.publish(stop_msg)
                self.last_alert_state = True

            elif not any(self.obstacle_states.values()) and self.last_alert_state:
                self.get_logger().info('All paths clear. Reactivating robots.')
                go_msg = Bool()
                go_msg.data = True
                for pub in self.robot_publishers.values():
                    pub.publish(go_msg)
                self.last_alert_state = False

        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
