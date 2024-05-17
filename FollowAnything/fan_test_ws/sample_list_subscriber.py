import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FloatListSubscriber(Node):
    def __init__(self):
        super().__init__('float_list_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'target_pose',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Received: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    float_list_subscriber = FloatListSubscriber()
    rclpy.spin(float_list_subscriber)
    float_list_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

