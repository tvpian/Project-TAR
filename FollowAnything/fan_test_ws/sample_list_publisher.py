import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class FloatListPublisher(Node):
    def __init__(self):
        super().__init__('float_list_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target_pose', 10)
        self.timer = self.create_timer(1, self.publish_list)

    def publish_list(self):
        msg = Float32MultiArray()
        # Example list of floats
        float_list = [-1., 0., 0. , 0.]
        msg.data = float_list
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)
    float_list_publisher = FloatListPublisher()
    rclpy.spin(float_list_publisher)
    float_list_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
