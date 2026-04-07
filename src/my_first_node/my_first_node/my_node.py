import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info("第一个节点测试.")
        self.publisher = self.create_publisher(Int32,'send_msg',10)
        self.counter = 0
        self.timer = self.create_timer(1,self.timer_callback)
    
    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {self.counter}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()