import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class blah(Node):

    def __init__(self, node :Node):
        self.node = node
        self.drone_whycon_pose_array = PoseArray()
        self.subscription = self.create_subscription(
            PoseArray,
            "/whycon/poses",
            self.whycon_callback,
            1
        )
    
    def whycon_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        print(msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z)
# parser = argparse.ArgumentParser(description='Process some integers.')
# parser.add_argument('--image', metavar='N', type=str,
#                     help='an integer for the accumulator')
# parser.add_argument('--sum', dest='accumulate', action='store_const',
#                     const=sum, default=max,
#                     help='sum the integers (default: find the max)')

# args = parser.parse_args()
# print(args.image)
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('controller')
    minimal_subscriber = blah(node)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()