import sys

import rclpy
from rclpy.node import Node

from module_2_services.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)

    node = Node('add_two_ints_client')
    cli = node.create_client(AddTwoInts, 'add_two_ints')

    req = AddTwoInts.Request()
    req.a = int(sys.argv[1])
    req.b = int(sys.argv[2])

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result of add_two_ints: for %d + %d = %d' %
            (req.a, req.b, future.result().sum))
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
