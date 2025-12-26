import rclpy
from rclpy.node import Node

from module_2_services.srv import AddTwoInts


class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Ready to add two ints.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.get_logger().info('Sending back response: [%d]' % response.sum)
        return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_service = AddTwoIntsService()

    rclpy.spin(add_two_ints_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
