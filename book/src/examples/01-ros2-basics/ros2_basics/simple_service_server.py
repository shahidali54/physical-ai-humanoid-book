import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceServer(Node):
    """
    A simple ROS 2 service server that adds two integers.
    """
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server started. Ready to add two ints.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    try:
        rclpy.spin(simple_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        simple_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
