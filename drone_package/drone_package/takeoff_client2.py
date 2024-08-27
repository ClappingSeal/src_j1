import rclpy
from rclpy.node import Node
from msgs.srv import TAKEOFF


class Takeoff_client2(Node):

    def __init__(self, altitude):
        super().__init__('takeoff_client')
        self.client = self.create_client(TAKEOFF, 'takeoff2')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TAKEOFF.Request()
        self.req.altitude = float(altitude)  # Set altitude for takeoff

    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        try:
            response = self.future.result()
            self.get_logger().info('Takeoff Result: success: %s, message: "%s"' % (response.success, response.message))
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)

    if args is not None and len(args) > 1:
        try:
            altitude = float(args[1])
        except ValueError:
            altitude = 6  # Default value if not provided
            print(3)
            print('!!!!!!!!!!!!!!!!!!!!!')
    else:
        altitude = 6 # Default altitude for takeoff

    takeoff_client2 = Takeoff_client2(altitude)
    takeoff_client2.send_request()
    takeoff_client2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
