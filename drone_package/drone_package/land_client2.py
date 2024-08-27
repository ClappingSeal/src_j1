import rclpy
from rclpy.node import Node
from msgs.srv import LAND


class Land_client2(Node):

    def __init__(self):
        super().__init__('land_client2')
        self.client = self.create_client(LAND, 'land2')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LAND.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        try:
            response = self.future.result()
            self.get_logger().info('Land Result: success: %s, message: "%s"' % (response.success, response.message))
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    land_client2 = Land_client2()
    land_client2.send_request()
    land_client2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
