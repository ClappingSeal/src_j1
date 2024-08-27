import rclpy
from rclpy.node import Node
from msgs.msg import POS
from std_msgs.msg import Float64MultiArray
from datetime import datetime
import csv


class PosSubscriber(Node):
    def __init__(self):
        super().__init__('pos_sub')

        self.pub = self.create_publisher(Float64MultiArray, 'drones_info', 10)
        self.data_list = []
        self.start_time = datetime.now()

        self.subscription1 = self.create_subscription(
            POS,
            'position_topic1',
            lambda msg: self.listener_callback(msg, 1),
            10)
        self.subscription2 = self.create_subscription(
            POS,
            'position_topic2',
            lambda msg: self.listener_callback(msg, 2),
            10)

        # 드론 추가시 여기다 subscription 추가 !!!!!

    def listener_callback(self, msg, drone_id):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        base_lat = 35.2266470
        base_lon = 126.8405244

        x = (msg.x - base_lat) * LATITUDE_CONVERSION
        y = (base_lon - msg.y) * LONGITUDE_CONVERSION

        elapsed_time = (datetime.now() - self.start_time).total_seconds()

        data = Float64MultiArray()
        data.data = [float(drone_id), x, y, msg.z]
        self.pub.publish(data)

        existing_time_entry = next((item for item in self.data_list if item['Time'] == elapsed_time), None)

        if existing_time_entry is None:
            new_row = {'Time': elapsed_time, f'x{drone_id}': x, f'y{drone_id}': y, f'z{drone_id}': msg.z}
            self.data_list.append(new_row)
        else:
            existing_time_entry.update({f'x{drone_id}': x, f'y{drone_id}': y, f'z{drone_id}': msg.z})
        print(data.data)


def main(args=None):
    rclpy.init(args=args)
    pos_node = PosSubscriber()
    try:
        rclpy.spin(pos_node)
    except KeyboardInterrupt:
        keys = ['Time', 'x1', 'y1', 'z1', 'x2', 'y2', 'z2']  # 드론 추가시 여기다 정보 추가 !!!!!!!!!!
        with open('drone_positions.csv', 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, fieldnames=keys)
            dict_writer.writeheader()
            dict_writer.writerows(pos_node.data_list)
        print("Data saved to 'drone_positions.csv'")
    finally:
        pos_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
