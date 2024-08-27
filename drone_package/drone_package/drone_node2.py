import sys
import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import numpy as np
import logging

# Definition Library
from msgs.msg import POS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from msgs.srv import TAKEOFF, LAND
from std_msgs.msg import Float64MultiArray
from drone_package.APF_Settings import APFEnvironment

logging.getLogger('dronekit').setLevel(logging.CRITICAL)


class DroneNode2(Node):
    def __init__(self, goal_x, goal_y, goal_z):
        super().__init__('drone_node2')
        # Vehicle connection
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200, timeout=60)
        self.init_lat = self.vehicle.location.global_relative_frame.lat
        self.init_lon = self.vehicle.location.global_relative_frame.lon
        self.base_lat = 35.2266470
        self.base_lon = 126.8405244

        # QoS settings
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE

        # Publisher Node
        self.publisher = self.create_publisher(POS, 'position_topic2', qos_profile)

        # Subscriber Node
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'drones_info',
            self.subscribe_position,
            qos_profile
        )

        # Service Node
        self.takeoff_service = self.create_service(TAKEOFF, 'takeoff2', self.takeoff_callback)
        self.land_service = self.create_service(LAND, 'land2', self.land_callback)

        # Spinning
        self.timer_period = 0.01  # seconds
        self.create_timer(self.timer_period, self.publish_position)
        self.path_planning_timer = None  # path planning not start before takeoff

        # Variable
        self.drone_num = 2
        self.position = POS()  # ego drone pos
        self.other_drones_positions = {}  # other drone pos
        self.goal_position = np.array([goal_x, goal_y, goal_z])  # value input from sys
        self.landing_threshold = 1.0
        self.force = 5

    # for Publisher Node
    def publish_position(self):
        location = self.vehicle.location.global_relative_frame
        self.position.n = self.drone_num
        self.position.x = location.lat
        self.position.y = location.lon
        self.position.z = location.alt
        self.publisher.publish(self.position)

    # for Subscriber Node
    def subscribe_position(self, msg):
        drone_id = int(msg.data[0])
        x = msg.data[1]
        y = msg.data[2]
        z = msg.data[3]

        if drone_id == self.drone_num:
            self.position.x = x
            self.position.y = y
            self.position.z = z
        else:
            self.other_drones_positions[drone_id] = np.array([x, y, 5])

    # path_planning Sub Function1
    def calculate_position(self):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        current_lat = self.vehicle.location.global_relative_frame.lat
        current_lon = self.vehicle.location.global_relative_frame.lon
        current_alt = self.vehicle.location.global_relative_frame.alt

        # north : x-axis, west : y-axis
        x = (current_lat - self.base_lat) * LATITUDE_CONVERSION
        y = (self.base_lon - current_lon) * LONGITUDE_CONVERSION

        return np.array([x, y, current_alt])

    # path_planning Sub Function2
    def start_path_planning(self):
        if self.path_planning_timer is None:
            self.path_planning_timer = self.create_timer(self.timer_period, self.path_planning)
            print("Path planning started")

    # path_planning Sub Function3
    def goto(self, x, y, z, speed=10):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        target_lat = self.base_lat + (x / LATITUDE_CONVERSION)
        target_lon = self.base_lon - (y / LONGITUDE_CONVERSION)
        target_alt = 6
        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.1)
        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
        self.vehicle.groundspeed = speed
        self.vehicle.simple_goto(target_location)
        print("Moving to: ", x, y, 6)

    # Main Function (APF)
    def path_planning(self):
        # check distance for landing
        current_position = self.calculate_position()
        distance_to_goal = np.linalg.norm(current_position[:2] - self.goal_position[:2])

        if distance_to_goal <= self.landing_threshold:  # landing condition
            # Stop the path planning timer
            if self.path_planning_timer is not None:
                self.path_planning_timer.cancel()
                self.path_planning_timer = None
                print("Path planning timer stopped.")

            print("Goal reached. Initiating landing sequence.")
            self.land()
        else:
            env = APFEnvironment(current_position)
            next_position = current_position + np.array(
                env.apf(goal=self.goal_position, obs_pos=self.other_drones_positions)) * self.force
            self.goto(next_position[0], next_position[1], 6)

    # takeoff function
    def takeoff(self, h):
        self.vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(0.5)
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        cmds.clear()
        takeoff_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, h)
        cmds.add(takeoff_cmd)
        cmds.upload()
        time.sleep(0.5)  # upload wait
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(3)
        print("ARMED : ", self.vehicle.armed)
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system, self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0,
            0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(2)
        print("Mission started")

        while True:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= h * 0.8:
                print("Reached target altitude!!!!!!!!!!!!!!!!!!!!")
                break
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(0.1)
        print("Start path planning")
        self.start_path_planning()  # path planning start after takeoff

    # takeoff function callback
    def takeoff_callback(self, request, response):
        try:
            h = request.altitude
            self.takeoff(h)
            response.success = True
            response.message = f"Vehicle has taken off to {h} altitude"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # land function
    def land(self):
        print("Initiating landing sequence")
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system, self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0, 0)
        while self.vehicle.location.global_relative_frame.alt > 0.3:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            time.sleep(1)
        print("Landed successfully!!!")

        # Stop the path planning timer
        if self.path_planning_timer is not None:
            self.path_planning_timer.cancel()
            self.path_planning_timer = None
            print("Path planning timer stopped.")

    # land function callback
    def land_callback(self, request, response):
        try:
            self.land()
            response.success = True
            response.message = "Vehicle has landed successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # close
    def close_connection(self):
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
            print("Vehicle connection closed.")


def main(args=None):
    rclpy.init(args=args)

    # get variable from command system
    args = sys.argv[1:]
    if len(args) != 3:
        print("Usage: ros2 run drone_package drone_node2 <goal_x> <goal_y> <goal_z>")
        return

    goal_x = float(args[0])
    goal_y = float(args[1])
    goal_z = float(args[2])

    drone2 = DroneNode2(goal_x, goal_y, goal_z)
    try:
        rclpy.spin(drone2)
    finally:
        drone2.close_connection()
        drone2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
