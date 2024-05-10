import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time
import threading
import numpy as np
 
offboard_control = OffboardControl()


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""
    
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Constants
        self.FLIGHT_ALTITUDE = -1.5
        self.RATE = 20  # loop rate in Hz
        self.CYCLE_S = 20  # Duration of one cycle in seconds
        self.STEPS = self.CYCLE_S * self.RATE


        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)


        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.timer = self.create_timer(0.05, self.publish_offboard_control_heartbeat_signal)  #50 milliseconds
        print("Heartbeat Started\n")

        # Starting the user input thread
        self.input_thread = threading.Thread(target=self.handle_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.new_input_event = threading.Event()

        # Current target position initialized to None
        # self.current_target = [0.0, 0.0, 0.0]

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        """Callback function for the timer."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        # print("Heartbeat sent\n")

    def handle_user_input(self):
        while True:
            # setpoint.velocity = [0.0, 0.0, 0.0]  # Assuming the drone should stop at the target
            # setpoint.acceleration = [0.0, 0.0, 0.0]
            if self.vehicle_status.arming_state == 2 and self.vehicle_status.nav_state == 14:
                x = float(input("\nEnter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                z = -1

                self.current_target = np.array([x, y, z])
                # self.new_input_event.set()
                self.update_trajectory()

            else: 
                print("\rWaiting for the Drone to switch to offboard node", end = " ")



    def update_trajectory(self):
        if rclpy.ok():
            target_position = self.current_target
            # self.new_input_event.clear()

            # Publish the trajectory setpoint
            setpoint = TrajectorySetpoint()
            setpoint.position = target_position.tolist()
            # setpoint.velocity = [0.0, 0.0, 0.0]  # Assuming the drone should stop at the target
            # setpoint.acceleration = [0.0, 0.0, 0.0]
            self.trajectory_setpoint_publisher.publish(setpoint)

            print(f"Updated drone target to: {target_position}")

def main(args=None) -> None:
    print('Starting offboard control node...\n')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # main()
    try:
        main()
    except Exception as e:
        print(e)