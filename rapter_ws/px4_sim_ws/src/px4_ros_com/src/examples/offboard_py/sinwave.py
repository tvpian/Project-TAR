#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time
import numpy as np

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
        self.RATE = 40  # loop rate in Hz
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
        self.init_trajectory()
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50 milliseconds

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def init_trajectory(self):
        
        # Circle parameters
        R = 3.0  # Radius of the circle
        omega = 2.0  # Frequency of the sinusoidal modulation
        z_amplitude = 0.7  # Amplitude of the sinusoidal modulation in z-direction
        z_offset = -3.0  # Offset of the sinusoidal modulation in z-direction

        # Time step for each increment in the loop
        dt = 2.0 * np.pi / self.STEPS

        # Assuming you have already defined the TrajectorySetpoint class

        self.path = [TrajectorySetpoint() for _ in range(self.STEPS)]

        for i in range(self.STEPS):
            theta = i * dt

            # Position
            self.path[i].position = [R * np.cos(theta), R * np.sin(theta), z_offset + z_amplitude * np.sin(omega * theta)]

            # Velocity
            self.path[i].velocity = [-R * np.sin(theta), R * np.cos(theta), z_amplitude * omega * np.cos(omega * theta)]

            # Acceleration
            self.path[i].acceleration = [-R * np.cos(theta), -R * np.sin(theta), -z_amplitude * omega**2 * np.sin(omega * theta)]

            # Yaw (assuming it faces towards the center of the circle)
            self.path[i].yaw = float(np.arctan2(-self.path[i].position[1], -self.path[i].position[0]))

        # Calculate yaw_rate by dirty differentiating yaw
        for i in range(self.STEPS - 1):
            next_yaw = float(self.path[(i + 2) % self.STEPS].yaw)
            curr_yaw = float(self.path[i].yaw)
            # account for wrap around +- pi
            if (next_yaw - curr_yaw) < -np.pi:
                next_yaw += 2 * np.pi
            elif (next_yaw - curr_yaw) > np.pi:
                next_yaw -= 2 * np.pi
            self.path[i].yawspeed = (next_yaw - curr_yaw) / dt

        # Set yaw speed for the last element
        self.path[self.STEPS - 1].yawspeed = self.path[0].yawspeed  # or some other appropriate value
        print("Trajectory Created\n")



    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.vehicle_status.arming_state == 2 and self.vehicle_status.nav_state == 14:
            # Increment the setpoint counter
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter >= self.STEPS:
                self.offboard_setpoint_counter = 0

            # Set the timestamp in microseconds (converted from nanoseconds)
            self.path[self.offboard_setpoint_counter].timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # Publish the trajectory setpoint
            self.trajectory_setpoint_publisher.publish(self.path[self.offboard_setpoint_counter])

        else:
            print("\rWaiting for the Drone to be Armed", end = '')



def main(args=None) -> None:
    print('Starting offboard control node...')
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