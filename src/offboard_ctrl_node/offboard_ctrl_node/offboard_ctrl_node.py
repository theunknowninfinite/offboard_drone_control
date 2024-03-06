#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time

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

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish control commands
        # self.timer = self.create_timer(0.008047, self.timer_callback)
        self.timer = self.create_timer(0.008047, self.timer_callback) #125hz , to match frequency of vehicle_local_position topic
        self.seq_state = 0
        self.target_point = [0.0, 0.0, -10.0]  # Initial target waypoint
        self.sequence_done=False #flag to check if sequence is done

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def near_way(self, tol=0.5):
        """Check if the drone is near the current waypoint."""
        current_pos = [self.vehicle_local_position.x,
                            self.vehicle_local_position.y,
                            self.vehicle_local_position.z]
        # euclidean distance
        dist = sum((cp - tw)**2 for cp, tw in zip(current_pos, self.target_point))**0.5
        return dist < tol
    
    def timer_callback(self):
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter <11:
            self.offboard_setpoint_counter += 1
        
        #Engaging offboard mode and publishing initial height of 10M
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            # self.arm()
            self.publish_position_setpoint(*self.target_point)
        
        #making sure drone near the waypoint before moving to next waypoint
        # if self.near_way() and self.sequence_done==False and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        if self.near_way() and self.sequence_done==False:

            if self.seq_state == 0 and self.offboard_setpoint_counter>500:
                self.target_point = [10.0, 0.0, -10.0]  # 10M foward
                self.seq_state = 1
                self.offboard_setpoint_counter=11

            elif self.seq_state == 1 and self.offboard_setpoint_counter>1000:
                self.target_point = [10.0, 0.0, -25.0]  # 15M Up
                self.seq_state = 2
                self.offboard_setpoint_counter=11

            elif self.seq_state == 2 and self.offboard_setpoint_counter>1000:
                self.target_point = [10.0, -5.0, -25.0]   # 5M Left
                self.seq_state = 3
                self.offboard_setpoint_counter=11

            elif self.seq_state == 3 and self.offboard_setpoint_counter>1000:
                self.target_point = [0.0, 0.0, -10.0]   # Back to Home Postion 
                # self.seq_state = 0
                self.sequence_done=True # Sequence Done 
                self.offboard_setpoint_counter=11

            self.offboard_setpoint_counter += 1
            print(self.offboard_setpoint_counter )

        # non functional since counter does not increment outside of the if loop 
        # if self.sequence_done==True and self.seq_state==3 and self.offboard_setpoint_counter>100:
        #     exit(0)
        #publishing waypoints 
        self.publish_position_setpoint(*self.target_point)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
