#!/usr/bin/env python

"""
Control Carla ego vehicle by using AckermannDrive messages from Mache's dbw 
"""

import sys

import numpy

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from ackermann_msgs.msg import AckermannDrive  
from std_msgs.msg import Header 
from carla_msgs.msg import CarlaEgoVehicleControl
from dbw_ford_msgs.msg import BrakeReport 
from dbw_ford_msgs.msg import ThrottleReport 
from autoware_auto_vehicle_msgs.msg import SteeringReport 
from carla_ackermann_msgs.msg import EgoVehicleControlInfo 

ROS_VERSION = roscomp.get_ros_version()

if ROS_VERSION == 1:
    from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig # pylint: disable=no-name-in-module,import-error,ungrouped-imports
    from dynamic_reconfigure.server import Server # pylint: disable=no-name-in-module,import-error
if ROS_VERSION == 2:
    from rcl_interfaces.msg import SetParametersResult


class CarlaAckermannControl(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(CarlaAckermannControl, self).__init__("carla_ackermann_control")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.02)
        self.last_ackermann_msg_received_sec =  self.get_time()
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.throttle_pedal = 0.0
        self.brake_pedal = 0.0
        self.steering_angle = 0.0

        # control info
        self.info = EgoVehicleControlInfo()
        self.info.restrictions.max_steering_angle = 0.6

         # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = False

        # ackermann drive commands
        self.throttle_subscriber = self.new_subscription(
            ThrottleReport,
            "/vehicle/throttle_report",
            self.throttle_command_updated,
            qos_profile=10
        )

        # current status of the vehicle
        self.brake_subscriber = self.new_subscription(
            BrakeReport,
            "/vehicle/brake_report",
            self.brake_command_updated,
            qos_profile=10
        )

        # vehicle info
        self.steering_subscriber = self.new_subscription(
            SteeringReport,
            "/vehicle/status/steering_report",
            self.steering_command_updated,
            qos_profile=10
        )

        # to send command to carla
        self.carla_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            qos_profile=1)

        # report controller info
        self.control_info_publisher = self.new_publisher(
            EgoVehicleControlInfo,
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            qos_profile=1)

    def get_msg_header(self):
        """
        Get a filled ROS message header
        :return: ROS message header
        :rtype: std_msgs.msg.Header
        """
        header = Header()
        header.frame_id = "map"
        header.stamp = roscomp.ros_timestamp(sec=self.get_time(), from_sec=True)
        return header

    def throttle_command_updated(self, throttle_dbw):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.throttle_pedal = throttle_dbw.pedal_output

    def brake_command_updated(self, brake_dbw):
        """
        Get the brake pedal from dataspeed dbw
        """
        self.brake_pedal = brake_dbw.pedal_output


    def steering_command_updated(self, steering_autoware):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.steering_angle = steering_autoware.steering_tire_angle

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # perform actual control
        self.control_steering()
        #self.control_stop_and_reverse()
        if not self.info.output.hand_brake:
            self.update_drive_vehicle_control_command()

            # only send out the Carla Control Command if AckermannDrive messages are
            # received in the last second (e.g. to allows manually controlling the vehicle)
            if (self.last_ackermann_msg_received_sec + 1.0) > \
                    self.get_time():
                self.info.output.header = self.get_msg_header()
                self.carla_control_publisher.publish(self.info.output)

    def control_steering(self):
        """
        Basic steering control
        """
        self.info.output.steer = self.steering_angle / \
            self.info.restrictions.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        # from this velocity on it is allowed to switch to reverse gear
        standing_still_epsilon = 0.1
        # from this velocity on hand brake is turned on
        full_stop_epsilon = 0.00001

        # auto-control of hand-brake and reverse gear
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.status.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    self.loginfo(
                        "VehicleControl: Change of driving direction to reverse")
                    self.info.output.reverse = True
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    self.loginfo(
                        "VehicleControl: Change of driving direction to forward")
                    self.info.output.reverse = False
            if self.info.target.speed_abs < full_stop_epsilon:
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.current.speed = 0.
                self.info.current.speed_abs = 0.
                self.info.current.accel = 0.
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            self.loginfo("VehicleControl: Request change of driving direction."
                         " v_current={} v_desired={}"
                         " Set desired speed to 0".format(self.info.current.speed,
                                                          self.info.target.speed))
            self.set_target_speed(0.)


    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        self.info.output.throttle = self.throttle_pedal
        self.info.outout.brake = self.brake_pedal

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.control_info_publisher.publish(self.info)

    def run(self):
        """

        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.vehicle_control_cycle()
            self.send_ego_vehicle_control_info_msg()

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("carla_ackermann_control", args=args)

    try:
        controller = CarlaAckermannControl()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()