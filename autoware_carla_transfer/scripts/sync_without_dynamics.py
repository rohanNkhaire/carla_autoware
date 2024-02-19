#!/usr/bin/env python3

"""
Control Carla ego vehicle by using AckermannDrive messages from Mache's dbw 
"""

import sys
import logging
import numpy
import carla
import time

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_ackermann_control import carla_control_physics as phys

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose

class NoDynamicsRelay(CompatibleNode):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        super(NoDynamicsRelay, self).__init__("no_dynamics_relay")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.02)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.vehicle = None
        self.ego_actor = None
        self.actor_list = []

        # variables init
        self.veh_velocity = 0.0
        self.ang_velocity = 0.0
        self.veh_pose = Pose()

        # subscribe to vehicle twist
        self.twist_subscriber = self.new_subscription(
            TwistStamped,
            "/vehicle/twist",
            self.twist_updated,
            qos_profile=10
        )

        # subscribe to vehicle pose
        self.twist_subscriber = self.new_subscription(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            self.pose_updated,
            qos_profile=10
        )

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

    def twist_updated(self, veh_twist):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.veh_velocity = veh_twist.twist.linear.x
        self.ang_velocity = veh_twist.twist.angular.z

    def pose_updated(self, ego_pose):
        """
        Get the throttle pedal from dataspeed dbw
        """
        self.veh_pose = ego_pose.pose.pose

    def vehicle_relay_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        '''
        # Relay velocity
        velocity = carla.Vector3D()
        velocity.x = self.veh_velocity
        velocity.y = 0.0
        velocity.z = 0.0
        self.vehicle.set_target_velocity(velocity)
        print(self.veh_velocity)
        '''

        # Relay Pose
        ego_pose = trans.ros_pose_to_carla_transform(self.veh_pose)
        self.vehicle.set_transform(ego_pose)

    def run(self):
        """

        Control loop

        :return:
        """
        # Get the client
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(2000.0)

            world = client.get_world()
        except:
            pass
        # Set the ego vehicle
        while len(self.actor_list) == 0:
            logging.warning("Searching Actors in the world") 
            self.actor_list = world.get_actors()     
            time.sleep(2.0)

        for actor in self.actor_list:
            if actor.type_id != "spectator":
                if actor.attributes["role_name"] == "ego_vehicle":
                    self.vehicle = self.actor_list.find(actor.id)
                    self.ego_actor = actor   

        logging.warning("Found ego-vehicle")            
        self.vehicle.set_enable_gravity(False)
        def loop(timer_event=None):
            self.vehicle_relay_cycle()

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """

    main function

    :return:
    """
    roscomp.init("no_dynamics_relay", args=args)

    try:
        controller = NoDynamicsRelay()
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
