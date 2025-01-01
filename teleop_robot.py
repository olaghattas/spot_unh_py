

import argparse
import os
import sys
import time
import logging

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration
from bosdyn.client.estop import EstopClient
from bosdyn.client.power import PowerClient

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Error as LeaseBaseError
LOGGER = logging.getLogger() ## shufe keef t2emiya
# import textwrap
# # Print controls
# print(
#     textwrap.dedent("""\
# | Button Combination | Functionality            |
# |--------------------|--------------------------|
# | A                  | Walk                     |
# | B                  | Stand                    |
# | X                  | Sit                      |
# | Y                  | Stairs                   |
# | LB + :             |                          |
# | - D-pad up/down    | Walk height              |
# | - D-pad left       | Battery-Change Pose      |
# | - D-pad right      | Self right               |
# | - Y                | Jog                      |
# | - A                | Amble                    |
# | - B                | Crawl                    |
# | - X                | Hop                      |
# |                    |                          |
# | If Stand Mode      |                          |
# | - Left Stick       |                          |
# | -- X               | Rotate body in roll axis |
# | -- Y               | Control height           |
# | - Right Stick      |                          |
# | -- X               | Turn body in yaw axis    |
# | -- Y               | Turn body in pitch axis  |
# | Else               |                          |
# | - Left Stick       | Move                     |
# | - Right Stick      | Turn                     |
# |                    |                          |
# | LB + RB + B        | E-Stop                   |
# | Start              | Motor power & Control    |
# | Back               | Exit                     |
#         """))


class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        # Create a subscription to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10  # QoS depth
        )
        self.get_logger().info("Joy Subscriber Node has been started.")
        self.stand = False
        self.sit = False

    def joy_callback(self, msg):
        # Callback triggered on receiving a message
        msg_ = msg
        # self.get_logger().info(f"Axes: {msg.axes}, Buttons: {msg.buttons}")
        # if LT pressed stand
        if msg_.axes[2] < -0.5:
            self.stand = True
            self.sit = False

        if msg_.axes[5] < -0.5:
            self.stand = False
            self.sit = True

        print("stand = " , self.stand)


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()

class TeleopInterface:
    def __init__(self,robot ):
        self.robot = robot
        self.lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self.robot_state_task = AsyncRobotState(self.robot_state_client)

        assert not self.robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                        'such as the estop SDK example, to configure E-Stop.'

        # Stuff that is set in start()

        self.lease_keepalive = None
        self._estop_keepalive = None

        self.node = JoySubscriber()

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self.lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True,
                                               return_at_exit=True)

        self.robot.logger.info('Powering on robot... This may take several seconds.')
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), 'Robot power on failed.'
        self.robot.logger.info('Robot powered on.')
        # if self._estop_endpoint is not None:
        #     self._estop_endpoint.force_simple_setup(
        #     )  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info('Shutting down ArmWasdInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self.lease_keepalive:
            self.lease_keepalive.shutdown()

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (bosdyn.client.ResponseError, bosdyn.client.RpcError, LeaseBaseError) as err:
            print("failed: ", err)
            return None
    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                     end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self.robot_state_task.proto

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def power_off(self):
        self.robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self.robot.is_powered_on(), 'Robot power off failed.'
        self.robot.logger.info('Robot safely powered off.')

    def teleop_spot(self):
        print("TELEOP")
        try:
            while rclpy.ok():
                print(self.node.stand)
                rclpy.spin_once(self.node, timeout_sec=0.1)  # Non-blocking spin
                if self.node.stand:
                    self._stand()
                    self.node.stand = False  # Reset to avoid repeated calls
                elif self.node.sit:
                    self._sit()
                    self.node.sit = False  # Reset to avoid repeated calls
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()
            self._safe_power_off()
            time.sleep(2.0)


def main():
    # Initialize rclpy
    rclpy.init()
    """Command line interface."""
    parser = argparse.ArgumentParser()
    # bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('-v', '--verbose', action='store_true', help='Print debug-level messages')
    parser.add_argument('--time-sync-interval-sec',
                        help='The interval (seconds) that time-sync estimate should be updated.',
                        type=float)
    options = parser.parse_args()

    bosdyn.client.util.setup_logging(options.verbose)
    # Create robot object.
    sdk = bosdyn.client.create_standard_sdk('TeleopClient')
    robot = sdk.create_robot(os.environ.get('SPOT_IP'))
    try:
        bosdyn.client.util.authenticate(robot)
        robot.start_time_sync(options.time_sync_interval_sec)
    except bosdyn.client.RpcError as err:
        robot.logger.error('Failed to communicate with robot: %s', err)
        return False

    teleop_spot = TeleopInterface(robot)
    try:
        teleop_spot.start()
    except (bosdyn.client.ResponseError, bosdyn.client.RpcError) as err:
        LOGGER.error('Failed to initialize robot communication: %s', err)
        return False

    try:
        print("&&& TELEOP")
        teleop_spot.teleop_spot()
    except Exception as e:
        robot.logger.error('Teleop has thrown an error: %s', e)
    finally:
        # Do any final cleanup steps.
        teleop_spot.power_off()



if __name__ == '__main__':
    if not main():
        sys.exit(1)