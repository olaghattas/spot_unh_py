import argparse
import os
import sys
import time
import logging

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.power import PowerClient

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Error as LeaseBaseError
import numpy as np



####  and BLT are empty
COMMAND_INPUT_RATE = 0.1
VELOCITY_CMD_DURATION = VELOCITY_CMD_DURATION_ARM = 0.2  # seconds 0.6 default
# VELOCITY_CMD_DURATION_ARM = 0.5 ## as in wasd

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_HAND_NORMALIZED = 0.5  # normalized hand velocity [0,1]
VELOCITY_ANGULAR_HAND = 1.0  # rad/sec

HEIGHT_MAX = 0.3  # m
HEIGHT_CHANGE = 0.1  # m per command

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, logging.getLogger(),
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()

class SpotInterface:
    def __init__(self,robot ):
        self.mobility_params = None
        self.body_height = 0.0
        
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
 
    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        ## force taking lease
        self.lease_client.take()
        self.lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client, must_acquire=True,
                                               return_at_exit=True)

        self.robot.logger.info('Powering on robot... This may take several seconds.')
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), 'Robot power on failed.'
        self.robot.logger.info('Robot powered on.')
        
        #walk
        self.mobility_params = spot_command_pb2.MobilityParams(
            locomotion_hint= spot_command_pb2.HINT_SPEED_SELECT_TROT,stair_hint=0)
        # if self._estop_endpoint is not None:
        #     self._estop_endpoint.force_simple_setup(
        #     )  # Set this endpoint as the robot's sole estop.
        ## stand the robot and unstow arm
        bosdyn.client.robot_command.blocking_stand(self._robot_command_client)
        # bosdyn.client.robot_command.blocking_stand(self._robot_command_client)
        self._unstow()
        time.sleep(10.0) 
        

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        logging.getLogger().info('Shutting down ArmWasdInterface.')
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

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def power_off(self):
        self.robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not self.robot.is_powered_on(), 'Robot power off failed.'
        self.robot.logger.info('Robot safely powered off.')

    ## FOR BODY  
    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self.mobility_params = RobotCommandBuilder.mobility_params(
            body_height=self.body_height, locomotion_hint=self.mobility_params.locomotion_hint,
            stair_hint=self.mobility_params.stair_hint)
        
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=self.mobility_params),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)
        
    
    def _change_height(self, direction):
        """Changes robot body height.

        Args:
            direction: 1 to increase height, -1 to decrease height.
        """

        self.body_height = self.body_height + direction * HEIGHT_CHANGE
        self.body_height = min(HEIGHT_MAX, self.body_height)
        self.body_height = max(-HEIGHT_MAX, self.body_height)
        self._velocity_cmd_helper()
        
    ## For ARM
    def _arm_full_velocity_cmd_helper(self, desc='', v_r = 0.0, v_theta = 0.0, v_z = 0.0 , v_rx=0.0, v_ry=0.0, v_rz=0.0):
        """ Helper function to build an arm velocity command from angular velocities measured with respect
            to the odom frame, expressed in the hand frame.

        params:
        + desc: string description of the desired command
        + v_rx: angular velocity about X-axis in units rad/sec
        + v_ry: angular velocity about Y-axis in units rad/sec
        + v_rz: angular velocity about Z-axis in units rad/sec

        """
        # Specify a zero linear velocity of the hand. This can either be in a cylindrical or Cartesian coordinate system.
        cylindrical_velocity = bosdyn.api.arm_command_pb2.ArmVelocityCommand.CylindricalVelocity()
        cylindrical_velocity.linear_velocity.r = v_r
        cylindrical_velocity.linear_velocity.theta = v_theta
        cylindrical_velocity.linear_velocity.z = v_z
        
        # Build the angular velocity command of the hand
        angular_velocity_of_hand_rt_odom_in_hand = bosdyn.api.geometry_pb2.Vec3(x=v_rx, y=v_ry, z=v_rz)

        arm_velocity_command = bosdyn.api.arm_command_pb2.ArmVelocityCommand.Request(
            cylindrical_velocity=cylindrical_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular_velocity_of_hand_rt_odom_in_hand,
            end_time=self.robot.time_sync.robot_timestamp_from_local_secs(time.time() +
                                                                           VELOCITY_CMD_DURATION_ARM))

        self._arm_velocity_cmd_helper(arm_velocity_command=arm_velocity_command, desc=desc)


    def _arm_velocity_cmd_helper(self, arm_velocity_command, desc=''):

        # Build synchronized robot command
        robot_command = bosdyn.api.robot_command_pb2.RobotCommand()
        robot_command.synchronized_command.arm_command.arm_velocity_command.CopyFrom(
            arm_velocity_command)

        self._start_robot_command(desc, robot_command,
                                  end_time_secs=time.time() + VELOCITY_CMD_DURATION_ARM)
    
    
    def _toggle_gripper_open(self):
        self._start_robot_command('open_gripper', RobotCommandBuilder.claw_gripper_open_command())

    def _toggle_gripper_closed(self):
        self._start_robot_command('close_gripper', RobotCommandBuilder.claw_gripper_close_command())

    def _unstow(self):
        self._start_robot_command('unstow', RobotCommandBuilder.arm_ready_command())
    
    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())
    
    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None  
                  
    def replay_actions(self):
     
        # self.action = [v_x_, v_y_, v_rot_, v_r_, v_theta_, v_z_, v_rx_, v_ry_, v_rz_]

        '''
        read npz action file
        '''
        demo_folder = "/home/olagh/Desktop/trial_demo_controlled2"
        action_list = np.load(demo_folder + "/testing_demo_action.npz")
        gripper_states_list = np.load(demo_folder + "/testing_demo_gripper_states.npz")
        actions = action_list["data"]
        gripper_states= gripper_states_list["data"]
        len_ = len(actions)
        assert len(actions) == len(gripper_states)
        for i in range(len_):
            
            v_x_, v_y_, v_rot_, v_r_, v_theta_, v_z_, v_rx_, v_ry_, v_rz_ = actions[i]
            
            if gripper_states[i] == 1:
                self._toggle_gripper_open()
            else:
                self._toggle_gripper_closed()
            print( "action",v_x_, v_y_, v_rot_, v_r_, v_theta_, v_z_, v_rx_, v_ry_, v_rz_)
            self._velocity_cmd_helper('move body', v_x=v_x_ , v_y=v_y_, v_rot=v_rot_)
            self._arm_full_velocity_cmd_helper('EndEff Rotation', v_r = v_r_, v_theta = v_theta_, v_z = v_z_ , v_rx=v_rx_, v_ry=v_ry_, v_rz=v_rz_)
            time.sleep(0.4615)
        self._stow()
        time.sleep(2.0)
        self._safe_power_off()
        time.sleep(2.0)


def main():

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

    spot_interface = SpotInterface(robot)
    try:
        spot_interface.start()
    except (bosdyn.client.ResponseError, bosdyn.client.RpcError) as err:
        logging.getLogger().error('Failed to initialize robot communication: %s', err)
        return False

    try:
        print("Action interface")
        spot_interface.replay_actions()
    except Exception as e:
        robot.logger.error('Teleop has thrown an error: %s', e)
    finally:
        # Do any final cleanup steps.
        spot_interface.power_off()


if __name__ == '__main__':
    if not main():
        sys.exit(1)