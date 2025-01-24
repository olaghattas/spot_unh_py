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
from bosdyn.util import duration_to_seconds


#### BLT are empty
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
        
    def print_feedback(self, feedback_resp, logger):
        """ Helper function to query for ArmJointMove feedback, and print it to the console.
            Returns the time_to_goal value reported in the feedback """
        joint_move_feedback = feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
        logger.info(f'  planner_status = {joint_move_feedback.planner_status}')
        logger.info(
            f'  time_to_goal = {duration_to_seconds(joint_move_feedback.time_to_goal):.2f} seconds.')

        # Query planned_points to determine target pose of arm
        logger.info('  planned_points:')
        for idx, points in enumerate(joint_move_feedback.planned_points):
            pos = points.position
            pos_str = f'sh0 = {pos.sh0.value:.3f}, sh1 = {pos.sh1.value:.3f}, el0 = {pos.el0.value:.3f}, ' \
                    f'el1 = {pos.el1.value:.3f}, wr0 = {pos.wr0.value:.3f}, wr1 = {pos.wr1.value:.3f}'
            logger.info(f'    {idx}: {pos_str}')
        return duration_to_seconds(joint_move_feedback.time_to_goal)

 
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
    
    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())


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

    def _toggle_gripper_open(self):
        self._start_robot_command('open_gripper', RobotCommandBuilder.claw_gripper_open_command())

    def _toggle_gripper_closed(self):
        self._start_robot_command('close_gripper', RobotCommandBuilder.claw_gripper_close_command())

    def _unstow(self):
        self._start_robot_command('unstow', RobotCommandBuilder.arm_ready_command())
    
    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())
    
    
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
         
    def make_robot_command(self, arm_joint_traj):
        """ Helper function to create a RobotCommand from an ArmJointTrajectory.
            The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
            filled out to follow the passed in trajectory. """

        joint_move_command = bosdyn.api.arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
        arm_command = bosdyn.api.arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
        sync_arm = bosdyn.api.synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
        arm_sync_robot_cmd = bosdyn.api.robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
        return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

  
    def move_arm(self, sh0, sh1, el0, el1, wr0, wr1):
        
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1)
        arm_joint_traj = bosdyn.api.arm_command_pb2.ArmJointTrajectory(points=[traj_point])
        # Make a RobotCommand
        command = self.make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = self._robot_command_client.robot_command(command)
        self.robot.logger.info('Moving arm to position 1.')

        # Query for feedback to determine how long the goto will take.
        feedback_resp = self._robot_command_client.robot_command_feedback(cmd_id)
        self.robot.logger.info('Feedback for Example 1: single point goto')
        time_to_goal = self.print_feedback(feedback_resp, self.robot.logger)
        time.sleep(time_to_goal)
        
    def replay_actions(self):
     
        # fl.hx
        # fl.hy
        # fl.kn
        # fr.hx
        # fr.hy
        # fr.kn
        # hl.hx
        # hl.hy
        # hl.kn
        # hr.hx
        # hr.hy
        # hr.kn

        # arm0.sh0
        # arm0.sh1
        # arm0.el0
        # arm0.el1
        # arm0.wr0
        # arm0.wr1
        # arm0.f1x

        '''
        read npz action file
        '''
        demo_folder = "/home/olagh/Desktop/gripper_fixed_trial_demo_20250124_165300"
        joint_states_list = np.load(demo_folder + "/testing_demo_joint_states.npz")
        gripper_states_list = np.load(demo_folder + "/testing_demo_gripper_states.npz")
        states = joint_states_list["data"]
        gripper_states= gripper_states_list["data"]
        len_ = len(states)
        assert len(states) == len(gripper_states)
        # for i in range(1):
        for i in range(len_):
            print("GRIPPERSTATES: ", gripper_states[i])
            sh0 = states[i][12]
            sh1 = states[i][13]
            el0 = states[i][14]
            el1 = states[i][15]
            wr0 = states[i][16]
            wr1 = states[i][17]
            
            self.move_arm( sh0, sh1, el0, el1, wr0, wr1)
            
            if gripper_states[i] == 1:
                self._toggle_gripper_open()
            else:
                self._toggle_gripper_closed()
            print( "action", sh0, sh1, el0, el1, wr0, wr1)
            
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