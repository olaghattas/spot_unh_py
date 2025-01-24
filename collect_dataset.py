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
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.image import ImageClient

from xbox import JoySubscriber
import csv
def sign(x):
    return (x > 0) - (x < 0)


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

class TeleopInterface:
    def __init__(self,robot ):
        self.mobility_params = None
        self.body_height = 0.0
        
        self.robot = robot
        self.lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        self.robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._docking_client = robot.ensure_client(DockingClient.default_service_name)
        
        self.robot_state_task = AsyncRobotState(self.robot_state_client)

        assert not self.robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                        'such as the estop SDK example, to configure E-Stop.'

        # Stuff that is set in start()

        self.lease_keepalive = None
        self._estop_keepalive = None

        self.node = JoySubscriber()
        self.dock_id = 520
        # v_x_, v_y_, v_rot_, v_r_, v_theta_, v_z_, v_rx_, v_ry_, v_rz_
        self.action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # 0 closed 1 open
        self.gripper = 0 ## initially gripper closed
        
        ## added to track when we unstow becasue it easier to move with hand unstowed
        self.unstow = 0 # 1 when triggered
        self.start_ = False ## to start recording or use it as a flag
        self.csv_file = "step_times.csv"
        
        # Initialize CSV file with headers
        with open(self.csv_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Step", "Timestamp"])
        self.step_count = 1
        
    def xbox_to_command(self):
        buttons_pressed = self.node.buttons_pressed
        body = self.node.body
        end_eff = self.node.end_eff
        
        self.unstow = 0
        v_x_ = 0.0
        v_y_ = 0.0
        v_rot_ = 0.0

        v_r_ = 0.0
        v_theta_ = 0.0
        v_z_ = 0.0
        
        v_rx_ = 0.0
        v_ry_ = 0.0
        v_rz_ = 0.0
        
        if buttons_pressed == "ART":
            print("Start Dock")
            self._dock_undock()
            print("End Dock")
            time.sleep(2.0)  
            
        elif buttons_pressed == "ALT":
            print("Return/Acquire Lease")
            self._toggle_lease()
            time.sleep(2.0)
        
        elif buttons_pressed == "BRT":
            print("START FLAG")
            
            self.start_ = not self.start_
            print(self.start_)
            time.sleep(2.0)
            
        elif buttons_pressed == "BLT":
            print(" Nothing")
            time.sleep(2.0)
            
        elif buttons_pressed == "ARB":
            self._stow()
            time.sleep(2.0)
            
        elif buttons_pressed == "BRB":
            print("Unstow")
            self._unstow()
            time.sleep(2.0)
        
        elif buttons_pressed == "ALB":
            print("Power On")
            self.robot.power_on(timeout_sec=20)
            time.sleep(2.0)
            
        elif buttons_pressed == "BLB":
            print("Power Off")
            self._safe_power_off()
            time.sleep(2.0)
            
        elif buttons_pressed == "A":
            v_z_ = -0.5 * VELOCITY_HAND_NORMALIZED
            # v_z_ = -1 * VELOCITY_HAND_NORMALIZED
            print("Go down")

        elif buttons_pressed == "B":
            v_rx_ = 0.5 * VELOCITY_HAND_NORMALIZED
            print("CW")

        elif buttons_pressed == "X":
            v_rx_ = -0.5 * VELOCITY_HAND_NORMALIZED
            print("CCW")
            
        elif buttons_pressed == "Y":
            v_z_ = 0.5 * VELOCITY_HAND_NORMALIZED
            # v_z_ = VELOCITY_HAND_NORMALIZED
            print("Go up")
        
        elif buttons_pressed == "LT":
            self._stand()
            print("Stand")
            time.sleep(2.0)
            
        elif buttons_pressed == "RT":
            self._sit()
            print("Sit")
            time.sleep(2.0)
            
        elif buttons_pressed == "RB":
            self._toggle_gripper_open()
            self.gripper = 1
            print("open gripper")

        elif buttons_pressed == "LB":
            self._toggle_gripper_closed()
            self.gripper = 0
            print("close gripper")
        
        if body == "Left":
            if buttons_pressed == "LTRT":
                ## the arm move in two directions 
                v_rot_ = VELOCITY_BASE_ANGULAR
                print("RTLeft")
            else:    
                v_y_ = VELOCITY_BASE_SPEED
                # v_theta_ = VELOCITY_BASE_SPEED
                print("Left")
            
        if body == "Right":
            if buttons_pressed == "LTRT":
                ## the arm move in two directions 
                v_rot_ = -VELOCITY_BASE_ANGULAR
                print("RT Right")
            else:    
                v_y_ = -VELOCITY_BASE_SPEED
                # v_theta_ = 
                print("Right")
            
        if body == "Up": ## forward
            if buttons_pressed == "LTRT":
                # move body up
                self._change_height(1)
            else: 
                v_x_ = VELOCITY_BASE_SPEED
                # v_r_ = 0.75*VELOCITY_BASE_SPEED
            print("Up")

        
        if body == "Down": ## backward
            if buttons_pressed == "LTRT":
                # move body up
                self._change_height(-1)
            else:
                v_x_ = -VELOCITY_BASE_SPEED
                # v_r_ = -0.75*VELOCITY_BASE_SPEED
            print("Down")
            
        body_ = [v_x_ , v_y_, v_rot_]
        
        if any(body_):
            print("MOVE ARM")
            print("end_eff", end_eff)
            self._velocity_cmd_helper('move body', v_x=v_x_ , v_y=v_y_, v_rot=v_rot_)
            # self._arm_full_velocity_cmd_helper('EndEff Rotation', v_r = v_r_, v_theta = v_theta_, v_z = v_z_ , v_rx=v_rx_, v_ry=v_ry_, v_rz=v_rz_)
        
        end_eff_2 = [v_z_, v_rx_]
        
        ## end_eff
            # mapping to xbox
            #[Right Stick Y, Right Stick X, Left Stick Y, Left Stick X]
            # mapping end eff position
            #[Transl forward/back,Transl left/right, Rotation forward/back,Rotation left/right]
            
        if any(end_eff) or any(end_eff_2):
            print("MOVE ARM")
            print("end_eff", end_eff)
            if end_eff[0]:
                v_r_ += end_eff[0] * VELOCITY_HAND_NORMALIZED
                # v_r_ = sign(end_eff[0]) * VELOCITY_HAND_NORMALIZED
   
            if end_eff[1]:
                v_theta_ += end_eff[1] * VELOCITY_HAND_NORMALIZED
                
            if end_eff[2]:
                v_ry_ += end_eff[2] * VELOCITY_ANGULAR_HAND
                
            if end_eff[3]:
                v_rz_ += -1 * end_eff[3] * VELOCITY_ANGULAR_HAND
                
            self._arm_full_velocity_cmd_helper('EndEff Rotation', v_r = v_r_, v_theta = v_theta_, v_z = v_z_ , v_rx=v_rx_, v_ry=v_ry_, v_rz=v_rz_)

        self.action = [v_x_, v_y_, v_rot_, v_r_, v_theta_, v_z_, v_rx_, v_ry_, v_rz_]
        if self.start:
            # Record the current time
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            
            # Save to the CSV file
            with open(self.csv_file, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([self.step_count, timestamp])
            self.step_count += 1
        
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

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def _dock(self):
        print("function dock")
        bosdyn.client.robot_command.blocking_stand(self._robot_command_client)
        blocking_dock_robot(self.robot, self.dock_id)

    def _dock_undock(self):
        dock_id = get_dock_id(self.robot)
        if dock_id is None:
            print('Robot does not seem to be docked;')
            self._dock()
        else:
            print(f'Docked at {dock_id}')
            blocking_undock(self.robot)
        
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
    def _arm_cylindrical_velocity_cmd_helper(self, desc='', v_r=0.0, v_theta=0.0, v_z=0.0):
        """ Helper function to build an arm velocity command from unitless cylindrical coordinates.

        params:
        + desc: string description of the desired command
        + v_r: normalized velocity in R-axis to move hand towards/away from shoulder in range [-1.0,1.0]
        + v_theta: normalized velocity in theta-axis to rotate hand clockwise/counter-clockwise around the shoulder in range [-1.0,1.0]
        + v_z: normalized velocity in Z-axis to raise/lower the hand in range [-1.0,1.0]

        """
        # Build the linear velocity command specified in a cylindrical coordinate system
        cylindrical_velocity = bosdyn.api.arm_command_pb2.ArmVelocityCommand.CylindricalVelocity()
        cylindrical_velocity.linear_velocity.r = v_r
        cylindrical_velocity.linear_velocity.theta = v_theta
        cylindrical_velocity.linear_velocity.z = v_z

        arm_velocity_command = bosdyn.api.arm_command_pb2.ArmVelocityCommand.Request(
            cylindrical_velocity=cylindrical_velocity,
            end_time=self.robot.time_sync.robot_timestamp_from_local_secs(time.time() +
                                                                           VELOCITY_CMD_DURATION_ARM))

        self._arm_velocity_cmd_helper(arm_velocity_command=arm_velocity_command, desc=desc)

    def _arm_angular_velocity_cmd_helper(self, desc='', v_rx=0.0, v_ry=0.0, v_rz=0.0):
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

        # Build the angular velocity command of the hand
        angular_velocity_of_hand_rt_odom_in_hand = bosdyn.api.geometry_pb2.Vec3(x=v_rx, y=v_ry, z=v_rz)

        arm_velocity_command = bosdyn.api.arm_command_pb2.ArmVelocityCommand.Request(
            cylindrical_velocity=cylindrical_velocity,
            angular_velocity_of_hand_rt_odom_in_hand=angular_velocity_of_hand_rt_odom_in_hand,
            end_time=self.robot.time_sync.robot_timestamp_from_local_secs(time.time() +
                                                                           VELOCITY_CMD_DURATION_ARM))

        self._arm_velocity_cmd_helper(arm_velocity_command=arm_velocity_command, desc=desc)

    
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
        
    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('unstow', RobotCommandBuilder.arm_ready_command())
    
    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = bosdyn.client.lease.LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None  
                  
    def _maybe_display_image(self, image, display_time=3.0):
        """Try to display image, if client has correct deps."""
        try:
            import io

            from PIL import Image
        except ImportError:
            logger = bosdyn.client.util.get_logger()
            logger.warning('Missing dependencies. Can\'t display image.')
            return
        try:
            image = Image.open(io.BytesIO(image.data))
            image.show()
            time.sleep(display_time)
        except Exception as exc:
            logger = bosdyn.client.util.get_logger()
            logger.warning('Exception thrown displaying image. %r', exc)


    def teleop_spot(self):
        import numpy as np
        print("TELEOP")
        
        from datetime import datetime
        try:
            # rate = self.node.create_rate(10)
            self.node.print_button_combination()
            
            # data = {"action": []}
            cameras = ["frontleft_fisheye_image", "frontright_fisheye_image", "hand_color_image"]
            
            # Generate a timestamp 
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            # Create the folder with the timestamp
            folder = f"/home/olagh/Desktop/trial_demo_{timestamp}"
            
            previous_state_dict = None
            start = False
            
            data = {"action": [], "joint_states": [], "gripper_states": []}
            for camera in cameras:
                data[f"camera_{camera}"] = []
                
            while rclpy.ok():
                # print("RCLPY")
                prev_start = start
                # print("prev_start" ,prev_start)
                rclpy.spin_once(self.node, timeout_sec=0.1)  # Non-blocking spin
                
                start = self.start_
    
                self.xbox_to_command()
                
                image_client = self.robot.ensure_client(ImageClient.default_service_name)
                sources = image_client.list_image_sources()
                # print("self.gripper", self.gripper)
                # print("start", start)
                if start:
                    
                    action = self.action
                    state = self.robot_state_client.get_robot_state() 
                    data["action"].append(action)
                    
                    joint_states = state.kinematic_state.joint_states
                    positions = np.array([joint.position.value for joint in joint_states])

                    data["gripper_states"].append(self.gripper)
                    state_dict = {
                        "joint_states": positions,
                        # "gripper_states": np.array(self.gripper),
                    }
                    
                    if previous_state_dict is not None:
                        for proprio_key in state_dict.keys():
                            proprio_state = state_dict[proprio_key]
                            if np.sum(np.abs(proprio_state)) <= 1e-6:
                                proprio_state = previous_state_dict[proprio_key]
                            state_dict[proprio_key] = np.copy(proprio_state)
                            
                    for proprio_key in state_dict.keys():
                        data[proprio_key].append(state_dict[proprio_key])

                    previous_state_dict = state_dict
                    
                    
                    for camera in cameras:
                        image = image_client.get_image_from_sources([camera])[0].shot.image
                        data[f"camera_{camera}"].append(image)
                    ## ERROR - Teleop has thrown an error: 'camera_frontleft_fisheye_image'


                # start turned from true to false signaling to stop recording
                if not start and prev_start:
                    # print("ksdahfkdgfkjhdsfghkjdkfvhdf")
                    os.makedirs(folder, exist_ok=True)
                
                    np.savez(f"{folder}/testing_demo_action", data=np.array(data["action"]))
                    # np.savez(f"{folder}/testing_demo_ee_states", data=np.array(data["ee_states"]))
                    np.savez(f"{folder}/testing_demo_joint_states", data=np.array(data["joint_states"]))
                    np.savez(f"{folder}/testing_demo_gripper_states",data=np.array(data["gripper_states"]))
                    for camera in cameras:
                        np.savez(
                            f"{folder}/testing_demo_camera_{camera}",
                            data=np.array(data[f"camera_{camera}"]),
                        )
                        
                # rate.sleep()
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
        logging.getLogger().error('Failed to initialize robot communication: %s', err)
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