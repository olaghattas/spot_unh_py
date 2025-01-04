"""Teleoperating robot arm to collect demonstration data adapted from deoxys franka"""

import argparse
import json
import os

import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from teleop_robot import TeleopInterface

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


from sensor_msgs.msg import Joy
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Error as LeaseBaseError







def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', action='store_true', help='Print debug-level messages')
    parser.add_argument('--time-sync-interval-sec',
                        help='The interval (seconds) that time-sync estimate should be updated.',
                        type=float)
    parser.add_argument("--folder", type=Path, default="/tmp/spot_demo_data")
    
    return parser.parse_args()


def main():
    rclpy.init()
    
    args = parse_args()
    args.folder.mkdir(parents=True, exist_ok=True)

    experiment_id = 0
    # Create a folder that saves the demonstration raw states.
    for path in args.folder.glob("run*"):
        if not path.is_dir():
            continue
        try:
            folder_id = int(str(path).split("run")[-1])
            if folder_id > experiment_id:
                experiment_id = folder_id
        except BaseException:
            pass
    experiment_id += 1
    folder = str(args.folder / f"run{experiment_id}")
    
    data = {"action": [], "ee_states": [], "joint_states": [], "gripper_states": []}
    previous_state_dict = None
    start = False
    
    
    ## to startup spot
    bosdyn.client.util.setup_logging(args.verbose)
    # Create robot object.
    sdk = bosdyn.client.create_standard_sdk('TeleopClient')
    robot = sdk.create_robot(os.environ.get('SPOT_IP'))
    try:
        bosdyn.client.util.authenticate(robot)
        robot.start_time_sync(args.time_sync_interval_sec)
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
        print("&&& TELEOP and COLLECT DATA")
        # teleop_spot.teleop_spot()
        try:
            teleop_spot.node.print_button_combination()
            while rclpy.ok():
                prev_start = start
                rclpy.spin_once(teleop_spot.node, timeout_sec=0.1)  # Non-blocking spin
                start = teleop_spot.start_
                
                teleop_spot.xbox_to_command()
                
                if start:
                    # print("teleop_spot: ", teleop_spot.action)
                    # print("teleop_spot: ", teleop_spot.gripper)
                    action = teleop_spot.action
                    state = teleop_spot.robot_state_client.get_robot_state() 
                    data["action"].append(action)
                    
                #                    Joint: fl.hx, Position: 0.5140225887298584
                # Joint: fl.hy, Position: 1.4319690465927124
                # Joint: fl.kn, Position: -2.798213481903076
                # Joint: fr.hx, Position: -0.5205919146537781
                # Joint: fr.hy, Position: 1.4288368225097656
                # Joint: fr.kn, Position: -2.7980825901031494
                # Joint: hl.hx, Position: 0.5359048843383789
                # Joint: hl.hy, Position: 1.4178311824798584
                # Joint: hl.kn, Position: -2.7976937294006348
                # Joint: hr.hx, Position: -0.5417367219924927
                # Joint: hr.hy, Position: 1.4174433946609497
                # Joint: hr.kn, Position: -2.7983505725860596
                # Joint: arm0.sh0, Position: 0.016562938690185547
                # Joint: arm0.sh1, Position: -3.143608331680298
                # Joint: arm0.el0, Position: 3.1453487873077393
                # Joint: arm0.el1, Position: 1.5654301643371582
                # Joint: arm0.wr0, Position: -0.003148794174194336
                # Joint: arm0.wr1, Position: -1.5680160522460938
                # Joint: arm0.f1x, Position: -0.021681785583496094
                # {'joint_states': array([ 0.51402259,  1.43196905, -2.79821348, -0.52059191,  1.42883682,
                #        -2.79808259,  0.53590488,  1.41783118, -2.79769373, -0.54173672,
                #         1.41744339, -2.79835057,  0.01656294, -3.14360833,  3.14534879,
                #         1.56543016, -0.00314879, -1.56801605, -0.02168179])}

                    joint_positions = np.array([joint_state.position.value for joint_state in state.kinematic_state.joint_states])


                    state_dict = {
                        "ee_states": np.array(last_state.O_T_EE),
                        "joint_states": joint_positions,
                        "gripper_states": np.array(teleop_spot.gripper),
                    }
                    # 
                    if previous_state_dict is not None:
                        for proprio_key in state_dict.keys():
                            proprio_state = state_dict[proprio_key]
                            if np.sum(np.abs(proprio_state)) <= 1e-6:
                                proprio_state = previous_state_dict[proprio_key]
                            state_dict[proprio_key] = np.copy(proprio_state)
                            
                    for proprio_key in state_dict.keys():
                        data[proprio_key].append(state_dict[proprio_key])

                    previous_state_dict = state_dict
                    
                    # # 3. get images
                    # for camera_id in camera_ids:
                    #     img_info = cr_interfaces[camera_id].get_img_info()
                    #     data[f"camera_{camera_id}"].append(img_info)
                        

                # start turned from true to false signaling to stop recording
                if not start and prev_start:
                    os.makedirs(folder, exist_ok=True)
                    
                    np.savez(f"{folder}/testing_demo_action", data=np.array(data["action"]))
                    np.savez(f"{folder}/testing_demo_ee_states", data=np.array(data["ee_states"]))
                    np.savez(f"{folder}/testing_demo_joint_states", data=np.array(data["joint_states"]))
                    np.savez(f"{folder}/testing_demo_gripper_states",data=np.array(data["gripper_states"]),)

                    # for camera_id in camera_ids:
                    #     np.savez(
                    #         f"{folder}/testing_demo_camera_{camera_id}",
                    #         data=np.array(data[f"camera_{camera_id}"]),
                    #     )
                    #     cr_interfaces[camera_id].stop()
                    
                    break # check what you want to do here cause if we want to start a new demo changes need to be made
                
                print("Total length of the trajectory: ", len(data["action"]))  
        except KeyboardInterrupt:
            pass
        finally:
            teleop_spot.node.destroy_node()
            rclpy.shutdown()
            teleop_spot._safe_power_off()
            time.sleep(2.0)
        
        
    except Exception as e:
        robot.logger.error('Teleop has thrown an error: %s', e)
    finally:
        # Do any final cleanup steps.
        teleop_spot.power_off()


if __name__ == "__main__":
    main()