# Teleop Button Combination

        | Button Combination | Functionality            |
        |--------------------|--------------------------|
        | A                  | Endeff Up                |
        | B                  | CW wrist                 |
        | X                  | CCW wrist                |
        | Y                  | Endeff Down              |
        | RT                 | Sit                      |
        | LT                 | Stand                    |
        | RB                 | Open gripper             |
        | LB                 | Close gripper            |
        |                    |                          |
        | RT + :             |                          |
        | - A                | Dock/Undock              |
        | - B                | Start Flag               |
        |                    |                          |
        | RB + :             |                          |
        | - A                | Stow                     |
        | - B                | Unstow                   |
        |                    |                          |
        | LT + :             |                          |
        | - A                | Return/Acquire Lease     |
        | - B                |                          |
        |                    |                          |
        | LB + :             |                          |
        | - A                | Power On                 |
        | - B                | Power Off                |
        |                    |                          |
        | - Left Stick       | Endeff                   |
        |  - X               |  left/right              |
        |  - Y               |  forward/backward        |
        |                    |                          |
        | - Right Stick      | Endeff                   |
        |  - X               |  Rotate in yaw axis      |
        |  - Y               |  Rotate in yaw axis      |
        |                    |                          |
        | - Cross buttons    | Body                     |
        |  - Up              | Go forward               |
        |  - Down            | Go backward              |
        |  - Left            | Strafe left              |
        |  - Right           | Strafe right             |
        |  -- + LTRT         |                          |
        |   - Up             | Raise body               |
        |   - Down           | Lower body               |
        |   - Left           | Rotate left              |
        |   - Right          | Rotate right             |
        |                    |                          |
# Scripts:
teleop_robot.py: enable teleoperating spot with xbox. It listens to /joy topic.
collect_data: RT+A starts data collection saveds in npz files. 
read_npz.ipynb: notebook to visualize the save data
create_hdf5_dataset.py: transforms the npz files to one hdf5 file for policy training

# NOTE:

1- Move the hand with the body: the arm does not move as smoothly as the body, making synchronization challenging. This will be addressed later. For now, the arm and body need to be controlled separately. A workaround is to unstow the robot and move it, as the body and arm move together by default when in the unstowed position.

2- When docked the power goes off, so you need to power it back on with LB+A if you are planning to undock without restarting the script

# TODO:
1) merge keys sit stand tak RT/B 
2) fix replay 
