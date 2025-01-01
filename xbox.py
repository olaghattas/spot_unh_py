import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
"""Teleop SPOT with xbox using Boston Dynamics API"""
# ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

import textwrap
# # Print controls
print(
    textwrap.dedent("""\
| Button Combination | Functionality            |
|--------------------|--------------------------|
| A                  | Endeff Up                |
| B                  | CCW wrist                |
| X                  | CW wrist                 |
| Y                  | Endeff Down              |
| RT                 | Sit                      |
| LT                 | Stand                    |
|                    |                          |
| RT + :             |                          |
| - A                | Dock                     |
| - B                | Undock                   |
|                    |                          |
| LT + :             |                          |
| - A                | Return Lease             |
| - B                | Acquire Lease            |
|                    |                          |
| LB + :             |                          |
| - A                | Power On                 |
| - B                | Power Off                |
|                    |                          |
| - Left Stick       | Endeff                   |
|  -- X              |  left/right              |
|  -- Y              |  forward/backward        |
|                    |                          |
| - Right Stick      | Endeff                   |
|  -- X              |  Rotate in yaw axis      |
|  -- Y              |  Rotate in yaw axis      |
|                    |                          |
| Cross buttons      | Body                     |
|  - Up              | Go forward               |
|  - Down            | Go backward              |
|  - Left            | Go left                  |
|  - Right           | Go right                 |
|                    |                          |
        """))

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
        self.buttons_pressed = ""
        self.body = ""
        # x,y,z row,pitch,yaw should be 
        # but we are using button to control z and wrist
        self.end_eff = [0,0,0,0]
        
        self.wait_RT_started = False
        self.wait_LT_started = False
        self.wait_RT_done = False
        self.wait_LT_done = False
        
        self.wait_RB_started = False
        self.wait_LB_started = False
        self.wait_RB_done = False
        self.wait_LB_done = False

    def joy_callback(self, msg):
        # Callback triggered on receiving a message
        msg_ = msg

        buttons_pressed = ""
        self.body = ""
        self.end_eff = [0,0,0,0] # might have to change this
        
        if msg_.buttons[0]:
            buttons_pressed = buttons_pressed + "A"
        if msg_.buttons[1]:
            buttons_pressed = buttons_pressed + "B"
        if msg_.buttons[2]:
            buttons_pressed = buttons_pressed + "X"
        if msg_.buttons[3]:
            buttons_pressed = buttons_pressed + "Y"
        if msg_.buttons[4]:
            buttons_pressed = buttons_pressed + "LB"
        if msg_.buttons[5]:
            buttons_pressed = buttons_pressed + "RB"
            
        if msg_.axes[2] < -0.99:
            buttons_pressed = buttons_pressed + "LT"
            
        if msg_.axes[5] < -0.99:
            buttons_pressed = buttons_pressed + "RT"
           
        # This logic is to support using combinations
        if buttons_pressed == "RT":
            # print("in RT" ,buttons_pressed)
            if not self.wait_RT_started:
                # print("wait_RT_started")
                self.wait_RT_started = True
                time.sleep(1)
                # print("timer done")
                self.wait_RT_done = True
            elif self.wait_RT_done:
                print("wait_RT_done")
                self.wait_RT_started = False
                self.wait_RT_done = False
                self.buttons_pressed = buttons_pressed
        
        elif buttons_pressed == "LT":
            if not self.wait_LT_started:
                # print("wait_LT_started")
                self.wait_LT_started = True
                time.sleep(1)
                # print("timer done")
                self.wait_LT_done = True
                
            elif self.wait_LT_done:
                # print("wait_LT_done")
                self.buttons_pressed = buttons_pressed
                self.wait_LT_started = False
                self.wait_LT_done = False
                
        elif buttons_pressed == "RB":
            # print("in RT" ,buttons_pressed)
            if not self.wait_RB_started:
                # print("wait_RB_started")
                self.wait_RB_started = True
                time.sleep(1)
                # print("timer done")
                self.wait_RB_done = True
            elif self.wait_RB_done:
                # print("wait_RB_done")
                self.wait_RB_started = False
                self.wait_RB_done = False
                self.buttons_pressed = buttons_pressed
        
        elif buttons_pressed == "LB":
            if not self.wait_LB_started:
                # print("wait_LB_started")
                self.wait_LB_started = True
                time.sleep(1)
                # print("timer done")
                self.wait_LB_done = True
                
            elif self.wait_LB_done:
                # print("wait_LB_done")
                self.buttons_pressed = buttons_pressed
                self.wait_LB_started = False
                self.wait_LB_done = False
        else:
            # print("not in RT or LT" ,buttons_pressed)
            self.buttons_pressed = buttons_pressed

            # reset 
            self.wait_RT_started = False
            self.wait_RT_done = False
            self.wait_LT_started = False
            self.wait_LT_done = False
            self.wait_RB_started = False
            self.wait_LB_started = False
            self.wait_RB_done = False
            self.wait_LB_done = False

        if msg_.axes[6] == 1:
            self.body = "Left"
        elif msg_.axes[6] == -1:
            self.body = "Right"
            
        if msg_.axes[7] == 1:
            self.body = "Up"
        elif msg_.axes[7] == -1:
            self.body = "Down"
            

        if msg_.axes[0]:
            ## left right
            self.end_eff[1] = msg_.axes[0]
        if msg_.axes[1]:
            ## forward backward
            self.end_eff[0] = msg_.axes[1]
        
        if msg_.axes[3]:
            ## left right
            ## needs checking 
            self.end_eff[2] = msg_.axes[3]
        if msg_.axes[4]:
            ## forward backward
            self.end_eff[3] = msg_.axes[4]
 
        # print("stand = " , self.stand)
        # return buttons_pressed, axes

    def xbox_to_command(self):
        buttons_pressed = self.buttons_pressed
        # if buttons_pressed:
            # print(buttons_pressed)
        if buttons_pressed == "ART":
            print("Dock")
            time.sleep(2.0)
            
        elif buttons_pressed == "BRT":
            print("Undock")
            time.sleep(2.0)
            
        elif buttons_pressed == "ALT":
            print("Return Lease")
            time.sleep(2.0)
            
        elif buttons_pressed == "BLT":
            print("Acquire Lease")
            time.sleep(2.0)
        
        elif buttons_pressed == "ALB":
            print("Power On")
            time.sleep(2.0)
            
        elif buttons_pressed == "BLB":
            print("Power Off")
            time.sleep(2.0)
            
        elif buttons_pressed == "A":
            print("Go down")

        elif buttons_pressed == "B":
            print("CCW")

        elif buttons_pressed == "X":
            print("CW")
            
        elif buttons_pressed == "Y":
            print("Go up")
        
        elif buttons_pressed == "LT":
            print("Stand")
            time.sleep(2.0)
            
        elif buttons_pressed == "RT":
            print("Sit")
            time.sleep(2.0)
            
        if self.body == "Left":
             print("Left")
        
        if self.body == "Right":
             print("Right")
            
        if self.body == "Up":
             print("Up")
        
        if self.body == "Down":
             print("Down")
        
        # else:
        #     print("Please choose correct answer")
        if any(self.end_eff):
            print(self.end_eff)
    
def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
   
    node = JoySubscriber()
    print("TELEOP")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)  # Non-blocking spin
            node.xbox_to_command()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
        