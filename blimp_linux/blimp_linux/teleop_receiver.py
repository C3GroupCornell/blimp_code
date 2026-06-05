import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray, Bool
from blimp_msgs.msg import TeleopMode, MotorMsg, Blimps, GoalMsg

import socket
import threading
import re
import time
import pygame

MAX_ALT_VOLTAGE = 0.8
MAX_VOLTAGE = 0.4
DEADZONE = 0.1

data_lock = threading.Lock()
AGENT_PATTERN = re.compile(r"^agent_\d+$")

def clip_cmd(v): #checks deadzone and bounds
    if abs(v) > DEADZONE:
        return v if abs(v) < 1 else v/abs(v)*1
    return 0

class TeleopReceiver(Node):
    def __init__(self):
        super().__init__('teleop_receiver')

        self.current_blimp = None
        self.current_mode = None
        self.teleop = True
        self.blimps = {}
        self.prev_dpad_x = 0
        self.prev_dpad_y = 0
        self.prev_button_y = 0
        self.calibration_pubs = {}
        self.calibrating = False

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None

        self.create_subscription(Blimps, '/blimps_initialize',self.update_blimps_callback, 10)
        self.create_subscription(TeleopMode, '/teleop_mode',self.update_teleop_callback, 10)

        self.motor_pub = self.create_publisher(MotorMsg, '/motor_cmd', 10)
        self.fly_to_goal_pub = self.create_publisher(TeleopMode, "/fly_to_goal", 5)
        self.teleop_mode_pub = self.create_publisher(TeleopMode, "/teleop_mode", 10)
    
    def update_blimps_callback(self, msg): # Maps blimps to com ports
        self.get_logger().info(f"Blimps: {msg.ids}, {msg.coms}")
        ids, coms = list(msg.ids), list(msg.coms)
        self.blimps = dict(zip(ids, coms))

    def update_teleop_callback(self, msg):
        self.current_blimp = msg.id
        self.current_mode = msg.mode
        if msg.mode==2: # Only create publisher if in controlled mode
            self.goal_pub = self.create_publisher(GoalMsg, f'/agent_{self.current_blimp}/controller/goal_inc', 10)
        self.get_logger().info(f"Teleop: id={self.current_blimp}, mode={self.current_mode}")

    def receive_teleop(self):
        self.get_logger().info("Started receiving teleop")
        while rclpy.ok():
            pygame.event.pump()
            if self.joystick:
                axes = [clip_cmd(self.joystick.get_axis(i)) for i in range(self.joystick.get_numaxes())]
                buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
                hat = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
                dpad_x, dpad_y = hat
                button_a, button_b, button_x, button_y, left_bumper, right_bumper = buttons[:6]
                lj_horizontal, lj_vert,_, rj_horizontal, rj_vert, _ = axes
                # Change teleop mode
                if dpad_y != 0 and self.prev_dpad_y == 0 and self.current_blimp is not None:
                    # dpad up (+1) increments mode, dpad down (-1) decrements; modes are 0=Manual, 1=All, 2=Controlled
                    step = 1 if dpad_y == 1 else -1
                    base_mode = self.current_mode if self.current_mode is not None else 0
                    new_mode = (base_mode + step) % 3
                    tm = TeleopMode()
                    tm.id = self.current_blimp
                    tm.mode = new_mode
                    self.teleop_mode_pub.publish(tm)
                    self.get_logger().info(f"Switched to mode={new_mode}")
                self.prev_dpad_y = dpad_y


                if button_a == 1: # Enable manual teleop
                    self.teleop = True
                    self.calibrating = False
                    targets = list(self.blimps.keys()) if self.current_mode == 1 else (
                        [self.current_blimp] if self.current_blimp is not None else []
                    )
                    for bid in targets:
                        msg = TeleopMode()
                        msg.id = bid
                        msg.mode = 0
                        self.fly_to_goal_pub.publish(msg)

                elif button_b == 1: # Enable fly to goal
                    self.teleop = False
                    targets = list(self.blimps.keys()) if self.current_mode == 1 else (
                        [self.current_blimp] if self.current_blimp is not None else []
                    )
                    for bid in targets:
                        msg = TeleopMode()
                        msg.id = bid
                        msg.mode = 1
                        self.fly_to_goal_pub.publish(msg)

                if self.teleop and not self.calibrating and (self.current_mode == 0 or self.current_mode == 1) and self.current_blimp is not None: # Manual mode
                    voltages = [0.0] * 6
                    vertical = -lj_vert/abs(lj_vert+1e-6) * min(abs(lj_vert),MAX_ALT_VOLTAGE) #dirn times magnitude
                    yaw = -rj_horizontal/abs(rj_horizontal+1e-6) * min(abs(rj_horizontal),MAX_VOLTAGE)
                    forward = -rj_vert/abs(rj_vert+1e-6) * min(abs(rj_vert),MAX_VOLTAGE)
                    voltages[2] = vertical
                    voltages[3] = -vertical
                    if yaw > 0:
                        voltages[1] = voltages[4] = yaw
                    else:
                        voltages[0] = voltages[5] = abs(yaw)
                    
                    if forward > 0:
                        voltages[4] += forward
                        voltages[5] += forward
                    elif forward < 0:
                        voltages[0] += abs(forward)
                        voltages[1] += abs(forward)
                    
                    self.get_logger().info(f"Publishing voltages: {voltages}")

                    if self.current_mode == 0: # Single control
                        self.motor_pub.publish(MotorMsg(id=self.current_blimp, com=self.blimps[self.current_blimp], voltages=Float32MultiArray(data=voltages)))
                    else: # All control
                        for b in self.blimps:
                            self.motor_pub.publish(MotorMsg(id=b,com=self.blimps[b],voltages=Float32MultiArray(data=voltages)))

                elif self.teleop and not self.calibrating and self.current_mode == 2 and self.current_blimp is not None: # Controlled mode
                    vertical = -lj_vert/abs(lj_vert+1e-6)
                    yaw = -rj_horizontal/abs(rj_horizontal+1e-6)
                    forward = -rj_vert/abs(rj_vert+1e-6)
                    msg = GoalMsg()
                    msg.x = forward
                    msg.z = vertical
                    msg.yaw = yaw
                    self.goal_pub.publish(msg)
                


def main(args=None):
    rclpy.init(args=args)
    teleop_receiver = TeleopReceiver()

    teleop_thread = threading.Thread(target=teleop_receiver.receive_teleop, daemon=True)
    teleop_thread.start()

    try:
        rclpy.spin(teleop_receiver)
    except KeyboardInterrupt:
        pass
    teleop_receiver.destroy_node()
    rclpy.shutdown()
'''
AXES:
0: Left Joystick Left/Right (-1 left)
1: Left Joystick Up/Down (-1 up)
2: Right Joystick Left/Right (-1 left)
3: Right Joystick Up/Down (-1 up)

BUTTONS:
0: A
1: B
2: X
3: Y
4: LB
5: RB
'''