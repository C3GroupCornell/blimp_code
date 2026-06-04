########################################################
# optitrack_node.py
#
# Receives data from Windows script that communicates with OptiTrack
# Note: THIS BINDS TO A SCRIPT ON THE WINDOWS SYSTEM
# That script is launched by the launch file, but this script will
# Not be able to communicate directly with OptiTrack as WSl
# is a virtual machine
#
########################################################


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from blimp_msgs.msg import OptiTrackPose, GoalMsg, Blimps
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32

import threading, time, sys
import socket
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import re

from NatNetClient import NatNetClient

SERVER_IP = "192.168.0.104"   # Motive PC IP (Can be found in OptiTrack software under data streaming panel)
LOCAL_IP  = "192.168.0.101"   # This machine's IP (Can be found using "ip a" in terminal or on TPLink domain 192.168.0.1)

data_lock = threading.Lock()
AGENT_PATTERN = re.compile(r"^agent_\d+$")


class OptiTrackNode(Node):
    def __init__(self):
        super().__init__('optitrack_node')

        self.data_lock = threading.Lock()
        self.goal_to_agent = dict()
        self.goal_publisher_map = dict()
        self.publisher_map = dict()

        self.maps_init = False

        self.discovered_id_pub = self.create_publisher(Int32, '/optitrack_node/discovered_id', 20)
        self.create_subscription(Blimps, '/blimps_initialize', self.blimps_initialize_callback, 10)

    def blimps_initialize_callback(self, msg):
        with self.data_lock:
            ids = [int(id_) for id_ in msg.ids if id_ != '']
            goals = [int(goal) for goal in msg.goals if goal != '']
            self.get_logger().info(f"Blimps initialize: {ids}, {goals}")
            for id_ in ids:
                self.publisher_map[id_] = self.create_publisher(OptiTrackPose, f'/agent_{id_}/optitrack_node/pose', 5)
                self.goal_publisher_map[id_] = self.create_publisher(GoalMsg, f'/agent_{id_}/controller/goal', 5)
            # One goal marker may be shared by multiple blimps, so map goal_id -> [agent_ids]
            self.goal_to_agent = {}
            for (id_, goal) in zip(ids, goals):
                self.goal_to_agent.setdefault(goal, []).append(id_)
    
        
            self.maps_init = True

    def publish_goal(self,goal_values,publisher):
        msg = GoalMsg() #Custom goal message defined in blimp_msgs/msg/GoalMsg.msg
        msg.id = int(goal_values[-1])
        msg.x = goal_values[0]
        msg.y = goal_values[1]
        msg.z = goal_values[2]
        msg.roll = goal_values[3]
        msg.pitch = goal_values[4]
        msg.yaw = goal_values[5]
        msg.ux = goal_values[6]
        msg.uy = goal_values[7]
        msg.uz = goal_values[8]
        msg.wx = goal_values[9]
        msg.wy = goal_values[10]
        msg.wz = goal_values[11]
        publisher.publish(msg)

    def receive_natnet(self,id_,position,rotation):
        try:
            id_int = int(id_)
        except ValueError:
            return
        
        discovered = Int32()
        discovered.data = id_int
        self.discovered_id_pub.publish(discovered)

        x,y,z = position
        qx,qy,qz,qw = rotation
        
        if id_int in self.publisher_map and self.maps_init: #if pose, publish pose
            roll,pitch,yaw = self.quat_to_euler(np.array([qx,qy,qz,qw]).astype(float))
            msg = OptiTrackPose()
            msg.id = id_int
            msg.x = float(x)
            msg.y = float(y)
            msg.z = float(z)
            msg.roll = float(roll)
            msg.pitch = float(pitch)
            msg.yaw = float(yaw)
            msg.time = self.get_clock().now().nanoseconds/1e9
            self.publisher_map[id_int].publish(msg)

        if id_int in self.goal_to_agent and self.maps_init: # if goal, publish to all agents that use this marker
            goal_arr = [0.0]*13
            goal_arr[0] = x
            goal_arr[1] = y
            goal_arr[-1] = id_int
            goal_msg = np.array(goal_arr, dtype=np.float64)
            for agent_id in self.goal_to_agent[id_int]:
                self.publish_goal(goal_msg, self.goal_publisher_map[agent_id])

    def quat_to_euler(self, q):
        # assume quaternion normalized (or normalize here)
        # rotation order: roll about x, pitch about z, yaw about y (intrinsic)
        qx, qy, qz, qw = q
        # qw *= -1
        r10 = 2.0*(qw*qz + qx*qy)
        r11 = 1.0 - 2.0*(qx*qx + qz*qz)
        r12 = 2.0*(qy*qz - qw*qx)
        r00 = 1.0 - 2.0*(qy*qy + qz*qz)
        r20 = 2.0*(qx*qz - qw*qy)

        # clamp for asin numeric stability
        if r10 > 1.0: r10 = 1.0
        if r10 < -1.0: r10 = -1.0

        pitch = math.asin(r10)              # pitch (about z)
        roll  = math.atan2(-r12, r11)      # roll  (about x)
        yaw   = math.atan2(-r20, r00)      # yaw   (about y)
        if yaw < 0: # yaw in range [-pi,pi] -> change to [0,2*pi]
            yaw += 2*np.pi

        return roll, pitch, yaw  # (phi, theta, psi) in radians
        
    
def main(args=None):
    rclpy.init(args=args)
    optitrack_node = OptiTrackNode()

    # Start the NatNetClient
    client = NatNetClient(SERVER_IP, LOCAL_IP)
    client.rigidBodyListener = optitrack_node.receive_natnet
    t = threading.Thread(target=client.run,daemon=True)
    t.start()

    # Start receiving data
    try:
        rclpy.spin(optitrack_node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown client and node
    t.join(timeout=2.0)
    optitrack_node.destroy_node()

if __name__ == '__main__':
    main()