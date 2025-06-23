#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper
from scipy.spatial.transform import Rotation as R
import pandas as pd
import numpy as np
import socket

from geometry_msgs.msg import PoseStamped, TwistStamped, QuaternionStamped
from flightros_msgs.msg import FlightmareStamped

import rospy
from std_msgs.msg import Bool, Empty, Float32MultiArray
from geometry_msgs.msg import Vector3
from cascated_pid import CascatedPID

from collections import deque

from pysticks import get_controller

MANUAL = False

class Pilot(object):
    #
    def __init__(
        self,
    ):
        # URL (is displayed on arduino side)
        self.UDP_IP = "192.168.100.187"
        self.UDP_PORT = 2390
        self.sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP

        # Action
        # self.con = get_controller()
        self.action = np.zeros(4)

        #
        self.control_rate = 100
        self.policy = CascatedPID(self.control_rate)

        
        self.t0 = 0
        #
        self.init()
        
        self.state_action_pub_ = rospy.Publisher(
            "/state_action", FlightmareStamped, queue_size=1, tcp_nodelay=True
        )

        self.action_wifi_ = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.cmd_wifi_callback
        )

        self.mocap_pose_sub = rospy.Subscriber(
            "/vrpn_client_node/FlatPlate/pose", PoseStamped, self.mocap_pose_callback, queue_size=1
        )
        self.timer_control_loop = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.cmd_pub_callback
        )
    
    def __del__(self):
        com =  self.encode([-1,0,0,0])
        data = self.call(com)

    def init(self):
        self.init_mocap_pose = False

        #
        run_average = 1
        self.pos = np.array([0,0,1]) #can't initialize on the ground, otherwise collision detected
        self.pos_old = self.pos
        self.quat = np.array([1,0,0,0])
        self.quat_old = self.quat
        self.vel = np.array([0,0,0])
        self.vel_old = self.vel
        self.omega = np.array([0,0,0])
        self.omegas = []
        for _ in range(run_average):
            self.omegas.append(self.omega)
        self.omega_old = self.omega
        self.acc = np.array([0,0,0])
        self.aac = np.array([0,0,0])

        # running mean over aac
        aac_rm = 1
        self.aac_list = [self.aac]*aac_rm

        #
        self.frame_id = 0
        self.hist_act_buffer = []

        self.attitude_fused_msg = PoseStamped()

        self.state_action_msg = FlightmareStamped()


    def cmd_pub_callback(self, timer):
        if True:
            # filling state
            state = np.concatenate([self.pos, self.quat, self.vel, self.omega, self.acc, self.aac], axis=0).astype(np.float64)

            robot_state = np.concatenate(([rospy.Time.now()],state)) #in simulation the first entry of the robot state is the time
            self.action = self.policy.predict(robot_state, deterministic=True)[0].flatten()

            # self.action[0] = -1
            
            self.state_action_msg.header.stamp = rospy.Time.now()
            self.state_action_msg.act_0 = self.action[0]
            self.state_action_msg.act_1 = self.action[1]
            self.state_action_msg.act_2 = self.action[2]
            self.state_action_msg.act_3 = self.action[3]
            self.state_action_msg.s_position.x = state[0]
            self.state_action_msg.s_position.y = state[1]
            self.state_action_msg.s_position.z = state[2]
            self.state_action_msg.s_orientation.w = state[3]
            self.state_action_msg.s_orientation.x = state[4]
            self.state_action_msg.s_orientation.y = state[5]
            self.state_action_msg.s_orientation.z = state[6]
            self.state_action_msg.s_linear_velocity.x = state[7]
            self.state_action_msg.s_linear_velocity.y = state[8]
            self.state_action_msg.s_linear_velocity.z = state[9]
            self.state_action_msg.s_angular_velocity.x = state[10]
            self.state_action_msg.s_angular_velocity.y = state[11]
            self.state_action_msg.s_angular_velocity.z = state[12]
            self.state_action_msg.s_linear_acceleration.x = state[13]
            self.state_action_msg.s_linear_acceleration.y = state[14]
            self.state_action_msg.s_linear_acceleration.z = state[15]
            self.state_action_msg.s_angular_acceleration.x = state[16]
            self.state_action_msg.s_angular_acceleration.y = state[17]
            self.state_action_msg.s_angular_acceleration.z = state[18]
            self.state_action_pub_.publish(self.state_action_msg)

            self.frame_id += 1

    def cmd_wifi_callback(self, timer):
        if MANUAL:
            self.con.update()
            action = [self.con.getThrottle(), self.con.getRoll(), self.con.getPitch(), self.con.getYaw()]

            send command
            com =  self.encode(action)
            data = self.call(com)
            self.action = action
        else:
            com =  self.encode(self.action)
            data = self.call(com)

    def mocap_pose_callback(self, data):
        # position
        self.pos_old = self.pos
        self.pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        pos_offset = [0,0,0] #change manually
        self.pos -= pos_offset
        
        # orientation
        self.quat_old = self.quat
        self.quat = [data.pose.orientation.w,data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z]
        r = R.from_quat(np.hstack([self.quat[1:],self.quat[0]]))
        R_T = np.transpose(r.as_matrix())

        # linear velocity
        self.vel_old = self.vel
        vel = (self.pos - self.pos_old)*self.control_rate
        self.vel = np.matmul(R_T, vel)

        # angular velocity
        self.omega_old = self.omega
        
        attitude_euler = np.flip(r.as_euler('ZYX'))
        attitude_euler_old = np.flip(R.from_quat(np.hstack([self.quat_old[1:],self.quat_old[0]])).as_euler('ZYX'))
        omega = (attitude_euler - attitude_euler_old)*self.control_rate
        self.omegas.append(omega)
        self.omegas.pop(0)
        self.omega = np.mean(self.omegas,axis=0)

        # linear acceleration
        self.acc = (self.vel - self.vel_old)*self.control_rate

        # angular acceleration
        self.aac = (self.omega - self.omega_old)*self.control_rate

        if not self.init_mocap_pose:
            print('mocap_pose initialized')
            self.init_mocap_pose = True

    def encode(self, con):
        con = np.array(con)
        # all the different commands (-1,1) are converted into a string with positive numbers (0,1) at a certain resolution
        res = 3
        con = np.round(con, res)
        con = self.compress(con)
        num = 0
        for i in range(len(con)):
            con_clipped = np.clip(con[i],0,1-10**-res)
            num += int(con_clipped*10**res) * 10**(i*res)
        return str(num).zfill(len(con)*res)

    def compress(self, con):
        return con/2 + 0.5

    def call(self, con):
        self.sock.sendto(bytes(str(con), "utf-8"), (self.UDP_IP, self.UDP_PORT))

def main():
    # -- ros spin
    rospy.init_node("flightpilot", anonymous=True)

    pilot = Pilot()

    rospy.spin()

if __name__ == "__main__":
    main()
