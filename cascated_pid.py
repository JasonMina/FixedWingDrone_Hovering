import numpy as np
from pid import PID
from scipy.spatial.transform import Rotation as R
from ruamel.yaml import YAML

class CascatedPID:
    def __init__(self, ctr_dt):
        self.ctr_dt = ctr_dt
        self.act_dim = 4

        self.pid_x = PID(0.0, 0, 0.0, output_limits=(-1,1))
        self.pid_y = PID(0.0, 0, 0.0, output_limits=(-1,1))

        self.pid_thr = PID(1.0, 0, 0.5, output_limits=(-1,1))
        self.pid_ele = PID(1.0, 0, 0.1, output_limits=(-1,1))
        self.pid_rud = PID(0.5, 0, 0.0, output_limits=(-1,1))
        self.pid_ail = PID(0.5, 0, 0.0, output_limits=(-1,1))
        
        self.offset_thr = 0.04

        self.i = 0

    def set_pid_gains_x(self, p, i, d):
        self.pid_x.Kp = p
        self.pid_x.Ki = i
        self.pid_x.Kd = d
        return 0

    def set_pid_gains_y(self, p, i, d):
        self.pid_y.Kp = p
        self.pid_y.Ki = i
        self.pid_y.Kd = d
        return 0
    
    def set_pid_gains_thr(self, p, i, d):
        self.pid_thr.Kp = p
        self.pid_thr.Ki = i
        self.pid_thr.Kd = d
        return 0
    
    def set_pid_gains_ele(self, p, i, d):
        self.pid_ele.Kp = p
        self.pid_ele.Ki = i
        self.pid_ele.Kd = d
        return 0

    def set_pid_gains_rud(self, p, i, d):
        self.pid_rud.Kp = p
        self.pid_rud.Ki = i
        self.pid_rud.Kd = d
        return 0

    def set_pid_gains_ail(self, p, i, d):
        self.pid_ail.Kp = p
        self.pid_ail.Ki = i
        self.pid_ail.Kd = d
        return 0
    

    def set_weights(self):
        cfg = YAML().load(open("weights.yaml", "r"))

        self.pid_x.Kp = cfg["x"]["p"]
        self.pid_x.Ki = cfg["x"]["i"]
        self.pid_x.Kd = cfg["x"]["d"]

        self.pid_y.Kp = cfg["y"]["p"]
        self.pid_y.Ki = cfg["y"]["i"]
        self.pid_y.Kd = cfg["y"]["d"]

        self.pid_thr.Kp = cfg["thr"]["p"]
        self.pid_thr.Ki = cfg["thr"]["i"]
        self.pid_thr.Kd = cfg["thr"]["d"]

        self.pid_ele.Kp = cfg["ele"]["p"]
        self.pid_ele.Ki = cfg["ele"]["i"]
        self.pid_ele.Kd = cfg["ele"]["d"]

        self.pid_rud.Kp = cfg["rud"]["p"]
        self.pid_rud.Ki = cfg["rud"]["i"]
        self.pid_rud.Kd = cfg["rud"]["d"]

        self.pid_ail.Kp = cfg["ail"]["p"]
        self.pid_ail.Ki = cfg["ail"]["i"]
        self.pid_ail.Kd = cfg["ail"]["d"]

    def predict(self, state, deterministic=True):
        goal = [-4,0,1.8]
        return self.update(state, goal), []

    def update(self, state, goal):
        if self.i % 100 == 0:
            self.set_weights()
        self.i += 1

        state = state.flatten()
        self.pos = state[1:4]
        self.quat = state[4:8]
        self.vel = state[8:11]
        self.omega = state[11:14]
        self.acc = state[14:17]
        self.aac = state[17:20]

        rot_R = R.from_euler('y', 90, degrees=True).as_matrix()
        quat = R.from_quat(np.hstack([self.quat[1:],self.quat[0]])).as_matrix()
        self.quat_r = R.from_matrix(np.matmul(quat,rot_R)).as_quat()
        yaw_pitch_roll_r = R.from_quat(self.quat_r).as_euler('ZYX')
        self.roll_pitch_yaw_r = np.flip(yaw_pitch_roll_r)

        # setpoints
        self.pid_x.setpoint = 0
        self.pid_y.setpoint = 0

        self.pid_thr.setpoint = 0

        # roll_pitch_yaw_des
        goal_b = self.world_to_body(goal - self.pos)
        roll_pitch_yaw_des_r = np.zeros(3)
        
        roll_pitch_yaw_des_r[0] = self.pid_y(-goal_b[1], dt=self.ctr_dt)
        roll_pitch_yaw_des_r[1] = self.pid_x(goal_b[0], dt=self.ctr_dt)
        roll_pitch_yaw_des_r[2] = 0

        a_thr = self.pid_thr(self.pos[2] - goal[2], dt=self.ctr_dt) + self.offset_thr

        # actions
        actions = np.zeros(self.act_dim)
        a_ele, a_rud, a_ail = self.attitude_control(roll_pitch_yaw_des_r)
        actions[0] = a_thr + abs(a_ail)*0.05
        actions[1] = a_ail - 0.05
        actions[2] = a_ele + 0.05
        actions[3] = a_rud

        return actions

    def attitude_control(self, roll_pitch_yaw_des_r):
        # setpoints
        self.pid_ele.setpoint = roll_pitch_yaw_des_r[1]
        self.pid_rud.setpoint = roll_pitch_yaw_des_r[0]
        self.pid_ail.setpoint = roll_pitch_yaw_des_r[2]

        a_ele = self.pid_ele(self.roll_pitch_yaw_r[1], dt=self.ctr_dt)
        a_rud = self.pid_rud(self.roll_pitch_yaw_r[0], dt=self.ctr_dt)
        a_ail = self.pid_ail(self.roll_pitch_yaw_r[2], dt=self.ctr_dt)

        return [a_ele, a_rud, a_ail]

    def world_to_body(self, vec):
        R_matrix = R.from_quat(np.hstack([self.quat_r[1:],self.quat_r[0]])).as_matrix()
        return np.matmul(R_matrix.transpose(), vec)