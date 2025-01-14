import threading
import queue

import IPython

import os
import numpy as np
import rospy
import time
import pinocchio as pin

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber

import unitree_sdk2py.idl.unitree_hg.msg.dds_ as dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import MotorCmd_, MotorState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsCmd_, BmsState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, MainBoardState_, PressSensorState_

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__MotorCmd_, unitree_hg_msg_dds__MotorState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__BmsCmd_, unitree_hg_msg_dds__BmsState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__IMUState_, unitree_hg_msg_dds__MainBoardState_, unitree_hg_msg_dds__PressSensorState_

from utils import *
from g1.loco.g1_loco_client import LocoClient
from g1.config.G1_JOINTS import UpperJointIndex, kNotUsedJoint, UPPER_JOINT_LIMITS, IS_WEAK_JOINT, IS_WRIST_JOINT, IS_WAIST_JOINT, DOFs
from g1.g1_utils.robot_arm_ik import G1_29_ArmIK
from g1.g1_utils.data_struct import *


class G1Controller:
    """
    ROS Node Controller for Unitree G1. 
    """

    def __init__(self,
                 use_real=False) -> None:
        
        # constants
        self.n_joint_dof = len(UpperJointIndex)
        self.n_lin_dof = 3
        self.n_dof = self.n_joint_dof + self.n_lin_dof
        self.control_dt = 0.01
        
        # default vel bound
        self.dof_vel_max = np.ones(self.n_dof) * 2.0
        self.dof_vel_min = -np.ones(self.n_dof) * 2.0

        self._initialize_nominal_tracking_pid()
        
        # state
        self.dof_pos = np.zeros(self.n_dof)
        self.dof_vel = np.zeros(self.n_dof)
        self.dof_acc = np.zeros(self.n_dof)
        self.use_real = use_real
        
        # async command
        self.dof_pos_cmd = np.zeros(self.n_dof)
        self.dof_vel_cmd = np.zeros(self.n_dof)
        self.dof_acc_cmd = np.zeros(self.n_dof)
        
        # If using the real G1 humanoid robot.
        if use_real:
            ChannelFactoryInitialize()
            
            self._setup_subscribers()
            self._start_sub_threads()

            self._setup_publishers()
            self._setup_clients()

            self._initialize_arm_sdk_cmd()
            self._initialize_arms_parameters()
            self._initialize_sports_mode()
        
        self._initialize_arm_ik()
            
        try:
            console_width = os.get_terminal_size().columns
        except OSError:
            console_width = 80
        print()
        print("*" * console_width)
        print("SPARK".center(console_width))
        print("G1 Controller Initialized...".center(console_width))
        if use_real:
            print("for real Unitree hardware.".center(console_width))
        else:
            print("for Gazebo simulation.".center(console_width))
        print("*" * console_width)
        print()
        
    def _setup_subscribers(self):
        self._low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)  # WORKS!
        self._low_state_suber.Init()
        self._low_state_buffer = DataBuffer()

        self._arm_sdk_suber = ChannelSubscriber("rt/arm_sdk", LowCmd_)  # WORKS!
        self._arm_sdk_suber.Init()
        self._arm_sdk_buffer = DataBuffer()

    def _start_sub_threads(self):
        threading.Thread(target=self._low_state_sub_thread, daemon=True).start()
        
        cnt = 0
        while not self._low_state_buffer.GetData():
            time.sleep(0.1)
            cnt += 1
            if cnt % 10 == 0:
                print("Waiting to get lowstate data...")
            if cnt > 100:
                raise Exception("Failed to get lowstate data.")

        # Arm SDK hasn't published any data yet. Thus, don't wait.
        threading.Thread(target=self._arm_sdk_sub_thread, daemon=True).start()

    def _low_state_sub_thread(self):
        while True:
            msg = self._low_state_suber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                # IMU State
                lowstate.imu_rpy = msg.imu_state.rpy
                lowstate.imu_quaternion = msg.imu_state.quaternion
                lowstate.imu_gyroscope = msg.imu_state.gyroscope
                lowstate.imu_accelerometer = msg.imu_state.accelerometer
                # Motor State
                for id, motor in enumerate(msg.motor_state):
                    lowstate.motor_state[id].q = motor.q
                    lowstate.motor_state[id].dq = motor.dq
                    lowstate.motor_state[id].ddq = motor.ddq
                    lowstate.motor_state[id].tau_est = motor.tau_est

                self._low_state_buffer.SetData(lowstate)

    def _arm_sdk_sub_thread(self):
        while True:
            msg = self._arm_sdk_suber.Read()
            if msg is not None:
                arm_sdk_state = G1_29_ArmSDK_State()
                # NotUsedJoint
                arm_sdk_state.arm_sdk_notUsedJoint_q = msg.motor_cmd[29].q
                # arm_sdk_state.arm_sdk_notUsedJoint_q = msg.motor_cmd[29].q
                # Arm SDK Command
                for id, motor in enumerate(msg.motor_cmd[12:29]):
                    arm_sdk_state.arm_motor_cmd[id-12].q = motor.q
                    arm_sdk_state.arm_motor_cmd[id-12].dq = motor.dq
                    arm_sdk_state.arm_motor_cmd[id-12].kp = motor.kp
                    arm_sdk_state.arm_motor_cmd[id-12].kd = motor.kd
                    arm_sdk_state.arm_motor_cmd[id-12].tau = motor.tau

                self._arm_sdk_buffer.SetData(arm_sdk_state)
                
    def _initialize_arm_ik(self):
        self.arm_dyn = G1_29_ArmIK(Unit_Test=False, Visualization=False)

    def _setup_publishers(self):
        self._arm_sdk_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self._arm_sdk_publisher.Init()

    def _setup_clients(self):
        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()
        
    def _initialize_arm_sdk_cmd(self):
        # Arm SDK Command
        self.weight_notUsedJoint = 0.0      # Weight for notUsedJoint
        self.arm_sdk_msg = unitree_hg_msg_dds__LowCmd_()    # Arm SDK Command
    
    def _initialize_nominal_tracking_pid(self):
        self.nominal_K_p = np.ones(self.n_dof)
        self.nominal_K_d = np.ones(self.n_dof)
        # self.tracking_K_p = np.ones(self.n_dof)
        ### Nominal ###
        self.nominal_K_p_high  = 105.0
        self.nominal_K_d_high  = 0.0 #0.05
        
        self.nominal_K_p_low   = 105.0
        self.nominal_K_d_low   = 0.0 #0.05
        
        self.nominal_K_p_wrist = 105.0
        self.nominal_K_d_wrist = 0.0 #0.025
        ### Tracking ###
        # self.tracking_K_p_high = 0.1
        # self.tracking_K_p_low = 0.8
        # self.tracking_K_p_wrist = 0.3
        
        for idx, joint in enumerate(UpperJointIndex):
            if IS_WRIST_JOINT(joint):
                self.nominal_K_p[idx] = self.nominal_K_p_wrist
                self.nominal_K_d[idx] = self.nominal_K_d_wrist
                # self.tracking_K_p[idx] = self.tracking_K_p_wrist
            elif IS_WEAK_JOINT(joint):
                self.nominal_K_p[idx] = self.nominal_K_p_low
                self.nominal_K_d[idx] = self.nominal_K_d_low
                # self.tracking_K_p[idx] = self.tracking_K_p_low
            else:
                self.nominal_K_p[idx] = self.nominal_K_p_high
                self.nominal_K_d[idx] = self.nominal_K_d_high
                # self.tracking_K_p[idx] = self.tracking_K_p_high
              
        self.nominal_K_p[-3:] = 0.0 
        self.nominal_K_d[-3:] = 0.0 
        # self.tracking_K_p[-3:] = 0.0  
    
    def _initialize_arms_parameters(self):
        self.arm_q_lambda = 0.8
        ### Arm controller ###
        self.arm_sdk_kp_high = 100.0
        self.arm_sdk_kd_high = 3.0
        self.arm_sdk_kp_low = 80.0
        self.arm_sdk_kd_low = 3.0
        self.arm_sdk_kp_wrist = 30.0
        self.arm_sdk_kd_wrist = 1.5
        self.arm_sdk_velocity_limit = 20.0
        ### Arm control parameters ###
        self._arm_sdk_speed_gradual_max = False
        self._arm_sdk_gradual_start_time = None
        self._arm_sdk_gradual_time = None
        ### Arm Kp and Kd ###
        self.default_arm_sdk_kp = np.zeros(17)
        self.default_arm_sdk_kd = np.zeros(17)
        for idx, joint in enumerate(UpperJointIndex):
            if IS_WRIST_JOINT(joint):
                self.default_arm_sdk_kp[idx] = self.arm_sdk_kp_wrist
                self.default_arm_sdk_kd[idx] = self.arm_sdk_kd_wrist
            elif IS_WEAK_JOINT(joint):
                self.default_arm_sdk_kp[idx] = self.arm_sdk_kp_low
                self.default_arm_sdk_kd[idx] = self.arm_sdk_kd_low
            else:
                self.default_arm_sdk_kp[idx] = self.arm_sdk_kp_high
                self.default_arm_sdk_kd[idx] = self.arm_sdk_kd_high

    def _initialize_sports_mode(self):
        """
        Equivalent to `Damp` --> `LockStand` --> `R1 + X` (Mode 1) in the joystick controller.
        """
        # paths = {
        #     0: [1, 4, 200],
        #     1: [4, 200],
        #     4: [200],
        #     500: [200],
        #     200: [],
        # }
        
        paths = {
            0: [1, 4, 500],
            1: [4, 500],
            4: [500],
            200: [500],
            # 500: [1, 4, 500],
            500: [],
        }
        
        current_fsm_id = self.loco_client.GetFsmId(0)[1]
        required_sequence = paths.get(current_fsm_id)
        if required_sequence is None:
            # Handle unexpected FSM ID
            return False
        for fsm_id in required_sequence:
            if not self._switch_fsm_id(fsm_id):
                return False
        return True

    def _switch_fsm_id(self, 
                       target_id: int, 
                       max_attempts=5, 
                       sleep_time=1.0):
        attempts = 0
        while attempts < max_attempts:
            code, data = self.loco_client.SetFsmId(target_id)
            if code == 0:
                time.sleep(5)  # Wait after successful switch
                return True
            attempts += 1
            time.sleep(sleep_time)
        return False
    
    def update_dynamic_states(self, dof_pos=None, dof_vel=None, dof_acc=None):
        
        if self.use_real:
            self.dof_pos[:self.n_joint_dof] = self.get_upper_motor_q()
            self.dof_pos[self.n_joint_dof:] = 0.0 # update lin info later
            self.dof_vel[:self.n_joint_dof] = self.get_upper_motor_dq()
            self.dof_vel[self.n_joint_dof:] = 0.0 # update lin info later
            self.dof_acc[:self.n_joint_dof] = self.get_upper_motor_ddq()
            self.dof_acc[self.n_joint_dof:] = 0.0 # update lin info later
        else:
            self.dof_pos[:] = dof_pos[:]
            self.dof_vel[:] = dof_vel[:]
            self.dof_acc[:] = dof_acc[:]
    
    def get_imu_rpy(self):
        return np.array(self._low_state_buffer.GetData().imu_rpy)
    
    def get_imu_quaternion(self):
        return np.array(self._low_state_buffer.GetData().imu_quaternion)
    
    def get_imu_gyroscope(self):
        return np.array(self._low_state_buffer.GetData().imu_gyroscope)
    
    def get_imu_accelerometer(self):
        return np.array(self._low_state_buffer.GetData().imu_accelerometer)
    
    def get_motor_q(self):
        cur_motor_q = np.array([
            self._low_state_buffer.GetData().motor_state[id].q for id in range(35)
        ])
        return cur_motor_q
    
    def get_motor_dq(self):
        cur_motor_dq = np.array([
            self._low_state_buffer.GetData().motor_state[id].dq for id in range(35)
        ])
        return cur_motor_dq
    
    def get_motor_ddq(self):
        cur_motor_ddq = np.array([
            self._low_state_buffer.GetData().motor_state[id].ddq for id in range(35)
        ])
        return cur_motor_ddq
      
    def get_upper_motor_q(self):
        cur_upper_motor_q = np.array([
            self._low_state_buffer.GetData().motor_state[id].q for id in range(12,29)
        ])
        return cur_upper_motor_q
    
    def get_upper_motor_dq(self):
        cur_upper_motor_dq = np.array([
            self._low_state_buffer.GetData().motor_state[id].dq for id in range(12,29)
        ])
        return cur_upper_motor_dq
    
    def get_upper_motor_ddq(self):
        cur_upper_motor_ddq = np.array([
            self._low_state_buffer.GetData().motor_state[id].ddq for id in range(12,29)
        ])
        return cur_upper_motor_ddq
    
    def get_arm_sdk_q(self):
        cur_arm_sdk_q = np.array([
            self._arm_sdk_buffer.GetData().arm_motor_cmd[id].q for id in range(17)
        ])
        return cur_arm_sdk_q
    
    def solve_ik(self, 
                 L_tf_target: pin.SE3, 
                 R_tf_target: pin.SE3, 
                 current_arm_q: np.ndarray, 
                 current_arm_dq: np.ndarray):
        """
        Solve the Inverse Kinematics for the arms.
        """
        sol_q = np.zeros(14)
        sol_tauff = np.zeros(14)
        try:
            sol_q, sol_tauff = self.arm_dyn.solve_IK(L_tf_target.homogeneous, R_tf_target.homogeneous, current_arm_q, current_arm_dq)
        except Exception as e:
            print("IK Failed: ", e)
        return sol_q, sol_tauff

    def clip_arm_q_target(self, 
                          target_q: np.ndarray, 
                          velocity_limit: float, 
                          ctrl_dt: float):
        """
        Clip the target joint angles based on the velocity limit.
        """
        current_q = self.motor_q[12:29]
        delta_q = target_q - current_q
        motion_scale = np.max(np.abs(delta_q)) / (velocity_limit * ctrl_dt)
        cliped_target_q = current_q + delta_q / max(motion_scale, 1.0)
        return cliped_target_q
    
    def speed_gradual_max(self, t=5.0):
        """
        Parameter `t` is the total time required for arms velocity to gradually
        increase to its maximum value, in seconds. 
        """
        self._arm_sdk_gradual_start_time = time.time()
        self._arm_sdk_gradual_time = t
        self._arm_sdk_speed_gradual_max = True

    def emergency_stop(self):
        self.loco_client.SetFsmId(1)    # Damp
    
    def reset_arm_sdk_cmd(self):
        self._initialize_arm_sdk_cmd()
    
    def reset_arms_parameters(self):
        self._initialize_arms_parameters()
        
    def send_arm_sdk_command(self):
        self._arm_sdk_publisher.Write(self.arm_sdk_msg)

    def set_arm_sdk_command(self, 
                            weight: float, 
                            arm_mode: np.ndarray, 
                            arm_q: np.ndarray, 
                            arm_dq: np.ndarray, 
                            arm_kp: np.ndarray, 
                            arm_kd: np.ndarray, 
                            arm_tau: np.ndarray):
        self.weight_notUsedJoint = max(min(weight, 1.0), 0)
        self.arm_sdk_msg.motor_cmd[kNotUsedJoint].q = weight * weight
        
        for idx, joint in enumerate(UpperJointIndex):
            # filter the command q
            joint_min, joint_max = UPPER_JOINT_LIMITS.get(joint, (-np.inf, np.inf))
            command_arm_q = self.arm_q_lambda * self.arm_sdk_msg.motor_cmd[joint].q + (1 - self.arm_q_lambda) * arm_q[idx]
            command_arm_q = np.clip(command_arm_q, joint_min, joint_max)
            
            self.arm_sdk_msg.motor_cmd[joint].q = command_arm_q
            self.arm_sdk_msg.motor_cmd[joint].dq = arm_dq[idx]
            self.arm_sdk_msg.motor_cmd[joint].kp = arm_kp[idx]
            self.arm_sdk_msg.motor_cmd[joint].kd = arm_kd[idx]
            self.arm_sdk_msg.motor_cmd[joint].tau = arm_tau[idx]

    def reset_upper_body(self):
        
        self.update_dynamic_states()
        
        # print("DOF: ", self.dof_pos)
        
        # time_to_reset = 5.0
        # reset_dof_pos = np.zeros(self.n_dof)
        # reset_dof_pos[DOFIndex.RightElbow] = 1.0
        # reset_dof_pos[DOFIndex.LeftElbow] = 1.0
        # reset_dof_pos[DOFIndex.RightShoulderRoll] = -0.22
        # reset_dof_pos[DOFIndex.LeftShoulderRoll] = 0.22
        # reset_dof_pos[DOFIndex.RightShoulderPitch] = 0.3
        # reset_dof_pos[DOFIndex.LeftShoulderPitch] = 0.3
        
        # reset_time = np.abs(reset_dof_pos - self.dof_pos).max() / 1.0
        # n_pts = np.ceil(reset_time / self.control_dt).astype(int)
        # ddof_pos = (reset_dof_pos - self.dof_pos) / n_pts
        
        # print(f"reset_time: {reset_time}, n_pts: {n_pts}")
        # print(f"ddof_pos: {ddof_pos}")
        
        for i in range(100):
            
            # if i < (n_pts//3):
            #     continue
            
            target_q = np.zeros(17)
            target_dq = np.zeros(17)
            target_kp = np.zeros_like(self.default_arm_sdk_kp)
            target_kp[DOFs.WaistPitch] = self.default_arm_sdk_kp[DOFs.WaistPitch]
            target_kp[DOFs.WaistRoll] = self.default_arm_sdk_kp[DOFs.WaistRoll]
            target_kp[DOFs.WaistYaw] = self.default_arm_sdk_kp[DOFs.WaistYaw]
            target_kd = np.zeros_like(self.default_arm_sdk_kd)
            target_kd[DOFs.WaistPitch] = self.default_arm_sdk_kd[DOFs.WaistPitch]
            target_kd[DOFs.WaistRoll] = self.default_arm_sdk_kd[DOFs.WaistRoll]
            target_kd[DOFs.WaistYaw] = self.default_arm_sdk_kd[DOFs.WaistYaw]
            target_tauff = np.zeros(17)
            target_tauff[DOFs.RightShoulderPitch] = -1.0
            target_tauff[DOFs.LeftShoulderPitch] = -1.0
            target_tauff[DOFs.RightShoulderRoll] = -2.0
            target_tauff[DOFs.LeftShoulderRoll] = 2.0
            target_tauff[DOFs.RightShoulderYaw] = -0.5
            target_tauff[DOFs.LeftShoulderYaw] = 0.5
            target_tauff[DOFs.RightElbow] = -2.0
            target_tauff[DOFs.LeftElbow] = -2.0
            # zero when physically blocked
            target_tauff[DOFs.RightWristPitch] = 0.0 # -0.2
            target_tauff[DOFs.LeftWristPitch] = 0.0 # -0.2
            
            target_tauff[DOFs.RightWristYaw] = -0.2
            target_tauff[DOFs.LeftWristYaw] = 0.2
            
            self.set_arm_sdk_command(1.0, 1.0, target_q, target_dq, target_kp, target_kd, target_tauff)
            self.send_arm_sdk_command()
            
            time.sleep(self.control_dt)
        
        time.sleep(1.0)
        self.update_dynamic_states()
        self.dof_pos_cmd[:] = self.dof_pos[:]
        self.dof_vel_cmd[:] = self.dof_vel[:]
        self.dof_acc_cmd[:] = self.dof_acc[:]
        
        print("Upper body reset done.")

    def reset_upper_body_old(self):
        self.weight_notUsedJoint = 1.0
        _control_dt = 0.02
        _weight_rate = 0.2
        _delta_weight = _weight_rate * _control_dt      # 0.004

        _period = 5
        _num_time_steps = int(_period / _control_dt)    # 250
        
        # _current_jpos_des = self._low_state_buffer.GetData().motor_q[12:29]
        _current_jpos_des = [self._low_state_buffer.GetData().motor_state[id].q for id in range(12, 29)]
        # _current_jpos_des = self.motor_q[12:29]
        for i in range(_num_time_steps):
            self.weight_notUsedJoint -= _delta_weight
            _q_list = _current_jpos_des
            _dq_list = [0 for _ in range(17)]
            _kp_list = [60 for _ in range(17)]
            _kd_list = [1.5 for _ in range(17)]
            _tau_ff_list = [0 for _ in range(17)]
            self.set_arm_sdk_command(self.weight_notUsedJoint, 0, _q_list, _dq_list, _kp_list, _kd_list, _tau_ff_list)
            self.send_arm_sdk_command()
            time.sleep(_control_dt)

        self.reset_arm_sdk_cmd()
        self.reset_arms_parameters()
        print("Upper body reset done.")
        
    def nominal_controller(self, desired_dof_pos):
        
        nominal_dof_vel = self.nominal_K_p * (desired_dof_pos - self.dof_pos_cmd) - self.nominal_K_d * self.dof_vel_cmd
        
        return nominal_dof_vel
    


if __name__ == "__main__":
    
    g1 = G1Controller(use_real=True)
    time.sleep(3)
    g1.reset_upper_body()