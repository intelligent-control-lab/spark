import os
import time
from enum import IntEnum
import threading
import numpy as np

from spark_agent.base.base_agent import BaseAgent
from spark_agent.real.g1.g1_idl_struct import DataBuffer, MotorState, MotorCmd, LowStateDataStruct, CmdDataStruct, HighStateDataStruct
from spark_robot import RobotConfig


class UnitreeControlLevel(IntEnum):

    LOW = 0     # low level control (`lowcmd`: all joints)
    HIGH = 1    # sports mode (`armsdk` + `LocoClient`)

class G1FSMID(IntEnum):
    
    ZeroTorque = 0
    Damping    = 1
    Squat      = 2
    Sit        = 3
    LockStand  = 4
    MainMode   = 200

class H1FSMID(IntEnum):

    ZeroTorque = 0
    Damping    = 1
    LockStand  = 2
    MainMode   = 204


class G1RealAgent(BaseAgent):
    '''
    Unitree Humanoid Agent. 
    Current version works for Unitree G1 humanoid.
    Future versions will include Unitree H1 humanoid.

    Args:
        robot_cfg (RobotConfig): Robot configuration
        unitree_model (str): Unitree humanoid model. Choose either 'g1' or 'h1'.
        level (str): Control level. Choose either 'low' or 'high'. High level is sports mode.
    '''
    def __init__(self, robot_cfg: RobotConfig, **kwargs):
        super().__init__(robot_cfg)

        # ----------------------------- Agent Parameters ----------------------------- #
        self.num_real_motor = len(self.robot_cfg.RealMotors)
        self.real_motor_min_idx = min(motor.value for motor in self.robot_cfg.RealMotors)
        self.real_motor_max_idx = max(motor.value for motor in self.robot_cfg.RealMotors)

        self.kNotUsedJoint = self.real_motor_max_idx + 1     # NOT SURE IF THIS IS CORRECT???
                                                        # TODO: confirm that this works with H1
                                                        # and G1 low-level control
        self.dt = kwargs.get("dt", 0.01)
        self.control_decimation = kwargs.get("control_decimation", 1)
        self.send_cmd = kwargs.get("send_cmd", False)
        # ------------------------------- Unitree Model ------------------------------ #
        self.unitree_model = kwargs["unitree_model"]    # g1 or h1
        if self.unitree_model not in ["g1", "h1"]:
            raise ValueError("Invalid unitree humanoid model. Choose either 'g1' or 'h1'.")
        elif self.unitree_model == "h1":
            raise NotImplementedError("Unitree H1 not implemented in the current version.")
        
        # ------------------------------- Control Level ------------------------------ #
        if kwargs["level"] == "high":
            self.control_level = UnitreeControlLevel.HIGH
        elif kwargs["level"] == "low":
            # self.control_level = UnitreeControlLevel.LOW
            raise NotImplementedError("Low level control not implemented in the current version.")
        else:
            raise ValueError("Invalid control level. Choose either 'low' or 'high'.\nNote that low level control is not implemented in the current version.")
        
        # -------------------------------- Robot Setup ------------------------------- #
        self._setup_robot()
        self._setup_subscribers()
        self._setup_publishers()
        self._start_sub_threads()

        # ----------------------------- Robot Parameters ----------------------------- #
        self.cmd_q_lambda = getattr(kwargs, "motor_lambda", 1.0)
        self.motor_kp_normal = getattr(kwargs, "motor_kp_normal", 100.0)
        self.motor_kd_normal = getattr(kwargs, "motor_kd_normal", 3.0)
        self.motor_kp_weak = getattr(kwargs, "motor_kp_weak", 80.0)
        self.motor_kd_weak = getattr(kwargs, "motor_kd_weak", 3.0)
        self.motor_kp_delicate = getattr(kwargs, "motor_kp_delicate", 30.0)
        self.motor_kd_delicate = getattr(kwargs, "motor_kd_delicate", 1.5)
        
        self._initialize_robot_motor_parameters()
        
        # ------------------------------ Robot Cmd Setup ----------------------------- #
        self._initialize_motor_cmd()

        # NOTE: Don't define them here. For the initial `get_feedback`, they should be `None`.
        # self.dof_pos_fbk = np.zeros(self.num_dof)
        # self.dof_vel_fbk = np.zeros(self.num_dof)
        # self.dof_acc_fbk = np.zeros(self.num_dof)

        # self.dof_pos_cmd = np.zeros(self.num_dof)
        # self.dof_vel_cmd = np.zeros(self.num_dof)
        # self.dof_acc_cmd = np.zeros(self.num_dof)
        
        self.reset_upper_body()
    

    def _setup_robot(self):
        ''' Start and setup Unitree humanoid '''

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize()

        # --------------------- High-level control initialization -------------------- #
        if self.control_level == UnitreeControlLevel.HIGH:
            
            if self.unitree_model == "g1":
                # from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
                from spark_agent.real.g1.loco.g1_loco_client import LocoClient
                from spark_agent.real.g1.g1_idl_struct import G1_29_LowState as LowStateDataStruct
                from spark_agent.real.g1.g1_idl_struct import G1_29_ArmSDK_State as CmdDataStruct
                from spark_agent.real.g1.g1_idl_struct import G1_29_HighState as HighStateDataStruct
                self.fsm_id = G1FSMID
            else: # self.unitree_model == "h1":
                # from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
                from spark_agent.real.g1.g1_idl_struct import H1_LowState as LowStateDataStruct
                from spark_agent.real.g1.g1_idl_struct import H1_ArmSDK_State as CmdDataStruct
                from spark_agent.real.g1.g1_idl_struct import H1_HighState as HighStateDataStruct
                self.fsm_id = H1FSMID
            
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            print("Loco Client Initialized.")
            self._initialize_sports_mode()

        # --------------------- Low-level control initialization --------------------- #
        else:   # low level control
            if self.unitree_model == "g1":
                from spark_agent.real.g1.g1_idl_struct import G1_29_LowState as LowStateDataStruct
                from spark_agent.real.g1.g1_idl_struct import G1_29_LowCmd_State as CmdDataStruct
            else: # self.unitree_model == "h1":
                from spark_agent.real.g1.g1_idl_struct import H1_LowState as LowStateDataStruct
                from spark_agent.real.g1.g1_idl_struct import H1_LowCmd_State as CmdDataStruct

            raise NotImplementedError("Low level control not implemented for Unitree humanoids.")

    def _switch_fsm_id(self, 
                       target_id: int, 
                       max_attempts=5, 
                       sleep_time=1.0):
        attempts = 0
        while attempts < max_attempts:
            # import ipdb; ipdb.set_trace()
            code = self.loco_client.SetFsmId(target_id)
            if code == 0:
                time.sleep(1)  # Wait after successful switch
                return True
            attempts += 1
            time.sleep(sleep_time)
        return False
    
    def _initialize_sports_mode(self):
        # ----------------------------------- TODO ----------------------------------- #
        # --------- Find a way to let users know that you need to put the G1 --------- #
        # -------------------- into locked stand mode beforehand. -------------------- #
        # ------------------------------------ --- ----------------------------------- #
        # -- NOTE： This is because G1 loco client doesn't have "GetFsmId" function. -- #
        # ------------------------------------ --- ----------------------------------- #
        ''' 
        Initialize sports mode for Unitree humanoids 
        `ZeroTorque` --> `Damp` --> `LockStand` --> `MainMode`
        '''
        sports_mode_paths = {
            self.fsm_id.ZeroTorque: [self.fsm_id.Damping, self.fsm_id.LockStand, self.fsm_id.MainMode],
            self.fsm_id.Damping: [self.fsm_id.LockStand, self.fsm_id.MainMode],
            self.fsm_id.LockStand: [self.fsm_id.MainMode],
            self.fsm_id.MainMode: []
        }

        sports_mode_paths = {key.value: [v.value for v in values] for key, values in sports_mode_paths.items()}
        
        current_fsm_id = self.loco_client.GetFsmId(0)[1]
        required_sequence = sports_mode_paths.get(current_fsm_id)
        
        print("Current FSM ID: ", current_fsm_id)
        if required_sequence is None:
            # Handle unexpected FSM ID
            return False
        
        for fsm_id in required_sequence:
            print("Switching to FSM ID: ", fsm_id)
            if not self._switch_fsm_id(fsm_id):
                return False
        
        print("Sports mode initialized.")
                
        return True

    def _setup_subscribers(self):

        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_ import SportModeState_

        # --------------------------- LowState subscribers --------------------------- #
        self._low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)
        self._low_state_suber.Init()
        self._low_state_buffer = DataBuffer()

        # --------------------------- HighState subscriber --------------------------- #
        if self.control_level == UnitreeControlLevel.HIGH:
            self._high_state_suber = ChannelSubscriber("rt/odommodestate", SportModeState_)
            self._high_state_suber.Init()
            self._high_state_buffer = DataBuffer()
        else:
            pass

        # ---------------------------- LowCmd subscribers ---------------------------- #
        if self.control_level == UnitreeControlLevel.HIGH:
            self._cmd_suber = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        else:
            self._cmd_suber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._cmd_suber.Init()
        self._cmd_buffer = DataBuffer()
    
    def _state_sub_thread(self):
        while True:
            msg = self._low_state_suber.Read()
            if msg is not None:
                low_state = LowStateDataStruct()
                # --------------------------------- IMU State -------------------------------- #
                low_state.imu_rpy = msg.imu_state.rpy
                low_state.imu_quaternion = msg.imu_state.quaternion
                low_state.imu_gyroscope = msg.imu_state.gyroscope
                low_state.imu_accelerometer = msg.imu_state.accelerometer
                # -------------------------------- Motor State ------------------------------- #
                for id, motor in enumerate(msg.motor_state):
                    low_state.motor_state[id].q = motor.q
                    low_state.motor_state[id].dq = motor.dq
                    low_state.motor_state[id].ddq = motor.ddq
                    low_state.motor_state[id].tau_est = motor.tau_est

                self._low_state_buffer.SetData(low_state)
    
    # ---------------------------------------------------------------------------- #
    #           NOTE: Not working at the moment. Need a firmware update?           #
    # ---------------------------------------------------------------------------- #
        
    # def _high_state_sub_thread(self):
    #     while True:
    #         msg = self._high_state_suber.Read()
    #         if msg is not None:
    #             high_state = HighStateDataStruct()
    #             # --------------------------------- Position --------------------------------- #
    #             high_state.position = msg.position
    #             high_state.velocity = msg.velocity
    #             high_state.yaw_speed = msg.yaw_speed

    #             self._high_state_buffer.SetData(high_state)
    
    # ---------------------------------------------------------------------------- #
    #                         NOTE: Not used at the moment.                        #
    # ---------------------------------------------------------------------------- #

    # def _cmd_sub_thread(self):
    #     while True:
    #         msg = self._cmd_suber.Read()
    #         if msg is not None:
    #             cmd_state = CmdDataStruct()
    #             if self.control_level == UnitreeControlLevel.HIGH:
    #                 cmd_state.arm_sdk_notUsedJoint_q = msg.motor_cmd[self.kNotUsedJoint].q
    #             for id, motor in enumerate(msg.motor_cmd[self.min_motor_idx: self.max_motor_idx+1]):
    #                 cmd_state.motor_cmd[id-self.real_motor_min_idx].q = motor.q
    #                 cmd_state.motor_cmd[id-self.real_motor_min_idx].dq = motor.dq
    #                 cmd_state.motor_cmd[id-self.real_motor_min_idx].kp = motor.kp
    #                 cmd_state.motor_cmd[id-self.real_motor_min_idx].kd = motor.kd
    #                 cmd_state.motor_cmd[id-self.real_motor_min_idx].tau = motor.tau

    #             self._cmd_buffer.SetData(cmd_state)

    def _start_sub_threads(self):
        # ------------------------ Low State subscriber thread ----------------------- #
        threading.Thread(target=self._state_sub_thread, daemon=True).start()

        cnt = 0
        while not self._low_state_buffer.GetData():
            time.sleep(0.1)
            cnt += 1
            if cnt % 10 == 0:
                print("Waiting to receive Unitree state data...")
            if cnt > 100:
                raise TimeoutError("Failed to receive Unitree state data.")

        # ---------------------------------------------------------------------------- #
        #           NOTE: Not working at the moment. Need a firmware update?           #
        # ---------------------------------------------------------------------------- #

        # ----------------------- High State subscriber thread ----------------------- #
        # if self.control_level == UnitreeControlLevel.HIGH:
        #     threading.Thread(target=self._high_state_sub_thread, daemon=True).start()

        #     cnt = 0
        #     while not self._high_state_buffer.GetData():
        #         time.sleep(0.1)
        #         cnt += 1
        #         if cnt % 10 == 0:
        #             print("Waiting to receive Unitree high state data...")
        #         if cnt > 100:
        #             raise TimeoutError("Failed to receive Unitree high state data.")
        
        # ---------------------------------------------------------------------------- #
        #                         NOTE: Not used at the moment.                        #
        # ---------------------------------------------------------------------------- #

        # -------------------------- Command subscriber thread ------------------------ #
        # threading.Thread(target=self._cmd_sub_thread, daemon=True).start()

    def _setup_publishers(self):

        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        if self.control_level == UnitreeControlLevel.HIGH:
            self._cmd_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        else:
            self._cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._cmd_pub.Init()

    def _initialize_motor_cmd(self):

        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        self.cmd_msg = unitree_hg_msg_dds__LowCmd_()
        if self.control_level == UnitreeControlLevel.HIGH:
            self.weight_notUsedJoint = 0.0
    
    def _initialize_robot_motor_parameters(self):
        self.motor_kp = np.zeros(self.num_real_motor)
        self.motor_kd = np.zeros(self.num_real_motor)

        self.set_motor_cmd_parameters()
        
    def emergency_stop(self):
        if self.control_level == UnitreeControlLevel.HIGH:
            self.loco_client.SetFsmId(self.fsm_id.Damping.value)
        else:
            raise NotImplementedError("Emergency stop not implemented for low level control.")

    # TODO: deprecated???
    # TODO: this was used to sync with AVP
    def update_dynamic_states(self, dof_pos=None, dof_vel=None, dof_acc=None):
        
        self.dof_pos[:self.num_real_motor] = self.get_real_motor_q()
        self.dof_pos[self.num_real_motor:] = 0.0 # update lin info later
        self.dof_vel[:self.num_real_motor] = self.get_real_motor_dq()
        self.dof_vel[self.num_real_motor:] = 0.0 # update lin info later
        self.dof_acc[:self.num_real_motor] = self.get_real_motor_ddq()
        self.dof_acc[self.num_real_motor:] = 0.0 # update lin info later

    def reset_upper_body(self):

        
        for _ in range(100):
            
            target_kp = np.zeros_like(self.motor_kp)
            # target_kp[self.robot_cfg.DoFs.WaistPitch] = self.motor_kp[self.robot_cfg.DoFs.WaistPitch]
            # target_kp[self.robot_cfg.DoFs.WaistRoll] = self.motor_kp[self.robot_cfg.DoFs.WaistRoll]
            # target_kp[self.robot_cfg.DoFs.WaistYaw] = self.motor_kp[self.robot_cfg.DoFs.WaistYaw]
            
            target_kd = np.zeros_like(self.motor_kd)
            # target_kd[self.robot_cfg.DoFs.WaistPitch] = self.motor_kd[self.robot_cfg.DoFs.WaistPitch]
            # target_kd[self.robot_cfg.DoFs.WaistRoll] = self.motor_kd[self.robot_cfg.DoFs.WaistRoll]
            # target_kd[self.robot_cfg.DoFs.WaistYaw] = self.motor_kd[self.robot_cfg.DoFs.WaistYaw]
            
            target_q = np.zeros(self.num_real_motor)
            target_dq = np.zeros(self.num_real_motor)
            target_tauff = np.zeros(self.num_real_motor)
            
            target_tauff[self.robot_cfg.DoFs.RightShoulderPitch] = -1.0
            target_tauff[self.robot_cfg.DoFs.LeftShoulderPitch] = -1.0
            target_tauff[self.robot_cfg.DoFs.RightShoulderRoll] = -2.0
            target_tauff[self.robot_cfg.DoFs.LeftShoulderRoll] = 2.0
            target_tauff[self.robot_cfg.DoFs.RightShoulderYaw] = -0.5
            target_tauff[self.robot_cfg.DoFs.LeftShoulderYaw] = 0.5
            target_tauff[self.robot_cfg.DoFs.RightElbow] = -2.0
            target_tauff[self.robot_cfg.DoFs.LeftElbow] = -2.0
            target_tauff[self.robot_cfg.DoFs.RightWristPitch] = -0.2
            target_tauff[self.robot_cfg.DoFs.LeftWristPitch] = -0.2
            
            target_tauff[self.robot_cfg.DoFs.RightWristYaw] = -0.2
            target_tauff[self.robot_cfg.DoFs.LeftWristYaw] = 0.2
            
            # if self.send_cmd:
            self.set_motor_cmd(cmd_q=target_q, cmd_dq=target_dq, cmd_tau=target_tauff, cmd_kp=target_kp, cmd_kd=target_kd, weight=1.0)
            self._cmd_pub.Write(self.cmd_msg)
            
            time.sleep(self.dt)
        
        time.sleep(1.0)
        print("Upper body reset done.")

    def reset_motor_cmd(self):
        self._initialize_motor_cmd()

    def set_motor_cmd_parameters(self, 
                                 motor_kp_normal   = None,
                                 motor_kd_normal   = None,
                                 motor_kp_weak     = None,
                                 motor_kd_weak     = None,
                                 motor_kp_delicate = None,
                                 motor_kd_delicate = None):
        
        if motor_kp_normal is not None:
            self.motor_kp_normal = motor_kp_normal
        if motor_kd_normal is not None:
            self.motor_kd_normal = motor_kd_normal
        if motor_kp_weak is not None:
            self.motor_kp_weak = motor_kp_weak
        if motor_kd_weak is not None:
            self.motor_kd_weak = motor_kd_weak
        if motor_kp_delicate is not None:
            self.motor_kp_delicate = motor_kp_delicate
        if motor_kd_delicate is not None:
            self.motor_kd_delicate = motor_kd_delicate

        for idx, joint in enumerate(self.robot_cfg.RealMotors):
            if joint in self.robot_cfg.DelicateMotor:
                self.motor_kp[idx] = self.motor_kp_delicate
                self.motor_kd[idx] = self.motor_kd_delicate
            elif joint in self.robot_cfg.WeakMotor:
                self.motor_kp[idx] = self.motor_kp_weak
                self.motor_kd[idx] = self.motor_kd_weak
            else:
                self.motor_kp[idx] = self.motor_kp_normal
                self.motor_kd[idx] = self.motor_kd_normal

    def set_motor_cmd(self,
                      cmd_q: np.ndarray,
                      cmd_dq: np.ndarray,
                      cmd_tau: np.ndarray,
                      cmd_kp: np.ndarray,
                      cmd_kd: np.ndarray,
                      weight: float): 
        
        if self.control_level == UnitreeControlLevel.HIGH:
            if weight is not None:
                self.weight_notUsedJoint = max(min(weight, 1.0), 0.0)
            self.cmd_msg.motor_cmd[self.kNotUsedJoint].q = self.weight_notUsedJoint ** 2

        for idx, joint in enumerate(self.robot_cfg.RealMotors):
            joint_pos_min, joint_pos_max = self.robot_cfg.RealMotorPosLimit.get(joint, (-np.inf, np.inf))
            joint_q = (1 - self.cmd_q_lambda) * self.cmd_msg.motor_cmd[joint].q + self.cmd_q_lambda * cmd_q[idx]
            joint_q = np.clip(joint_q, joint_pos_min, joint_pos_max)

            self.cmd_msg.motor_cmd[joint].q = joint_q
            self.cmd_msg.motor_cmd[joint].dq = cmd_dq[idx]
            self.cmd_msg.motor_cmd[joint].tau = cmd_tau[idx]
            self.cmd_msg.motor_cmd[joint].kp = cmd_kp[idx]
            self.cmd_msg.motor_cmd[joint].kd = cmd_kd[idx]

    def send_control(self, command: np.ndarray, **kwargs):
        '''
        Command is consisted of
            - HIGH control level: the motor commands for the upperbody and velocity commands for the locomotion.
            - LOW control level: the motor commands for all joints.
        All control inputs are given as velocity. Thus, they need to be integrated to get the position commands.

        Args:
            control (np.ndarray): Control input for the robot. Always velocity (NOTE: for now).
            cmd_kp (np.ndarray): Proportional gain for the motor control.
            cmd_kd (np.ndarray): Derivative gain for the motor control.
            weight (float): Weight for the not used joint in sports mode.
        '''
        # ---------------------------- Control parameters ---------------------------- #
        target_kp = kwargs.get("cmd_kp", self.motor_kp)
        target_kd = kwargs.get("cmd_kd", self.motor_kd)
        weight = kwargs.get("weight", None)
        
        # ----------------------------- Process commands ----------------------------- #
        # 1. Retrieve the states, `x` using `compose_state`.
        # 2. Retrieve the velocity, `x_dot` from the commands.
        # 3. Integrate to get the desired position for the motors.
        #    Note that unlike the MujocoAgent, the desired position is NOT the same as the 
        #    actual position.
        # 4. When integrating, we should also integrate for the locomotion velocity.
        #    This will act as the desired `dof_pos_cmd` for the locomotion.
        #    Of course, this is NOT the same as the actual locomotion position.
        # 5. Set the position and velocity commands to `self.dof_pos_cmd` and `self.dof_vel_cmd`.
        # 6. Send the commands.

        x = self.compose_cmd_state()
        x_dot = self.robot_cfg.dynamics_xdot(x, command)
        
        # integration
        x += x_dot * self.dt * self.control_decimation

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        # if self.robot_cfg.Control.vLinearX
        # vx=self.dof_vel_cmd[], 
        # vy=self.dof_vel_cmd[self.robot_cfg.Control.vLinearY],
        # vyaw=self.dof_vel_cmd[self.robot_cfg.Control.vRotYaw
        vx, vy, vyaw = 0.0, 0.0, 0.0
        for control in self.robot_cfg.Control:
            if control.name == "vLinearX":
                vx = self.dof_vel_cmd[control.value]
            if control.name == "vLinearY":
                vy = self.dof_vel_cmd[control.value]
            if control.name == "vRotYaw":
                vyaw = self.dof_vel_cmd[control.value]
                
        if self.send_cmd:
            if np.isnan(self.dof_pos_cmd).any() or np.isinf(self.dof_pos_cmd).any():
                print("NAN or INF in dof_pos_cmd")
                return
            # self.dof_pos_cmd[self.robot_cfg.DoFs.LeftWristPitch] = 0.0
            # self.dof_pos_cmd[self.robot_cfg.DoFs.LeftWristYaw] = 0.0
            # self.dof_pos_cmd[self.robot_cfg.DoFs.RightWristPitch] = 0.0
            # self.dof_pos_cmd[self.robot_cfg.DoFs.RightWristYaw] = 0.0
            # ------------------------------- Motor control ------------------------------ #
            self.set_motor_cmd(cmd_q=self.dof_pos_cmd[:self.num_real_motor], 
                            cmd_dq=np.zeros(self.num_real_motor), 
                            cmd_tau=np.zeros(self.num_real_motor), 
                            cmd_kp=target_kp, 
                            cmd_kd=target_kd, 
                            weight=weight)
            self._cmd_pub.Write(self.cmd_msg)
            # ------------------------------- Loco control ------------------------------- #
            # if self.control_level == UnitreeControlLevel.HIGH:
            #     self.loco_client.Move(vx=vx, 
            #                         vy=vy,
            #                         vyaw=vyaw,
            #                         continous_move=True)

    def compose_cmd_state(self):
        '''
        Compose the state for the robot agent.
        Since the real G1 agent is controlled with velocity, this is the positions.
        '''
        x = self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)
        
        return x

    def get_imu_rpy(self):
        return np.array(self._low_state_buffer.GetData().imu_rpy)

    def get_imu_quaternion(self):
        return np.array(self._low_state_buffer.GetData().imu_quaternion)

    def get_imu_gyroscope(self):
        return np.array(self._low_state_buffer.GetData().imu_gyroscope)

    def get_imu_accelerometer(self):
        return np.array(self._low_state_buffer.GetData().imu_accelerometer)
    
    def get_all_motor_q(self):
        '''
        Motor position (q) for all motors, regardless of the control level and whether the motor is used or not.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].q for id in range(self.robot_cfg.NumTotalMotors)
        ])
    
    def get_all_motor_dq(self):
        '''
        Motor velocity (dq) for all motors, regardless of the control level and whether the motor is used or not.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].dq for id in range(self.robot_cfg.NumTotalMotors)
        ])
    
    def get_all_motor_ddq(self):
        '''
        Motor acceleration (ddq) for all motors, regardless of the control level and whether the motor is used or not.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].ddq for id in range(self.robot_cfg.NumTotalMotors)
        ])

    def get_real_motor_q(self):
        '''
        Motor position (q) for real motors only.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].q for id in range(self.real_motor_min_idx, self.real_motor_max_idx+1)
        ])
    
    def get_real_motor_dq(self):
        '''
        Motor velocity (dq) for real motors only.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].dq for id in range(self.real_motor_min_idx, self.real_motor_max_idx+1)
        ])
    
    def get_real_motor_ddq(self):
        '''
        Motor acceleration (ddq) for real motors only.
        '''
        return np.array([
            self._low_state_buffer.GetData().motor_state[id].ddq for id in range(self.real_motor_min_idx, self.real_motor_max_idx+1)
        ])
    
    def get_sports_position(self):
        return np.array(self._high_state_buffer.GetData().position)
    
    def get_sports_velocity(self):
        return np.array(self._high_state_buffer.GetData().velocity)
    
    def get_sports_yaw_speed(self):
        return self._high_state_buffer.GetData().yaw_speed
    
    def get_loco_position(self):
        # TODO If using mocap, get it from mocap. Otherwise, get it from the robot.
        # TODO For now, just return zeros.
        return np.zeros(3)
    
    def get_loco_velocity(self):
        # TODO If using mocap, get it from mocap. Otherwise, get it from the robot.
        # TODO For now, just return zeros.
        return np.zeros(3)

    def get_feedback(self):
        
        ret = {}

        # ------------------------------ Robot Base Frame ----------------------------- #
        robot_base_frame = np.eye(4)
        robot_base_frame[2, 3] = 0.793

        ret["robot_base_frame"] = robot_base_frame

        # ------------------------------ State Feedback ------------------------------ #
        real_robot_qpos = self.get_all_motor_q()
        real_robot_qpvel = self.get_all_motor_dq()
        # loco_q = self.get_loco_position()
        # real_robot_qpos = np.concatenate((real_robot_qpos, loco_q))
        
        dof_pos_fbk = np.zeros(self.num_dof)
        dof_vel_fbk = np.zeros(self.num_dof)
        for dof in self.robot_cfg.DoFs:
            real_dof = self.robot_cfg.DoF_to_RealDoF[dof]
            dof_pos_fbk[dof] = real_robot_qpos[real_dof]
            dof_vel_fbk[dof] = real_robot_qpvel[real_dof]

        ret["dof_pos_fbk"] =dof_pos_fbk
        ret["dof_vel_fbk"] = dof_vel_fbk

        # ------------------------------- Robot Command ------------------------------ #
        if self.dof_pos_cmd is None:
            self.dof_pos_cmd = dof_pos_fbk
        if self.dof_vel_cmd is None:
            self.dof_vel_cmd = np.zeros(self.num_dof)
        if self.dof_acc_cmd is None:
            self.dof_acc_cmd = np.zeros(self.num_dof)
        
        ret["dof_pos_cmd"] = self.dof_pos_cmd
        ret["dof_vel_cmd"] = self.dof_vel_cmd
        ret["dof_acc_cmd"] = self.dof_acc_cmd

        # ------------------------------ Dynamics State ------------------------------ #
        ret["state"] = self.compose_cmd_state() # NOTE: Not sure if this is all we need.

        # # ------------------------ Retrieve Unitree state data ----------------------- #
        # imu_rpy = self.get_imu_rpy()
        # imu_quaternion = self.get_imu_quaternion()
        # imu_gyroscope = self.get_imu_gyroscope()
        # imu_accelerometer = self.get_imu_accelerometer()

        # real_motor_q = self.get_real_motor_q()
        # real_motor_dq = self.get_real_motor_dq()
        # real_motor_ddq = self.get_real_motor_ddq()

        # if self.control_level == UnitreeControlLevel.HIGH:
        #     sports_position = self.get_sports_position()
        #     sports_velocity = self.get_sports_velocity()
        #     sports_yaw_speed = self.get_sports_yaw_speed()
        #     sports_loco_velocity = np.array(sports_velocity[:2], sports_yaw_speed)
        
        # # ----------------------------- Feedback dof pos ----------------------------- #
        # dof_pos_fbk = np.zeros(self.num_dof)
        # if self.control_level == UnitreeControlLevel.HIGH:
        #     ret["dof_pos"] = np.concatenate((real_motor_q, sports_position))
        
        return ret
        
        
if __name__ == "__main__":
    from spark_robot import G1FixedBaseDynamic1Config
    g1_real_agent = G1RealAgent(robot_cfg = G1FixedBaseDynamic1Config(), unitree_model="g1", level="high", send_cmd=True)
    fsm_id, data = g1_real_agent.loco_client.GetFsmId(0)
    print("FSM ID: ", data)
    
    # code = g1_real_agent.loco_client.HighStand()
    # print(code)
    
    g1_real_agent.reset_upper_body()
    g1_real_agent.get_feedback()
    while True:
        g1_real_agent.send_control(np.zeros(17), control_type="pos")
        # g1_real_agent.get_feedback()
    
    # while True:
    #     print(g1_real_agent.get_imu_rpy())
    #     print(g1_real_agent.get_imu_quaternion())
    #     print(g1_real_agent.get_imu_gyroscope())
    #     print(g1_real_agent.get_imu_accelerometer())
    #     print(g1_real_agent.get_real_motor_q())
    #     print(g1_real_agent.get_real_motor_dq())
    #     print(g1_real_agent.get_real_motor_ddq())
    #     # target_q = np.zeros(17)
    #     # g1_real_agent.send_control(target_q)
    #     # time.sleep(0.1)
    
    