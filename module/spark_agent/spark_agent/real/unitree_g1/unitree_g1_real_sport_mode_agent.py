import os
import sys
import time
from enum import IntEnum
import threading
import numpy as np
import select
import termios
import tty
import atexit

from spark_agent.base.base_agent import BaseAgent
from spark_agent.dynamics import DynamicsExecutor
from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
    DataBuffer,
    MotorState,
    MotorCmd,
    LowStateDataStruct,
    CmdDataStruct,
    HighStateDataStruct,
)
from spark_robot import RobotConfig
from scipy.spatial.transform import Rotation as R


class UnitreeControlLevel(IntEnum):
    LOW = 0  # low level control (`lowcmd`: all joints)
    HIGH = 1  # sports mode (`armsdk` + `LocoClient`)


class UnitreeG1FSMID(IntEnum):
    ZeroTorque = 0
    Damping = 1
    Squat = 2
    Sit = 3
    LockStand = 4
    MainMode = 200


class UnitreeH1FSMID(IntEnum):
    ZeroTorque = 0
    Damping = 1
    LockStand = 2
    MainMode = 204


class UnitreeG1RealSportModeAgent(BaseAgent):
    """
    Unitree Humanoid Agent.
    Current version works for Unitree G1 humanoid.
    Future versions will include Unitree H1 humanoid.

    Args:
        robot_cfg (RobotConfig): Robot configuration
        unitree_model (str): Unitree humanoid model. Choose either 'g1' or 'h1'.
        level (str): Control level. Choose either 'low' or 'high'. High level is sports mode.
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs):
        super().__init__(robot_cfg)
        try:
            from unitree_sdk2py.utils.crc import CRC
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "UnitreeG1RealAgent requires the optional Unitree SDK. "
                "Install it with './install.sh --profile mujoco --unitree-sdk'."
            ) from exc
        self.crc = CRC()

        # ----------------------------- Agent Parameters ----------------------------- #
        self.num_real_motor = len(self.robot_cfg.RealMotors)
        self.real_motor_min_idx = min(motor.value for motor in self.robot_cfg.RealMotors)
        self.real_motor_max_idx = max(motor.value for motor in self.robot_cfg.RealMotors)
        self.real_motor_to_index = {
            motor: idx for idx, motor in enumerate(self.robot_cfg.RealMotors)
        }
        self.kNotUsedJoint = self.real_motor_max_idx + 1  # NOT SURE IF THIS IS CORRECT???
        # TODO: confirm that this works with H1
        # and G1 low-level control
        self.dt = kwargs.get("dt", 0.01)
        self.control_decimation = kwargs.get("control_decimation", 1)
        self.dynamics_model = self.robot_cfg.create_dynamics_model()
        self.dynamics_executor = DynamicsExecutor(
            self.dynamics_model,
            dt=self.dt * self.control_decimation,
            integrator=kwargs.get("model_integrator"),
            substeps=kwargs.get("model_substeps", 1),
            seed=kwargs.get("dynamics_seed"),
        )
        self.send_cmd = kwargs.get("send_cmd", False)
        self.unitree_network_interface = kwargs.get(
            "unitree_network_interface", kwargs.get("net", "")
        )
        self.release_motion_mode_before_loco = bool(
            kwargs.get("release_motion_mode_before_loco", False)
        )
        self.select_motion_mode_before_loco = kwargs.get("select_motion_mode_before_loco", "ai")
        self.large_torque_threshold = float(kwargs.get("large_torque_threshold", 100.0))
        self.enable_large_torque_protection = bool(
            kwargs.get("enable_large_torque_protection", True)
        )
        self.reset_upper_body_duration = float(kwargs.get("reset_upper_body_duration", 2.0))
        self.reset_upper_body_hold_duration = float(
            kwargs.get("reset_upper_body_hold_duration", 0.5)
        )
        self.reset_upper_body_weight = float(kwargs.get("reset_upper_body_weight", 1.0))
        self.reset_upper_body_on_reset = bool(kwargs.get("reset_upper_body_on_reset", True))
        self.fsm_switch_max_attempts = int(kwargs.get("fsm_switch_max_attempts", 10))
        self.fsm_switch_sleep_time = float(kwargs.get("fsm_switch_sleep_time", 1.0))
        self.lockstand_settle_duration = float(kwargs.get("lockstand_settle_duration", 5.0))
        self.real_joint_debug_rate = float(kwargs.get("real_joint_debug_rate", 2.0))
        self._last_real_joint_debug_time = 0.0
        self.enable_keyboard_control = bool(kwargs.get("enable_keyboard_control", True))
        self.enable_terminal_keyboard_control = bool(
            kwargs.get("enable_terminal_keyboard_control", self.enable_keyboard_control)
        )
        self.manual_step_size = float(kwargs.get("manual_movement_step_size", 0.02))
        self.manual_step_size_min = float(kwargs.get("manual_movement_step_size_min", 0.001))
        self.manual_step_size_max = float(kwargs.get("manual_movement_step_size_max", 0.5))
        self.manual_step_size_delta = float(kwargs.get("manual_movement_step_size_delta", 0.002))
        self.translate_in_local_frame = False
        self.left_goal_debug_frame = np.eye(4)
        self.right_goal_debug_frame = np.eye(4)
        self.base_goal_debug_frame = np.eye(4)
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self.debug_object = self.right_goal_debug_frame
        self._terminal_keyboard_stop = threading.Event()
        self._terminal_keyboard_thread = None
        self._terminal_settings = None

        self.update_mode_machine_ = False
        self.mode_machine_ = 0
        self.counter_ = 0
        self.sports_mode_initialized = False
        # ------------------------------- Unitree Model ------------------------------ #
        self.unitree_model = kwargs["unitree_model"]  # g1 or h1
        if self.unitree_model not in ["g1", "h1"]:
            raise ValueError("Invalid unitree humanoid model. Choose either 'g1' or 'h1'.")
        elif self.unitree_model == "h1":
            raise NotImplementedError("Unitree H1 not implemented in the current version.")

        # ------------------------------- Control Level ------------------------------ #
        if kwargs["level"] == "high":
            self.control_level = UnitreeControlLevel.HIGH
        elif kwargs["level"] == "low":
            self.control_level = UnitreeControlLevel.LOW
            # raise NotImplementedError("Low level control not implemented in the current version.")
        else:
            raise ValueError(
                "Invalid control level. Choose either 'low' or 'high'.\nNote that low level control is not implemented in the current version."
            )

        # -------------------------------- Robot Setup ------------------------------- #
        self._setup_robot()
        self._setup_subscribers()
        self._setup_publishers()
        self._start_sub_threads()

        # ----------------------------- Robot Parameters ----------------------------- #
        self.cmd_q_lambda = kwargs.get("motor_lambda", 1.0)
        self.motor_kp_normal = kwargs.get("motor_kp_normal", 100.0)
        self.motor_kd_normal = kwargs.get("motor_kd_normal", 3.0)
        self.motor_kp_weak = kwargs.get("motor_kp_weak", 80.0)
        self.motor_kd_weak = kwargs.get("motor_kd_weak", 3.0)
        self.motor_kp_delicate = kwargs.get("motor_kp_delicate", 30.0)
        self.motor_kd_delicate = kwargs.get("motor_kd_delicate", 1.5)

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

        if kwargs.get("reset_upper_body_on_init", True):
            self.reset_upper_body()

        self._start_terminal_keyboard_control()

    def reset(self, agent_reset_info):
        self.dynamics_executor.clear()
        if self.reset_upper_body_on_reset:
            self.reset_upper_body()

    def _setup_robot(self):
        """Start and setup Unitree humanoid"""

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        if self.unitree_network_interface:
            ChannelFactoryInitialize(0, self.unitree_network_interface)
        else:
            ChannelFactoryInitialize(0)

        # --------------------- High-level control initialization -------------------- #
        if self.control_level == UnitreeControlLevel.HIGH:
            if self.unitree_model == "g1":
                # from unitree_sdk2py.unitree_g1.loco.unitree_g1_loco_client import LocoClient
                from spark_agent.real.unitree_g1.loco.unitree_g1_loco_client import LocoClient
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    G1_29_LowState as LowStateDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    G1_29_ArmSDK_State as CmdDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    G1_29_HighState as HighStateDataStruct,
                )

                self.fsm_id = UnitreeG1FSMID
            else:  # self.unitree_model == "h1":
                # from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    H1_LowState as LowStateDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    H1_ArmSDK_State as CmdDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    H1_HighState as HighStateDataStruct,
                )

                self.fsm_id = UnitreeH1FSMID

            self._prepare_motion_mode_before_loco()
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(10.0)
            self.loco_client.Init()
            print("Loco Client Initialized.")
            self.sports_mode_initialized = self._initialize_sports_mode()

        # --------------------- Low-level control initialization --------------------- #
        else:  # low level control
            if self.unitree_model == "g1":
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    G1_29_LowState as LowStateDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    G1_29_LowCmd_State as CmdDataStruct,
                )
            else:  # self.unitree_model == "h1":
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    H1_LowState as LowStateDataStruct,
                )
                from spark_agent.real.unitree_g1.unitree_g1_idl_struct import (
                    H1_LowCmd_State as CmdDataStruct,
                )

    def _prepare_motion_mode_before_loco(self):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        print("MotionSwitcher CheckMode:", status, result)
        if status != 0 or result is None:
            return

        if self.release_motion_mode_before_loco and result.get("name"):
            print("MotionSwitcher ReleaseMode()")
            code, _ = self.msc.ReleaseMode()
            print("MotionSwitcher ReleaseMode code:", code)
            time.sleep(1.0)
            status, result = self.msc.CheckMode()
            print("MotionSwitcher CheckMode after release:", status, result)

        if self.select_motion_mode_before_loco:
            print(f"MotionSwitcher SelectMode({self.select_motion_mode_before_loco!r})")
            code, _ = self.msc.SelectMode(self.select_motion_mode_before_loco)
            print("MotionSwitcher SelectMode code:", code)
            time.sleep(1.0)
            status, result = self.msc.CheckMode()
            print("MotionSwitcher CheckMode after select:", status, result)

    def _switch_fsm_id(self, target_id: int, max_attempts=None, sleep_time=None):
        max_attempts = self.fsm_switch_max_attempts if max_attempts is None else max_attempts
        sleep_time = self.fsm_switch_sleep_time if sleep_time is None else sleep_time
        attempts = 0
        while attempts < max_attempts:
            result = self.loco_client.SetFsmId(target_id)
            code = result[0] if isinstance(result, tuple) else result
            if code == 0:
                time.sleep(sleep_time)
                read_code, current_fsm_id = self.loco_client.GetFsmId(0)
                if read_code == 0 and current_fsm_id == target_id:
                    return True
                print(
                    f"FSM switch request accepted, but current FSM is {current_fsm_id}; "
                    f"waiting for {target_id}."
                )
            else:
                time.sleep(sleep_time)
            attempts += 1
        read_code, current_fsm_id = self.loco_client.GetFsmId(0)
        print(
            f"Failed to switch to FSM ID {target_id}. "
            f"Last GetFsmId result: code={read_code}, fsm_id={current_fsm_id}."
        )
        return False

    def _initialize_sports_mode(self):
        # ----------------------------------- TODO ----------------------------------- #
        # --------- Find a way to let users know that you need to put the G1 --------- #
        # -------------------- into locked stand mode beforehand. -------------------- #
        # ------------------------------------ --- ----------------------------------- #
        # -- NOTE： This is because G1 loco client doesn't have "GetFsmId" function. -- #
        # ------------------------------------ --- ----------------------------------- #
        """
        Initialize sports mode for Unitree humanoids
        `ZeroTorque` --> `Damp` --> `LockStand` --> `MainMode`
        """
        sports_mode_paths = {
            self.fsm_id.ZeroTorque: [
                self.fsm_id.Damping,
                self.fsm_id.LockStand,
                self.fsm_id.MainMode,
            ],
            self.fsm_id.Damping: [self.fsm_id.LockStand, self.fsm_id.MainMode],
            self.fsm_id.LockStand: [self.fsm_id.MainMode],
            self.fsm_id.MainMode: [],
        }

        sports_mode_paths = {
            key.value: [v.value for v in values] for key, values in sports_mode_paths.items()
        }

        code, current_fsm_id = self.loco_client.GetFsmId(0)
        required_sequence = sports_mode_paths.get(current_fsm_id)

        print("Current FSM ID: ", current_fsm_id)
        if code != 0:
            print(
                "Failed to query sport-mode FSM ID. Check the network interface and sport-mode service."
            )
            return False
        if required_sequence is None:
            # Handle unexpected FSM ID
            return False

        if not self.send_cmd:
            if required_sequence:
                print("Sport mode would require FSM switches:", required_sequence)
            print("send_cmd=False; skipping sport-mode FSM switches.")
            return True

        for fsm_id in required_sequence:
            print("Switching to FSM ID: ", fsm_id)
            if (
                fsm_id == self.fsm_id.MainMode.value
                and current_fsm_id == self.fsm_id.LockStand.value
            ):
                print(
                    f"Waiting {self.lockstand_settle_duration:.1f}s in LockStand before MainMode."
                )
                time.sleep(self.lockstand_settle_duration)
            if not self._switch_fsm_id(fsm_id):
                return False
            current_fsm_id = fsm_id

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
                if not self.update_mode_machine_:
                    self.mode_machine_ = msg.mode_machine
                    self.update_mode_machine_ = True

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

    def _start_terminal_keyboard_control(self):
        if not self.enable_terminal_keyboard_control:
            return
        if not sys.stdin.isatty():
            print("[real keyboard] stdin is not a TTY; terminal keyboard control disabled.")
            return
        if self._terminal_keyboard_thread is not None:
            return
        try:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            atexit.register(self._restore_terminal_settings)
        except Exception as exc:
            print(f"[real keyboard] failed to enable terminal keyboard control: {exc}")
            return
        self._terminal_keyboard_thread = threading.Thread(
            target=self._terminal_keyboard_loop, daemon=True
        )
        self._terminal_keyboard_thread.start()
        print(
            "[real keyboard] enabled. Select: o=right, p=left, b=base. "
            "Move: arrows or wasd, e/q z, 2/3 yaw, 4/5 pitch, 6/7 roll, +/- step, n local/world."
        )

    def _restore_terminal_settings(self):
        if self._terminal_settings is None:
            return
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
        except Exception:
            pass
        self._terminal_settings = None

    def _terminal_keyboard_loop(self):
        while not self._terminal_keyboard_stop.is_set():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    seq = ch + sys.stdin.read(2)
                    self._handle_terminal_key(seq)
                else:
                    self._handle_terminal_key(ch)
            except Exception as exc:
                print(f"[real keyboard] keyboard loop stopped: {exc}")
                break

    def _ensure_debug_object(self):
        if self.debug_object is None:
            self.debug_object = self.right_goal_debug_frame

    def _selected_goal_debug_side(self):
        if self.debug_object is self.right_goal_debug_frame:
            return "right"
        if self.debug_object is self.left_goal_debug_frame:
            return "left"
        return None

    def _set_selected_gripper_debug_state(self, closed: bool):
        side = self._selected_goal_debug_side()
        if side == "right":
            self.right_gripper_debug_state = bool(closed)
        elif side == "left":
            self.left_gripper_debug_state = bool(closed)

    def _adjust_manual_step_size(self, delta: float):
        self.manual_step_size = float(
            np.clip(
                self.manual_step_size + float(delta),
                self.manual_step_size_min,
                self.manual_step_size_max,
            )
        )
        print(f"[real keyboard] manual movement step size = {self.manual_step_size:.4f} m")

    def _translate_debug_object(self, d_world):
        self._ensure_debug_object()
        d_world = np.asarray(d_world, dtype=float).reshape(3)
        if self.translate_in_local_frame:
            d = self.debug_object[:3, :3] @ d_world
        else:
            d = d_world
        self.debug_object[:3, 3] += d

    def _rotate_debug_object(self, rpy):
        self._ensure_debug_object()
        rotation = R.from_euler("xyz", rpy, degrees=False)
        self.debug_object[:3, :3] @= rotation.as_matrix()

    def _handle_terminal_key(self, key):
        if not self.enable_keyboard_control:
            return
        self._ensure_debug_object()
        k = key.lower() if isinstance(key, str) and len(key) == 1 else key
        s = self.manual_step_size

        if key == "\x1b[C" or k == "d":
            self._translate_debug_object([0, +s, 0])
        elif key == "\x1b[D" or k == "a":
            self._translate_debug_object([0, -s, 0])
        elif key == "\x1b[A" or k == "w":
            self._translate_debug_object([-s, 0, 0])
        elif key == "\x1b[B" or k == "s":
            self._translate_debug_object([+s, 0, 0])
        elif k == "e":
            self._translate_debug_object([0, 0, +s])
        elif k == "q":
            self._translate_debug_object([0, 0, -s])
        elif k == "2":
            self._rotate_debug_object([0.0, 0.0, +0.1])
        elif k == "3":
            self._rotate_debug_object([0.0, 0.0, -0.1])
        elif k == "4":
            self._rotate_debug_object([0.0, +0.1, 0.0])
        elif k == "5":
            self._rotate_debug_object([0.0, -0.1, 0.0])
        elif k == "6":
            self._rotate_debug_object([+0.1, 0.0, 0.0])
        elif k == "7":
            self._rotate_debug_object([-0.1, 0.0, 0.0])
        elif k in ("-", "_"):
            self._adjust_manual_step_size(-self.manual_step_size_delta)
        elif k in ("=", "+"):
            self._adjust_manual_step_size(self.manual_step_size_delta)
        elif k == "o":
            self.debug_object = self.right_goal_debug_frame
            print("[real keyboard] selected right arm goal")
        elif k == "p":
            self.debug_object = self.left_goal_debug_frame
            print("[real keyboard] selected left arm goal")
        elif k == "b":
            self.debug_object = self.base_goal_debug_frame
            print("[real keyboard] selected base goal")
        elif k == "[":
            self._set_selected_gripper_debug_state(False)
        elif k == "]":
            self._set_selected_gripper_debug_state(True)
        elif k == "n":
            self.translate_in_local_frame = not self.translate_in_local_frame
            print(
                f"[real keyboard] translate frame = {'LOCAL' if self.translate_in_local_frame else 'WORLD'}"
            )

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
        else:
            self.weight_notUsedJoint = 0.0

    def _initialize_robot_motor_parameters(self):
        self.motor_kp = np.zeros(self.num_real_motor)
        self.motor_kd = np.zeros(self.num_real_motor)

        self.set_motor_cmd_parameters()
        self.tau_record = np.zeros(self.num_real_motor, dtype=np.float32)

    # def LowStateHandler(self, msg):
    #     self.low_state = msg

    #     if self.update_mode_machine_ == False:
    #         self.mode_machine_ = self.low_state.mode_machine
    #         self.update_mode_machine_ = True

    # self.counter_ +=1
    # if (self.counter_ % 500 == 0) :
    #     self.counter_ = 0
    #     print(self.low_state.imu_state.rpy)

    def emergency_stop(self):
        if self.control_level == UnitreeControlLevel.HIGH:
            self.loco_client.SetFsmId(self.fsm_id.Damping.value)
        else:
            raise NotImplementedError("Emergency stop not implemented for low level control.")

    # TODO: deprecated???
    # TODO: this was used to sync with AVP
    def update_dynamic_states(self, dof_pos=None, dof_vel=None, dof_acc=None):

        self.dof_pos[: self.num_real_motor] = self.get_real_motor_q()
        self.dof_pos[self.num_real_motor :] = 0.0  # update lin info later
        self.dof_vel[: self.num_real_motor] = self.get_real_motor_dq()
        self.dof_vel[self.num_real_motor :] = 0.0  # update lin info later
        self.dof_acc[: self.num_real_motor] = self.get_real_motor_ddq()
        self.dof_acc[self.num_real_motor :] = 0.0  # update lin info later

    def reset_upper_body(self):
        print("Resetting upper body gently.")
        init_q = self.get_real_motor_q()
        target_q = init_q.copy()
        upper_body_indices = self._upper_body_real_motor_indices()

        for idx in upper_body_indices:
            motor = list(self.robot_cfg.RealMotors)[idx]
            if hasattr(self.robot_cfg.DoFs, motor.name):
                dof = getattr(self.robot_cfg.DoFs, motor.name)
                target_q[idx] = self.robot_cfg.DefaultDoFVal[dof]
            else:
                target_q[idx] = 0.0

        upper_body_motors = list(self.robot_cfg.RealMotors)
        print(
            f"arm_sdk enable weight={self.reset_upper_body_weight:.3f}; "
            f"kNotUsedJoint={self.kNotUsedJoint}; "
            f"enable_q={self.reset_upper_body_weight**2:.3f}"
        )
        print("Upper-body reset targets:")
        for idx in upper_body_indices:
            motor = upper_body_motors[int(idx)]
            print(f"  {motor.name:>20s}: current={init_q[idx]:+.4f} target={target_q[idx]:+.4f}")

        total_time = max(self.reset_upper_body_duration, self.dt)
        num_step = max(1, int(total_time / self.dt))
        target_dq = np.zeros(self.num_real_motor)
        target_tauff = np.zeros(self.num_real_motor)
        reset_kp = np.zeros(self.num_real_motor)
        reset_kd = np.zeros(self.num_real_motor)
        reset_kp[upper_body_indices] = self.motor_kp[upper_body_indices]
        reset_kd[upper_body_indices] = self.motor_kd[upper_body_indices]

        for i in range(num_step):
            alpha = (i + 1) / num_step
            cmd_q = init_q.copy()
            cmd_q[upper_body_indices] = (
                init_q[upper_body_indices] * (1.0 - alpha) + target_q[upper_body_indices] * alpha
            )

            self.set_motor_cmd(
                cmd_q=cmd_q,
                cmd_dq=target_dq,
                cmd_tau=target_tauff,
                cmd_kp=reset_kp,
                cmd_kd=reset_kd,
                weight=self.reset_upper_body_weight,
            )

            if self.send_cmd:
                self._send_cmd(self.cmd_msg)

            time.sleep(self.dt)

        hold_steps = max(0, int(self.reset_upper_body_hold_duration / self.dt))
        for _ in range(hold_steps):
            self.set_motor_cmd(
                cmd_q=target_q,
                cmd_dq=target_dq,
                cmd_tau=target_tauff,
                cmd_kp=reset_kp,
                cmd_kd=reset_kd,
                weight=self.reset_upper_body_weight,
            )
            if self.send_cmd:
                self._send_cmd(self.cmd_msg)
            time.sleep(self.dt)

        final_q = self.get_real_motor_q()
        reset_error = np.abs(final_q[upper_body_indices] - target_q[upper_body_indices])
        self._sync_dof_cmd_from_real_motor_q(target_q)
        print(
            "Upper-body reset max position error: "
            f"{reset_error.max():.4f} rad; mean error: {reset_error.mean():.4f} rad."
        )
        print("Upper body reset done.")

    def _sync_dof_cmd_from_real_motor_q(self, real_motor_q):
        real_motor_q = np.asarray(real_motor_q, dtype=float).reshape(-1)
        dof_pos_cmd = np.zeros(self.num_dof, dtype=float)
        for dof in self.robot_cfg.DoFs:
            default = self.robot_cfg.DefaultDoFVal.get(dof, 0.0)
            real_dof = self.robot_cfg.DoF_to_RealDoF.get(dof, None)
            real_idx = self.real_motor_to_index.get(real_dof, None)
            dof_pos_cmd[dof] = real_motor_q[real_idx] if real_idx is not None else default

        self.dof_pos_cmd = dof_pos_cmd
        self.dof_vel_cmd = np.zeros(self.num_dof, dtype=float)
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)

    def _real_motor_cmd_from_dof_pos(self, dof_pos, fallback=None):
        if fallback is None:
            fallback = self.get_real_motor_q()
        cmd_q = np.asarray(fallback, dtype=float).reshape(self.num_real_motor).copy()
        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1)

        if dof_pos.shape[0] != self.num_dof:
            return self._values_in_motor_order(dof_pos, cmd_q)

        for dof in self.robot_cfg.DoFs:
            real_dof = self.robot_cfg.DoF_to_RealDoF.get(dof, None)
            real_idx = self.real_motor_to_index.get(real_dof, None)
            if real_idx is not None:
                cmd_q[real_idx] = dof_pos[dof]
        return cmd_q

    def _dof_target_from_action_info(self, action_info):
        if not isinstance(action_info, dict):
            return None
        for key in ("dof_pos_target", "target_dof_pos"):
            if key in action_info:
                target = np.asarray(action_info[key], dtype=float).reshape(-1)
                if target.shape[0] == self.num_dof:
                    return target
        return None

    def _maybe_print_real_joint_tracking_debug(self, action_info, cmd_q):
        if self.real_joint_debug_rate <= 0.0:
            return
        now = time.time()
        if now - self._last_real_joint_debug_time < 1.0 / self.real_joint_debug_rate:
            return
        self._last_real_joint_debug_time = now

        current_q = self.get_real_motor_q()
        target_dof_q = self._dof_target_from_action_info(action_info)
        if target_dof_q is None:
            target_q = self._real_motor_cmd_from_dof_pos(self.dof_pos_cmd, fallback=current_q)
            target_label = "dof_pos_cmd"
        else:
            target_q = self._real_motor_cmd_from_dof_pos(target_dof_q, fallback=current_q)
            target_label = "dof_pos_target"

        error_q = target_q - current_q
        max_error = float(np.max(np.abs(error_q))) if error_q.size else 0.0
        ik_success = action_info.get("ik_success", None) if isinstance(action_info, dict) else None
        print(
            f"\n[real arm joint tracking] source={target_label} "
            f"ik_success={ik_success} max_abs_error={max_error:.4f} rad"
        )
        for idx, motor in enumerate(self.robot_cfg.RealMotors):
            print(
                f"{motor.name:>20s}  "
                f"target={target_q[idx]:+.4f}  "
                f"current={current_q[idx]:+.4f}  "
                f"error={error_q[idx]:+.4f}  "
                f"cmd={cmd_q[idx]:+.4f}"
            )

    def _upper_body_real_motor_indices(self):
        upper_body_tokens = ("Waist", "Shoulder", "Elbow", "Wrist")
        indices = [
            idx
            for idx, motor in enumerate(self.robot_cfg.RealMotors)
            if any(token in motor.name for token in upper_body_tokens)
        ]
        if not indices:
            raise RuntimeError("No upper-body motors found in robot_cfg.RealMotors.")
        return np.asarray(indices, dtype=int)

    def reset_motor_cmd(self):
        self._initialize_motor_cmd()

    def set_motor_cmd_parameters(
        self,
        motor_kp_normal=None,
        motor_kd_normal=None,
        motor_kp_weak=None,
        motor_kd_weak=None,
        motor_kp_delicate=None,
        motor_kd_delicate=None,
    ):

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

        if self.num_real_motor == 29:
            self.motor_kp = np.array(
                [
                    60,
                    60,
                    60,
                    100,
                    40,
                    40,  # legs
                    60,
                    60,
                    60,
                    100,
                    40,
                    40,  # legs
                    60,
                    40,
                    40,  # waist
                    40,
                    40,
                    40,
                    40,
                    40,
                    40,
                    40,  # arms
                    40,
                    40,
                    40,
                    40,
                    40,
                    40,
                    40,  # arms
                ],
                dtype=float,
            )

            self.motor_kd = np.array(
                [
                    1,
                    1,
                    1,
                    2,
                    1,
                    1,  # legs
                    1,
                    1,
                    1,
                    2,
                    1,
                    1,  # legs
                    1,
                    1,
                    1,  # waist
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,  # arms
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,
                    1,  # arms
                ],
                dtype=float,
            )
        else:
            self.motor_kp = np.asarray(self.motor_kp, dtype=float)
            self.motor_kd = np.asarray(self.motor_kd, dtype=float)

    def _values_in_motor_order(self, values, default_values):
        values = np.asarray(values, dtype=float).reshape(-1)
        if values.shape[0] == self.num_real_motor:
            return values
        if values.shape[0] > self.real_motor_max_idx:
            return np.asarray(
                [values[int(joint)] for joint in self.robot_cfg.RealMotors], dtype=float
            )
        return np.asarray(default_values, dtype=float).reshape(self.num_real_motor)

    def _create_damping_cmd(self, cmd):
        for motor_cmd in cmd.motor_cmd:
            motor_cmd.q = 0
            motor_cmd.dq = 0
            motor_cmd.kp = 0
            motor_cmd.kd = 8
            motor_cmd.tau = 0

    def _send_cmd(self, cmd):
        if (
            self.enable_large_torque_protection
            and abs(self.tau_record).max() > self.large_torque_threshold
        ):
            high_tau_idx = np.arange(self.num_real_motor)[
                abs(self.tau_record) > self.large_torque_threshold
            ]
            real_motors = list(self.robot_cfg.RealMotors)
            high_tau_names = [real_motors[int(idx)].name for idx in high_tau_idx]
            print("large tau: ", high_tau_idx, high_tau_names)
            self._create_damping_cmd(self.cmd_msg)
            cmd = self.cmd_msg
        cmd.crc = self.crc.Crc(cmd)
        self._cmd_pub.Write(cmd)

    def set_motor_cmd(
        self,
        cmd_q: np.ndarray,
        cmd_dq: np.ndarray,
        cmd_tau: np.ndarray,
        cmd_kp: np.ndarray,
        cmd_kd: np.ndarray,
        weight: float,
    ):

        cmd_q = self._values_in_motor_order(cmd_q, np.zeros(self.num_real_motor))
        cmd_dq = self._values_in_motor_order(cmd_dq, np.zeros(self.num_real_motor))
        cmd_tau = self._values_in_motor_order(cmd_tau, np.zeros(self.num_real_motor))
        cmd_kp = self._values_in_motor_order(cmd_kp, self.motor_kp)
        cmd_kd = self._values_in_motor_order(cmd_kd, self.motor_kd)

        if weight is not None:
            self.weight_notUsedJoint = max(min(weight, 1.0), 0.0)
        self.cmd_msg.motor_cmd[self.kNotUsedJoint].q = self.weight_notUsedJoint**2
        current_q = self.get_real_motor_q()
        current_dq = self.get_real_motor_dq()
        for idx, joint in enumerate(self.robot_cfg.RealMotors):
            joint_pos_min, joint_pos_max = self.robot_cfg.RealMotorPosLimit.get(
                joint, (-np.inf, np.inf)
            )
            joint_q = (1 - self.cmd_q_lambda) * self.cmd_msg.motor_cmd[
                joint
            ].q + self.cmd_q_lambda * cmd_q[idx]
            joint_q = np.clip(joint_q, joint_pos_min, joint_pos_max)

            self.cmd_msg.motor_cmd[joint].mode = 1
            self.cmd_msg.mode_machine = self.mode_machine_
            self.cmd_msg.motor_cmd[joint].q = joint_q
            self.cmd_msg.motor_cmd[joint].dq = cmd_dq[idx]
            self.cmd_msg.motor_cmd[joint].tau = cmd_tau[idx]
            self.cmd_msg.motor_cmd[joint].kp = cmd_kp[idx]
            self.cmd_msg.motor_cmd[joint].kd = cmd_kd[idx]
            self.tau_record[idx] = (joint_q - current_q[idx]) * cmd_kp[idx] - current_dq[
                idx
            ] * cmd_kd[idx]
            # print("Setting motor command for joint {}: q = {}, dq = {}, tau = {}, kp = {}, kd = {}, weight = {}".format(
            #     joint.name, self.cmd_msg.motor_cmd[joint].q, self.cmd_msg.motor_cmd[joint].dq, self.cmd_msg.motor_cmd[joint].tau, self.cmd_msg.motor_cmd[joint].kp, self.cmd_msg.motor_cmd[joint].kd, weight
            # ))

    def send_control(self, command: np.ndarray, **kwargs):
        """
        Command is consisted of
            - HIGH control level: the motor commands for the upperbody and velocity commands for the locomotion.
            - LOW control level: the motor commands for all joints.
        All control inputs are given as velocity. Thus, they need to be integrated to get the position commands.

        Args:
            control (np.ndarray): Control input for the robot. Always velocity (NOTE: for now).
            cmd_kp (np.ndarray): Proportional gain for the motor control.
            cmd_kd (np.ndarray): Derivative gain for the motor control.
            weight (float): Weight for the not used joint in sports mode.
        """
        # ---------------------------- Control parameters ---------------------------- #
        target_kp = kwargs.get("cmd_kp", self.motor_kp)
        target_kd = kwargs.get("cmd_kd", self.motor_kd)
        weight = kwargs.get("weight", None)
        action_info = kwargs.get("action_info", {})

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

        x = self._compose_cmd_state()
        x = self.dynamics_executor.step(
            command,
            state=x,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )

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

            if self.control_level == UnitreeControlLevel.HIGH:
                cmd_q = self._real_motor_cmd_from_dof_pos(self.dof_pos_cmd)
            elif self.control_level == UnitreeControlLevel.LOW:
                self.default_pos_lower_body = [
                    -0.1,
                    0.0,
                    0.0,
                    0.3,
                    -0.2,
                    0.0,
                    -0.1,
                    0.0,
                    0.0,
                    0.3,
                    -0.2,
                    0.0,
                ]
                self.default_dof_pos = np.array(
                    [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
                )
                cmd_q = np.concatenate((self.default_pos_lower_body, self.default_dof_pos[:17]))

            else:
                raise ValueError("Invalid control level.")
            self._maybe_print_real_joint_tracking_debug(action_info, cmd_q)
            self.set_motor_cmd(
                cmd_q=cmd_q,
                cmd_dq=np.zeros(self.num_real_motor),
                cmd_tau=np.zeros(self.num_real_motor),
                cmd_kp=target_kp,
                cmd_kd=target_kd,
                weight=weight,
            )
            self._send_cmd(self.cmd_msg)
            # ------------------------------- Loco control ------------------------------- #
            # if self.control_level == UnitreeControlLevel.HIGH:
            #     self.loco_client.Move(vx=vx,
            #                         vy=vy,
            #                         vyaw=vyaw,
            #                         continous_move=True)

    def _compose_cmd_state(self):
        """
        Compose the state for the robot agent.
        Since the real G1 agent is controlled with velocity, this is the positions.
        """
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
        """
        Motor position (q) for all motors, regardless of the control level and whether the motor is used or not.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].q
                for id in range(self.robot_cfg.NumTotalMotors)
            ]
        )

    def get_all_motor_dq(self):
        """
        Motor velocity (dq) for all motors, regardless of the control level and whether the motor is used or not.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].dq
                for id in range(self.robot_cfg.NumTotalMotors)
            ]
        )

    def get_all_motor_ddq(self):
        """
        Motor acceleration (ddq) for all motors, regardless of the control level and whether the motor is used or not.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].ddq
                for id in range(self.robot_cfg.NumTotalMotors)
            ]
        )

    def get_real_motor_q(self):
        """
        Motor position (q) for real motors only.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].q
                for id in range(self.real_motor_min_idx, self.real_motor_max_idx + 1)
            ]
        )

    def get_real_motor_dq(self):
        """
        Motor velocity (dq) for real motors only.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].dq
                for id in range(self.real_motor_min_idx, self.real_motor_max_idx + 1)
            ]
        )

    def get_real_motor_ddq(self):
        """
        Motor acceleration (ddq) for real motors only.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].ddq
                for id in range(self.real_motor_min_idx, self.real_motor_max_idx + 1)
            ]
        )

    def get_real_motor_tau_est(self):
        """
        Estimated motor torque for real motors only.
        """
        return np.array(
            [
                self._low_state_buffer.GetData().motor_state[id].tau_est
                for id in range(self.real_motor_min_idx, self.real_motor_max_idx + 1)
            ]
        )

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

        ret["dof_pos_fbk"] = dof_pos_fbk
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
        ret["state"] = self._compose_cmd_state()  # NOTE: Not sure if this is all we need.

        # ------------------------------ Debug Frames -------------------------------- #
        ret["obstacle_debug_frame"] = np.empty((0, 4, 4))
        ret["obstacle_debug_geom"] = []
        ret["obstacle_debug_velocity"] = np.empty((0, 6))
        ret["robot_goal_left_offset"] = self.left_goal_debug_frame.copy()
        ret["robot_goal_right_offset"] = self.right_goal_debug_frame.copy()
        ret["robot_goal_base_offset"] = self.base_goal_debug_frame.copy()
        ret["left_gripper_debug_state"] = self.left_gripper_debug_state
        ret["right_gripper_debug_state"] = self.right_gripper_debug_state

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
    from spark_robot import (
        UnitreeG1FixedBaseDynamic1Config,
        UnitreeG1WholeBodyWithHandDynamic1Config,
    )

    # g1_real_agent = UnitreeG1RealSportModeAgent(robot_cfg = UnitreeG1FixedBaseDynamic1Config(), unitree_model="g1", level="low", send_cmd=True)
    g1_real_agent = UnitreeG1RealSportModeAgent(
        robot_cfg=UnitreeG1WholeBodyWithHandDynamic1Config(),
        unitree_model="g1",
        level="low",
        send_cmd=True,
    )

    # fsm_id, data = g1_real_agent.loco_client.GetFsmId(0)
    # print("FSM ID: ", data)

    # code = g1_real_agent.loco_client.HighStand()
    # print(code)

    # g1_real_agent.reset_upper_body()
    fbk = g1_real_agent.get_feedback()

    while g1_real_agent.update_mode_machine_ == False:
        time.sleep(1)

    # while True:
    #     g1_real_agent.send_control(np.zeros(17), control_type="pos")
    #     g1_real_agent.get_feedback()
    while True:
        # print(g1_real_agent.get_imu_rpy())
        # print(g1_real_agent.get_imu_quaternion())
        # print(g1_real_agent.get_imu_gyroscope())
        # print(g1_real_agent.get_imu_accelerometer())
        # print(g1_real_agent.get_real_motor_q())
        # print(g1_real_agent.get_real_motor_dq())
        # print(g1_real_agent.get_real_motor_ddq())
        cmd = np.zeros(len(g1_real_agent.robot_cfg.Control))
        g1_real_agent.send_control(cmd, control_type="pos")
        time.sleep(0.1)
