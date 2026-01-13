#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
import numpy as np
from typing import Dict, Any, List, Optional

try:
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Bool
except ImportError:
    JointState = TwistStamped = Bool = None

from spark_agent.base.base_agent import BaseAgent
from spark_robot import RobotConfig


class R1LiteRealAgent(BaseAgent):
    """
    R1 Lite real-robot agent (no kp/kd; publish-only).
    - Directly publishes JointState / TwistStamped / Bool to R1 Lite topics.
    - (Optional) subscribes to feedback topics for get_feedback().
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs):
        try:
            import rospy
        except ImportError:
            raise RuntimeError("R1LiteRealAgent requires rospy but it is not installed.")
        self.rospy = rospy
        super().__init__(robot_cfg)

        self.robot_cfg = robot_cfg
        self.dt = 0.1
        self.control_decimation =1
        # ---------- ROS init ----------
        # rospy.init_node("r1lite_agent", anonymous=True)

        # ---------- Publishers ----------
        self.left_arm_pub   = rospy.Publisher("/motion_target/target_joint_state_arm_left",  JointState, queue_size=10)
        self.right_arm_pub  = rospy.Publisher("/motion_target/target_joint_state_arm_right", JointState, queue_size=10)
        self.torso_pub_real = rospy.Publisher("/motion_target/target_joint_state_torso",     JointState, queue_size=10)

        self.left_grip_pub  = rospy.Publisher("/motion_target/target_position_gripper_left",  JointState, queue_size=10)
        self.right_grip_pub = rospy.Publisher("/motion_target/target_position_gripper_right", JointState, queue_size=10)

        self.chassis_vel_pub = rospy.Publisher("/motion_target/target_speed_chassis", TwistStamped, queue_size=10)
        self.acc_limit_pub   = rospy.Publisher("/motion_target/chassis_acc_limit",    TwistStamped, queue_size=10)
        self.brake_pub       = rospy.Publisher("/motion_target/brake_mode",           Bool,         queue_size=10)

        # ---------- (Optional) Feedback subscribers ----------
        self._fbk = {
            "arm_left":  None,  # type: Optional[JointState]
            "arm_right": None,
            "torso":     None,
        }
        self._sub_left  = rospy.Subscriber("/hdas/feedback_arm_left",   JointState, self._on_left_arm)
        self._sub_right = rospy.Subscriber("/hdas/feedback_arm_right",  JointState, self._on_right_arm)
        self._sub_torso = rospy.Subscriber("/hdas/feedback_torso",      JointState, self._on_torso)

        # ---------- Command caches (optional) ----------
        self._last_left_cmd  : Optional[List[float]] = None
        self._last_right_cmd : Optional[List[float]] = None
        self._last_torso_cmd : Optional[List[float]] = None

    # ------------------------------------------------------------------
    # Feedback callbacks
    # ------------------------------------------------------------------
    def _on_left_arm(self, msg: JointState):  self._fbk["arm_left"]  = msg
    def _on_right_arm(self, msg: JointState): self._fbk["arm_right"] = msg
    def _on_torso(self, msg: JointState):     self._fbk["torso"]     = msg

    # ------------------------------------------------------------------
    # Publish helpers (direct control)
    # ------------------------------------------------------------------
    def send_arm_positions(self,
                           left:  List[float],
                           right: List[float],
                           stamp_now: bool = True):
        lmsg = JointState(); rmsg = JointState()
        lmsg.position = list(left)
        rmsg.position = list(right)
        if stamp_now:
            lmsg.header.stamp = rospy.Time.now()
            rmsg.header.stamp = rospy.Time.now()
        self.left_arm_pub.publish(lmsg)
        self.right_arm_pub.publish(rmsg)
        self._last_left_cmd  = list(left)
        self._last_right_cmd = list(right)

    def send_torso_positions(self, torso: List[float], stamp_now: bool = True):
        tmsg = JointState()
        tmsg.position = list(torso)
        if stamp_now:
            tmsg.header.stamp = rospy.Time.now()
        self.torso_pub_real.publish(tmsg)
        self._last_torso_cmd = list(torso)

    def send_gripper_positions(self, left, right, stamp_now: bool = True):
        now = rospy.Time.now() if stamp_now else None

        if left is not None:
            lmsg = JointState()
            lmsg.position = [float(left)] if not isinstance(left, (list, tuple)) else list(left)
            if now: lmsg.header.stamp = now
            self.left_grip_pub.publish(lmsg)

        if right is not None:
            rmsg = JointState()
            rmsg.position = [float(right)] if not isinstance(right, (list, tuple)) else list(right)
            if now: rmsg.header.stamp = now
            self.right_grip_pub.publish(rmsg)


    def send_chassis_twist(self, vx: float, vy: float, wz: float, stamp_now: bool = True):
        t = TwistStamped()
        if stamp_now: t.header.stamp = rospy.Time.now()
        t.twist.linear.x = vx
        t.twist.linear.y = vy
        t.twist.angular.z = wz
        self.chassis_vel_pub.publish(t)

    def set_acc_limit(self, ax: float, ay: float, awz: float, stamp_now: bool = True):
        acc = TwistStamped()
        if stamp_now: acc.header.stamp = rospy.Time.now()
        acc.twist.linear.x = ax
        acc.twist.linear.y = ay
        acc.twist.angular.z = awz
        self.acc_limit_pub.publish(acc)

    def set_brake_mode(self, enabled: bool):
        b = Bool(data=bool(enabled))
        self.brake_pub.publish(b)

    def compose_cmd_state(self):
        '''
        Compose the state for the robot agent.
        Since the real G1 agent is controlled with velocity, this is the positions.
        '''
        x = self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)
        
        return x

    def send_control(self, command: np.ndarray, *, 
                    left_gripper_goal=None,
                    right_gripper_goal=None,
                    **kwargs):
        """
        Send a complete robot control command.

        DoF index layout:
        0-2   -> Torso joints (3 DoFs)
        3-8   -> Right arm joints (6 DoFs)
        9-14  -> Left arm joints (6 DoFs)


        Args:
            command: np.ndarray of shape (15,) for joint position targets (radians).
            left_gripper_goal: 0 or 1, open or close left gripper.
            right_gripper_goal: 0 or 1, open or close right gripper.
        """
        # -------------------- Original state integration logic --------------------
        x = self.compose_cmd_state()
        x_dot = self.robot_cfg.dynamics_xdot(x, command)

        # Integration
        x += x_dot * self.dt * self.control_decimation

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        # Split joint commands
        torso_cmd = self.dof_pos_cmd[0:3]                  # First 3 torso joints
        torso_cmd_4 = np.append(torso_cmd, 0.0)   # Add a 4th value = 0
        right_arm_cmd = self.dof_pos_cmd[3:9]              # Right arm 6 joints
        left_arm_cmd  = self.dof_pos_cmd[9:15]             # Left arm 6 joints
        # Send torso with 4 values
        self.send_torso_positions(torso_cmd_4.tolist())

        # Send arms: note the order is (left, right)
        self.send_arm_positions(left_arm_cmd.tolist(), right_arm_cmd.tolist())

        # -------------------- Gripper control --------------------
        action_info = kwargs.get("action_info", dict())
        left_gripper_goal  = action_info.get("left_gripper_goal",  False)
        right_gripper_goal = action_info.get("right_gripper_goal", False)

        if left_gripper_goal is not None:
            self.send_gripper_positions(0 if left_gripper_goal else 100, None)

        if right_gripper_goal is not None:
            self.send_gripper_positions(None, 0 if right_gripper_goal else 100)

    # High-level convenience routines
    # ------------------------------------------------------------------
    def reset(self):
        self.set_brake_mode(True)
        time.sleep(0.05)
        self.send_arm_positions([0,0,0,0,0,0], [0,0,0,0,0,0])
        self.send_torso_positions([0,0,0,0])
        self.send_gripper_positions([0], [0])

    def dance_demo(self):
        self.set_brake_mode(True)
        self.set_acc_limit(0.5, 0.79, 1.4)
        seq = [
            ([0,1.15,-1.19,0,0,0], [0,1.15,-1.19,0,0,0], 2.0),
            ([0.8,1.15,-1.19,0,0,0], [0.8,1.15,-1.19,0,0,0], 2.0),
            ([-0.8,1.15,-1.19,0,0,0], [-0.8,1.15,-1.19,0,0,0], 2.0),
            ([0,1.15,-1.19,0,0,0], [0,1.15,-1.19,0,0,0], 2.0),
            ([-1.5703,0.9,-1.18,0.59,0,0], [1.5703,0.9,-1.18,0.59,0,0], 3.0),
            ([0,0,0,0,0,0], [0,0,0,0,0,0], 3.0),
        ]
        self.send_torso_positions([0,0,0,0])
        for l, r, wait in seq:
            self.send_arm_positions(l, r)
            time.sleep(wait)
    def get_all_motor_q(self) -> np.ndarray:
        """
        Get all real robot motor positions (qpos).
        Returns:
            np.ndarray shape = (NumTotalMotors,)
            Order strictly follows self.robot_cfg.RealMotors
        """
        num_total = self.robot_cfg.NumTotalMotors
        qpos = np.zeros(num_total, dtype=float)

        # ----- Torso 0-2 -----
        if self._fbk.get("torso"):
            torso_pos = list(self._fbk["torso"].position)
            for i in range(min(3, len(torso_pos))):
                qpos[i] = torso_pos[i]

        # ----- Right Arm 3-8 -----
        if self._fbk.get("arm_right"):
            right_pos = list(self._fbk["arm_right"].position)
            for i in range(min(6, len(right_pos))):
                qpos[3 + i] = right_pos[i]

        # ----- Left Arm 9-14 -----
        if self._fbk.get("arm_left"):
            left_pos = list(self._fbk["arm_left"].position)
            for i in range(min(6, len(left_pos))):
                qpos[9 + i] = left_pos[i]
        return qpos

    def get_all_motor_dq(self) -> np.ndarray:
        """
        Get all real robot motor velocities (qvel).
        Returns:
            np.ndarray shape = (NumTotalMotors,)
            Order strictly follows self.robot_cfg.RealMotors
        """
        num_total = self.robot_cfg.NumTotalMotors
        qvel = np.zeros(num_total, dtype=float)

        # ----- Torso 0-2 -----
        if self._fbk.get("torso"):
            torso_vel = list(self._fbk["torso"].velocity)
            for i in range(min(3, len(torso_vel))):
                qvel[i] = torso_vel[i]

        # ----- Right Arm 3-8 -----
        if self._fbk.get("arm_right"):
            right_vel = list(self._fbk["arm_right"].velocity)
            for i in range(min(6, len(right_vel))):
                qvel[3 + i] = right_vel[i]

        # ----- Left Arm 9-14 -----
        if self._fbk.get("arm_left"):
            left_vel = list(self._fbk["arm_left"].velocity)
            for i in range(min(6, len(left_vel))):
                qvel[9 + i] = left_vel[i]
        return qvel

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------
    def get_feedback(self) -> Dict[str, Any]:
        ret = {}
        robot_base_frame = np.zeros((4, 4))
        robot_base_frame= np.eye(4)


        ret["robot_base_frame"] = robot_base_frame
        real_robot_qpos = self.get_all_motor_q()
        real_robot_qpvel = self.get_all_motor_dq()
        dof_pos_fbk = np.zeros(self.num_dof)
        dof_vel_fbk = np.zeros(self.num_dof)
        ret["dof_pos_fbk"] =dof_pos_fbk
        ret["dof_vel_fbk"] = dof_vel_fbk
        for dof in self.robot_cfg.DoFs:
            real_dof = self.robot_cfg.DoF_to_RealDoF[dof]
            dof_pos_fbk[dof] = real_robot_qpos[real_dof]
            dof_vel_fbk[dof] = real_robot_qpvel[real_dof]
        if self.dof_pos_cmd is None:
            self.dof_pos_cmd = dof_pos_fbk
        if self.dof_vel_cmd is None:
            self.dof_vel_cmd = np.zeros(self.num_dof)
        if self.dof_acc_cmd is None:
            self.dof_acc_cmd = np.zeros(self.num_dof)
        ret["dof_pos_cmd"] = self.dof_pos_cmd
        ret["dof_vel_cmd"] = self.dof_vel_cmd
        ret["dof_acc_cmd"] = self.dof_acc_cmd
        ret["state"] = self.compose_cmd_state() # NOTE: Not sure if this is all we need.

        return ret

