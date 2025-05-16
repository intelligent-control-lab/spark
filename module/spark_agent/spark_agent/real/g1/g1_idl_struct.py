"""
Data structures used to contain Unitree IDL (Interface Definition Language).
These structures are used to communicate with the Unitree SDK.
These data structures are based on buffer data structures, using threading.
"""
import threading


class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data
            
class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None
        self.ddq = None
        self.tau_est = None

class MotorCmd:
    def __init__(self):
        self.q = None
        self.dq = None
        self.kp = None
        self.kd = None
        self.tau = None

# ---------------------------------------------------------------------------- #
#                                   Low State                                  #
# ---------------------------------------------------------------------------- #
class LowStateDataStruct:
    def __init__(self):
        # TODO： change this to be more generic in the future for H1
        ### IMU State ###
        self.imu_rpy = None
        self.imu_quaternion = None
        self.imu_gyroscope = None
        self.imu_accelerometer = None
        ### Motor State ###
        self.motor_state = [MotorState() for _ in range(35)]

class G1_29_LowState(LowStateDataStruct):
    def __init__(self):
        super().__init__()
        ### IMU State ###
        self.imu_rpy = None
        self.imu_quaternion = None
        self.imu_gyroscope = None
        self.imu_accelerometer = None
        ### Motor State ###
        self.motor_state = [MotorState() for _ in range(35)]

class H1_LowState(LowStateDataStruct):
    def __init__(self):
        super().__init__()
        raise NotImplementedError("H1_LowState not implemented.")

# ---------------------------------------------------------------------------- #
#                            High State (Sport Mode)                           #
# ---------------------------------------------------------------------------- #
class HighStateDataStruct:
    def __init__(self):
        # TODO： change this to be more generic in the future for H1
        self.position = None
        self.velocity = None
        self.yaw_speed = None

class G1_29_HighState(HighStateDataStruct):
    def __init__(self):
        super().__init__()
        self.position = None
        self.velocity = None
        self.yaw_speed = None

class H1_HighState(HighStateDataStruct):
    def __init__(self):
        super().__init__()
        raise NotImplementedError("H1_HighState not implemented.")

# ---------------------------------------------------------------------------- #
#                                 Command State                                #
# ---------------------------------------------------------------------------- #

class CmdDataStruct:
    def __init__(self):
        # TODO： change this to be more generic in the future for H1
        ### Arm SDK Command ###
        self.motor_cmd = [MotorCmd() for _ in range(17)]
        ### NotUsedJoint ###
        self.arm_sdk_notUsedJoint_q = 0.0

class G1_29_ArmSDK_State(CmdDataStruct):
    def __init__(self):
        super().__init__()
        ### Arm SDK Command ###
        self.motor_cmd = [MotorCmd() for _ in range(17)]
        ### NotUsedJoint ###
        self.arm_sdk_notUsedJoint_q = 0.0

class H1_ArmSDK_State(CmdDataStruct):
    def __init__(self):
        super().__init__()
        raise NotImplementedError("H1_ArmSDK_State not implemented.")
    
class G1_29_LowCmd_State(CmdDataStruct):
    def __init__(self):
        super().__init__()
        ### Low Command ###
        self.motor_cmd = [MotorCmd() for _ in range(35)]

class H1_LowCmd_State(CmdDataStruct):
    def __init__(self):
        super().__init__()
        raise NotImplementedError("H1_LowCmd_State not implemented.")