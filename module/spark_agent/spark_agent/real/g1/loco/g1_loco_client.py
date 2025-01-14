"""
Developed by ICL (Intelligent Control Lab) at CMU Robotics Institute.
Developer: Kai Yun
"""
import json

from unitree_sdk2py.rpc.client import Client
from g1.loco.g1_loco_api import *


class LocoClient(Client):
    def __init__(self, enabaleLease: bool = False):
        super().__init__(LOCO_SERVICE_NAME, enabaleLease)

        self._continuous_move = False
        
    def Init(self):
        # Set API version
        self._SetApiVerson(LOCO_API_VERSION)

        # Regist API
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_PHASE, 0)

        self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)

    def _LocoError(self, code: int):
        if code == 0:
            return

        error_msg = "[LocoClient Error] "
        if code == 7301:
            error_msg += "LocoState not available."
        elif code == 7302:
            error_msg += "Invalid FSM ID."
        elif code == 3001:
            error_msg += "Unknown error."
        elif code == 3102:
            error_msg += "Request sending error."
        elif code == 3103:
            error_msg += "API not registered."
        elif code == 3104:
            error_msg += "Request timeout."
        elif code == 3105:
            error_msg += "Request and response data do not match."
        elif code == 3106:
            error_msg += "Invalid response data."
        elif code == 3107:
            error_msg += "Invalid lease."
        elif code == 3201:
            error_msg += "Response sending error."
        elif code == 3202:
            error_msg += "Internal server error."
        elif code == 3203:
            error_msg += "API not implemented on the server."
        elif code == 3204:
            error_msg += "API parameter error."
        elif code == 3205:
            error_msg += "Request rejection."
        elif code == 3206:
            error_msg += "Invalid lease."
        elif code == 3207:
            error_msg += "Lease already exists."

        print(error_msg)

    def GetFsmId(self, fsm_id: int):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None
    
    def GetFsmMode(self, fsm_mode: int):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None
        
    def GetBalanceMode(self, balance_mode: int):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None

    def GetSwingHeight(self, swing_height: float):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None
        
    def GetStandHeight(self, stand_height: float):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None
        
    def GetPhase(self, phase: list):
        p = {}
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_GET_PHASE, parameter)
        self._LocoError(code)

        if code == 0:
            d = json.loads(data)
            return code, d["data"]
        else:
            return code, None
        
    def SetFsmId(self, fsm_id: int):
        p = {}
        p["data"] = fsm_id
        
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, parameter)
        self._LocoError(code)

        return code, data
    
    def SetBalanceMode(self, balance_mode: int):
        p = {}
        p["data"] = balance_mode
        
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, parameter)
        self._LocoError(code)

        return code, data
    
    def SetSwingHeight(self, swing_height: float):
        p = {}
        p["data"] = swing_height
        
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, parameter)
        self._LocoError(code)

        return code, data
    
    def SetStandHeight(self, stand_height: float):
        p = {}
        p["data"] = stand_height
        
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, parameter)
        self._LocoError(code)

        return code, data
    
    def SetVelocity(self, vx: float, vy: float, omega: float, duration=1.0):
        p = {}
        p["velocity"] = [vx, vy, omega]
        p["duration"] = duration
        
        parameter = json.dumps(p)

        code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
        self._LocoError(code)

        return code, data
    
    def Damp(self):
        return self.SetFsmId(1)

    def Start(self):
        return self.SetFsmId(200)
    
    def Squat(self):
        return self.SetFsmId(2)

    def Sit(self):
        return self.SetFsmId(3)
    
    def StandUp(self):
        return self.SetFsmId(4)

    def ZeroTorque(self):
        return self.SetFsmId(0)
    
    def StopMove(self):
        return self.SetVelocity(0.0, 0.0, 0.0)

    def HighStand(self):
        max_uint32 = (2**32) - 1
        return self.SetStandHeight(max_uint32)
    
    def LowStand(self):
        return self.SetStandHeight(0.0)
    
    def Move(self, vx: float, vy: float, omega: float, continuous_move=None) -> int:
        # If continuous_move is not provided, use self._continuous_move as the default
        if continuous_move is None:
            continuous_move = self._continuous_move

        # Call the SetVelocity method with the calculated duration
        return self.SetVelocity(vx, vy, omega, 864000.0 if continuous_move else 1.0)
    
    def BalanceStand(self):
        return self.SetBalanceMode(0)
    
    def ContinuousGait(self, enable: bool):
        return self.SetBalanceMode(1 if enable else 0)
    
    def SwitchMoveMode(self, enable: bool):
        self._continuous_move = enable
        return 0


if __name__ == "__main__":
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize()

    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()

    code, data = client.GetFsmId(0)
    print("GetFsmId")
    print(code, data)
    code, data = client.GetFsmMode(0)
    print("GetFsmMode")
    print(code, data)
    code, data = client.GetBalanceMode(0)
    print("GetBalanceMode")
    print(code, data)
    code, data = client.GetSwingHeight(0.0)
    print("GetSwingHeight")
    print(code, data)
    code, data = client.GetStandHeight(0.0)
    print("GetStandHeight")
    print(code, data)
    code, data = client.GetPhase([])
    print("GetPhase")
    print(code, data)
    code, data = client.SetFsmId(1)
    print("SetFsmId to 1")
    print(code, data)