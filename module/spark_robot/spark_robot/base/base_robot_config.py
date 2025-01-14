from abc import ABC, abstractmethod
from enum import IntEnum, Enum
import numpy as np
import inspect

class ControlType(IntEnum):

    POS = 0
    VEL = 1
    ACC = 2
    JERK = 3

class RobotConfig(ABC):
    """
    Abstract base class for robot configuration.
    Subclasses must implement hardware, DoF, frame, and safety configurations.
    """
    
    def __init__(self) -> None:
        
        super().__init__()
        
        """ Initializes all member classes recursively. Ignores all names starting with '__' (built-in methods)."""
        self.init_member_classes(self)
    
    # taken from git@github.com:leggedrobotics/legged_gym.git
    @staticmethod
    def init_member_classes(obj):
        
        # Iterate over all attributes names
        for key in dir(obj):
            # Disregard builtin attributes
            if key == "__class__":
                continue
            
            # Get the corresponding attribute object
            var = getattr(obj, key)
            
            # Check if the attribute is a class but not an Enum
            if inspect.isclass(var) and not issubclass(var, Enum):
                
                # Instantiate the class
                i_var = var()
                # Set the attribute to the instance instead of the type
                setattr(obj, key, i_var)
                # Recursively init members of the attribute
                RobotConfig.init_member_classes(i_var)
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    @property
    @abstractmethod
    def RealMotors(self) -> IntEnum:
        """
        Enumeration for robot motor indices. Consistent with hardware interface.
        """
        pass

    @property
    @abstractmethod
    def RealMotorPosLimit(self) -> dict:
        """
        Mapping from RealMotors to joint limits.
        Each limit is a tuple (min, max).
        """
        pass

    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    @property
    @abstractmethod
    def DoFs(self) -> IntEnum:
        """
        Enumeration for robot degrees of freedom indices. For control purpose.
        """
        pass
    
    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    @property
    @abstractmethod
    def Control(self) -> IntEnum:
        """
        Enumeration for robot control indices.
        """
        pass
    
    @property
    @abstractmethod
    def ControlLimit(self) -> dict:
        """
        Mapping from Control to control limits.
        """
        pass

    @property
    @abstractmethod
    def NormalControl(self) -> list:
        """
        List of control indices representing normal actuators.
        """
        pass

    @property
    @abstractmethod
    def WeakControl(self) -> list:
        """
        List of control indices representing weak actuators.
        """
        pass

    @property
    @abstractmethod
    def DelicateControl(self) -> list:
        """
        List of control indices representing delicate actuators.
        """
        pass

    '''
        x_dot = f(x) + g(x) * control
        
        x: state
    '''
    
    @property
    @abstractmethod
    def num_state(self):
        pass

    @abstractmethod
    def compose_state_from_dof(self):
        pass

    @abstractmethod
    def decompose_state_to_dof(self):
        pass

    @abstractmethod
    def dynamics_f(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, 1]
        '''
        pass

    @abstractmethod
    def dynamics_g(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, num_control]
        '''
        pass

    def dynamics_xdot(self, state, control):
        '''
            state: [num_state,]
            control: [num_control,]
            return: [num_state,]
        '''

        return (self.dynamics_f(state.reshape(-1, 1)) + self.dynamics_g(state.reshape(-1, 1)) @ control.reshape(-1, 1)).reshape(-1)

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #

    @property
    @abstractmethod
    def MujocoDoFs(self) -> IntEnum:
        """
        Enumeration for Mujoco dofs. For indexing qpos.
        """
        pass
    
    @property
    @abstractmethod
    def MujocoMotors(self) -> IntEnum:
        """
        Enumeration for Mujoco motors. For indexing ctrl.data.
        """
        pass

    # ---------------------------------------------------------------------------- #
    #                                   Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    @property
    @abstractmethod
    def MujocoDoF_to_DoF(self) -> dict:
        """
        Mapping from Mujoco dof to dof.
        """
        pass
    
    @property
    @abstractmethod
    def DoF_to_MujocoDoF(self) -> dict:
        """
        Mapping from dof to Mujoco dof.
        """
        pass
    
    @property
    @abstractmethod
    def MujocoMotor_to_Control(self) -> dict:
        """
        Mapping from Mujoco motor to control.
        """
        pass

    @property
    @abstractmethod
    def RealMotor_to_Control(self) -> dict:
        """
        Mapping from real motor to control.
        """
        pass

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #

    @property
    @abstractmethod
    def Frames(self) -> IntEnum:
        """
        Enumeration for frames of interest.
        """
        pass

    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #

    @property
    @abstractmethod
    def CollisionVol(self) -> dict:
        """
        Mapping from Frames to collision volume spec.
        """
        pass

    @property
    @abstractmethod
    def AdjacentCollisionVolPairs(self) -> list:
        """
        List of tuples or lists representing pairs of adjacent collision volumes to ignore.
        """
        pass

    @property
    @abstractmethod
    def SelfCollisionVolIgnored(self) -> list:
        """
        List of Frames values representing ignored collision volumes.
        """
        pass

    # ---------------------------------------------------------------------------- #
    #                                    Helper                                    #
    # ---------------------------------------------------------------------------- #

    # Methods for checking DoF types
    def is_normal_dof(self, dof: int) -> bool:
        return dof in self.NormalControl
    
    def is_weak_dof(self, dof: int) -> bool:
        return dof in self.WeakControl

    def is_delicate_dof(self, dof: int) -> bool:
        return dof in self.DelicateControl
