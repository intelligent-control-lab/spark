from abc import ABC, abstractmethod
from enum import IntEnum, Enum
from dataclasses import replace
import numpy as np
import inspect

from .backend_capability import BackendCapability, IsaacArticulationSpec, SimulatorDynamicsSpec


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

    # Backend-to-agent compatibility belongs to the robot contract. Concrete
    # configs provide string names to avoid making spark_robot import
    # spark_agent. Several robot configs may intentionally name the same agent.
    agent_class_names = {}
    backend_capability_overrides = {}
    simulator_dynamics = SimulatorDynamicsSpec()
    isaac_articulation: IsaacArticulationSpec | None = None
    model_only = False
    # Optional deterministic goal-reaching gains used by the cross-backend
    # release conformance runner. They describe validation of the configured
    # plant; normal policies continue to own their controller tuning.
    conformance_position_kp: float | None = None
    conformance_velocity_kd: float | None = None
    # Optional ordered glob-to-RGBA mapping used by simulator adapters when a
    # source asset does not carry a useful visual palette. Patterns match URDF
    # link names and MuJoCo geometry names with a trailing ``_visual`` removed.
    visual_link_colors = ()

    @classmethod
    def agent_class_name(cls, backend="mujoco"):
        try:
            return cls.agent_class_names[backend]
        except KeyError as exc:
            raise ValueError(
                f"{cls.__name__} does not declare an agent for backend {backend!r}"
            ) from exc

    @classmethod
    def backend_capabilities(cls) -> dict[str, BackendCapability]:
        """Return structured records for every declared backend.

        Existing ``agent_class_names`` mappings remain the public compatibility
        contract. New code can use these records without forcing every robot
        configuration to duplicate common backend metadata.
        """
        capabilities = {}
        for backend, agent_class_name in cls.agent_class_names.items():
            defaults = {
                "mujoco": BackendCapability(
                    agent_class_name,
                    rendering=True,
                    validation="validated",
                ),
                "isaac": BackendCapability(
                    agent_class_name,
                    batched=False,
                    rendering=True,
                    tensor_io=False,
                    validation="experimental",
                ),
                "real": BackendCapability(
                    agent_class_name,
                    real_hardware=True,
                    validation="declared",
                ),
                "dynamics": BackendCapability(
                    agent_class_name,
                    rendering=False,
                    validation="validated",
                ),
            }
            capability = cls.backend_capability_overrides.get(
                backend,
                defaults.get(backend, BackendCapability(agent_class_name)),
            )
            # ``agent_class_names`` is the authoritative routing contract.
            # Capability overrides describe validation/rendering/tensor
            # properties and may be shared across several embodiment-specific
            # adapters, so do not let a shared historical class name bypass
            # the robot-specific agent selected above.
            capabilities[backend] = replace(capability, agent_class_name=agent_class_name)
        return capabilities

    def __init__(self) -> None:

        super().__init__()

        """ Initializes all member classes recursively. Ignores all names starting with '__' (built-in methods)."""
        self.init_member_classes(self)
        self._post_init()

    def _post_init(self):
        """
        Optional hook for subclasses.
        Called after nested config classes are instantiated.
        """
        pass

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
    def NumTotalMotors(self) -> int:
        """
        Total number of motors in the robot.
        """
        pass

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

    @property
    @abstractmethod
    def NormalMotor(self) -> list:
        """
        List of motor indices representing normal actuators.
        """
        pass

    @property
    @abstractmethod
    def WeakMotor(self) -> list:
        """
        List of motor indices representing weak actuators.
        """
        pass

    @property
    @abstractmethod
    def DelicateMotor(self) -> list:
        """
        List of motor indices representing delicate actuators.
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

    @property
    @abstractmethod
    def DefaultDoFVal(self) -> dict:
        """
        Mapping from DoFs to default joint values.
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

    """
        x_dot = f(x) + g(x) * control
        
        x: state
    """

    @property
    @abstractmethod
    def num_state(self):
        pass

    @property
    def num_dof(self):
        return len(self.DoFs)

    @property
    def dynamics_order(self):
        if self.num_state == self.num_dof:
            return 1
        if self.num_state == 2 * self.num_dof:
            return 2
        raise ValueError(
            f"Cannot infer dynamics order for {type(self).__name__}: "
            f"num_state={self.num_state}, num_dof={self.num_dof}."
        )

    @abstractmethod
    def compose_state_from_dof(self):
        pass

    @abstractmethod
    def decompose_state_to_dof_pos(self):
        pass

    @abstractmethod
    def decompose_state_to_dof_vel(self):
        pass

    @abstractmethod
    def dynamics_f(self, state):
        """
        state: [num_state, 1]
        return: [num_state, 1]
        """
        pass

    @abstractmethod
    def dynamics_g(self, state):
        """
        state: [num_state, 1]
        return: [num_state, num_control]
        """
        pass

    def dynamics_xdot(self, state, control):
        """
        state: [num_state,]
        control: [num_control,]
        return: [num_state,]
        """
        return (
            self.dynamics_f(state.reshape(-1, 1))
            + self.dynamics_g(state.reshape(-1, 1)) @ control.reshape(-1, 1)
        ).reshape(-1)

    def create_dynamics_model(self, state_dof_names=None, control_names=None):
        """Create the dynamics view used by policies and numerical examples.

        Dynamics selection is determined by the concrete robot configuration;
        callers may only select a reduced set of its state and control axes.
        """
        from .dynamics_model import RobotDynamicsModel

        return RobotDynamicsModel(
            self,
            state_dof_names=state_dof_names,
            control_names=control_names,
        )

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
        List of Frames values representing ignored self collision volumes.
        """
        pass

    @property
    @abstractmethod
    def EnvCollisionVolIgnored(self) -> list:
        """
        List of Frames values representing ignored environment collision volumes.
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
