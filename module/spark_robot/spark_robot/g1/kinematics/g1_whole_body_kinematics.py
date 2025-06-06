import casadi          
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin               
import os

from spark_robot.g1.kinematics.g1_fixed_base_kinematics import G1FixedBaseKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR

class G1WholeBodyKinematics(G1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
            
    def _init_whole_body_kinematics(self):
        print("Initializing G1WholeBodyKinematics")
        self.kinematics_model_path = 'g1/g1_29dof_whole_body_fixed.xml'
        self.robot = pin.RobotWrapper.BuildFromMJCF(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path),
            pin.JointModelFreeFlyer()
        )
        
        self.model = self.robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)
        
        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame] = pin_frame_id 
                    print(f"Frame {pin_frame.name} has ID {self.pin_frame_dict[frame]}")

        for j in range(self.model.nframes):
            pin_frame = self.model.frames[j]
            print(f"Frame {pin_frame.name} has ID {j}")
            
        # Compute total mass by summing the masses of all bodies
        total_mass = sum(inertia.mass for inertia in self.model.inertias)

        print(f"Total robot mass: {total_mass} kg")
    
    def pre_computation(self, dof_pos, dof_vel = None):
        pin_dof_pos = dof_pos.copy()
        quat = pin_dof_pos[3:7].copy()
        pin_quat = np.array([quat[1], quat[2], quat[3], quat[0]])
        pin_dof_pos[3:7] = pin_quat
        pin.forwardKinematics(self.model, self.data, pin_dof_pos)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        pin.computeCentroidalMap(self.model, self.data, pin_dof_pos)
        if dof_vel is not None:
            pin.computeJointJacobiansTimeVariation(self.model, self.data, pin_dof_pos, dof_vel)
        return
    
    def forward_kinematics(self, dof):   
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = self.model.getFrameId("robot")
        frames = np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames
        return frames
    
    def get_dynamics(self, dof_pos, dof_vel, contact_frames):
        pin_dof_pos = dof_pos.copy()
        quat = pin_dof_pos[3:7].copy()
        pin_quat = np.array([quat[1], quat[2], quat[3], quat[0]])
        pin_dof_pos[3:7] = pin_quat
        pin.forwardKinematics(self.model, self.data, pin_dof_pos)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, pin_dof_pos, dof_vel)
        M = pin.crba(self.model, self.data, pin_dof_pos)  # Compute mass matrix
        nle = pin.nonLinearEffects(self.model, self.data, pin_dof_pos, dof_vel)  # Compute Coriolis and gravity terms
        J_c = np.vstack([self.get_jacobian(frame_name)[:3,:] for frame_name in contact_frames])
        dJ_c = np.vstack([self.get_jacobian_dot(frame_name)[:3,:] for frame_name in contact_frames])
        frame_c = np.vstack([self.get_frame_transformation(frame_name)[:3,3] for frame_name in contact_frames])
        
        self.compute_centroidal_dynamics(pin_dof_pos, dof_vel, np.ones(self.robot.model.nv) * 0.)
        
        return M, nle, J_c, dJ_c, frame_c
    
    def compute_centroidal_dynamics(self, q, v, a):
        """ Compute centroidal dynamics. """
        pin.computeCentroidalMap(self.robot.model, self.robot.data, q)
        pin.computeCentroidalMomentumTimeVariation(self.robot.model, self.robot.data, q, v, a)
        
        Ag = self.robot.data.Ag.copy()  # Should now be filled
        dAg = (self.robot.data.dhg.vector[:, np.newaxis] - Ag @ a[:, np.newaxis]) @ np.linalg.pinv(v[:, np.newaxis])
        # print("dhg: ", self.robot.data.dhg.vector[:3])
        print("hg: ", np.round(self.robot.data.hg.vector[:3], 2))

        if (Ag @ a + dAg @ v - self.robot.data.dhg.vector).sum() >= 1e-6:
            print("Warning: Centroidal dynamics computation failed. Check the input values.")
            
        pin.computeCentroidalMomentum(self.robot.model, self.robot.data, q, v)
        pin.computeCentroidalMap(self.robot.model, self.robot.data, q)

        
        return Ag, dAg
        
    def get_jacobian(self, frame_name):
        """ Compute the Jacobian of a given frame in the robot base frame """
        return pin.getFrameJacobian(self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED)
    
    def get_jacobian_dot(self, frame_name):
        """ Compute the Jacobian of a given frame in the robot base frame """
        return pin.getFrameJacobianTimeVariation(self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED)
        
    def get_frame_transformation(self, frame_name):
        """ Compute the transformation matrix of a given frame in the robot base frame """
        return self.data.oMf[self.model.getFrameId(frame_name)].homogeneous
        

if __name__ == "__main__":
    
    arm_ik = G1WholeBodyKinematics()
