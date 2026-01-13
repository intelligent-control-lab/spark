# -----------------------------------------------------------------------------
# Author: Ruixuan Liu
# Email: ruixuanl@andrew.cmu.edu
# -----------------------------------------------------------------------------

from spark_policy.base.base_policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
from spark_robot import IIWA14DualDynamic1Config, IIWA14DualKinematics
import numpy as np

import pinocchio as pin
import numpy as np
import random
import time
from pinocchio.visualize import MeshcatVisualizer
import meshcat
from queue import PriorityQueue
import os
from scipy.spatial.transform import Rotation as R
from numpy.linalg import norm, solve
from scipy.optimize import least_squares


class Node:
    def __init__(self, q, from_g, from_q, to_q):
        self.q = np.copy(q)
        self.q_tuple = tuple(q)
        self.parent = None
        self.g = from_g + np.linalg.norm(q - from_q)
        self.h = np.linalg.norm(q - to_q)
        self.f = self.g + self.h

class RRTConnect:
    def __init__(self, urdf_model, collision_model, goal_tol=5, step_size=1, max_iter=500):
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_tol = goal_tol
        self.model = urdf_model
        self.collision_model = collision_model
        self.data = self.model.createData()
        self.collision_data = self.collision_model.createData()
        self.num_samples = 8
        self.connection_resolution = max(int(goal_tol / 0.1), 1)

    def determine_joint_ranges(self):
        self.joint_ranges = []
        for joint_name in self.planning_joint_list:
            joint_id = self.model.getJointId(joint_name)
            idx_q = self.model.joints[joint_id].idx_q
            lower = self.model.lowerPositionLimit[idx_q]
            upper = self.model.upperPositionLimit[idx_q]
            self.joint_ranges.append((lower, upper))
        self.joint_ranges = np.asarray(self.joint_ranges)

    def parse_joint_names(self, planning_joint_list):
        self.joint_name_to_idx = dict()
        for joint_name in planning_joint_list:
            joint_id = self.model.getJointId(joint_name)
            idx_q = self.model.joints[joint_id].idx_q
            self.joint_name_to_idx[joint_name] = idx_q

    def plan(self, start, goal, cur_full_q, planning_joint_list):
        self.start = np.copy(start)
        self.goal = np.copy(goal)
        self.cur_full_q = np.asarray(cur_full_q)
        self.planning_joint_list = planning_joint_list
        self.determine_joint_ranges()
        self.parse_joint_names(planning_joint_list)
        self.start_node = Node(self.start, 0, self.start, self.goal)
        self.goal_node = Node(self.goal, 0, self.goal, self.start)
        if(not self.is_collision_free(self.start_node) or not self.is_collision_free(self.goal_node)):
            return [], False
        
        forward_node = Node(self.start, self.start_node.g, self.start_node.q, self.goal_node.q)
        forward_queue = PriorityQueue()
        forward_queue.put((forward_node.f, forward_node))

        backward_node = Node(self.goal, self.goal_node.g, self.goal_node.q, self.start_node.q)
        backward_queue = PriorityQueue()
        backward_queue.put((backward_node.f, backward_node))

        open_set = {forward_node.q_tuple, backward_node.q_tuple}
        forward = False

        for iter in range(self.max_iter):
            forward = not forward
            # print(iter, forward, forward_queue.qsize(), backward_queue.qsize())
            if(forward):
                if(forward_queue.qsize() == 0):
                    continue
                cur_node = forward_queue.get()[1]
                reached_node, reached = self.reached_node_in_list(cur_node, backward_queue)
                if(reached):
                    # print("Reached goal from forward!")
                    path = self.construct_path(cur_node, reached_node, forward)
                    return path, True
                for i in range(self.num_samples):
                    rand_node = self.get_random_node(cur_node, self.goal_node)
                    if(self.is_collision_free(rand_node) and rand_node.q_tuple not in open_set):
                        next_node = self.steer(cur_node, rand_node, self.goal_node)
                        if(self.equal_nodes(next_node, cur_node)):
                            continue
                        next_node.parent = cur_node
                        open_set.add(next_node.q_tuple)
                        forward_queue.put((next_node.f, next_node))
            else:
                if(backward_queue.qsize() == 0):
                    continue
                cur_node = backward_queue.get()[1]
                reached_node, reached = self.reached_node_in_list(cur_node, forward_queue)
                if(reached):
                    # print("Reached goal from backward!")
                    path = self.construct_path(cur_node, reached_node, forward)
                    return path, True
                for i in range(self.num_samples):
                    rand_node = self.get_random_node(cur_node, self.start_node)
                    if(self.is_collision_free(rand_node) and rand_node.q_tuple not in open_set):
                        next_node = self.steer(cur_node, rand_node, self.start_node)
                        if(self.equal_nodes(next_node, cur_node)):
                            continue
                        next_node.parent = cur_node
                        open_set.add(next_node.q_tuple)
                        backward_queue.put((next_node.f, next_node))
        return [], False
            
    def equal_nodes(self, a, b):
        return np.all(a.q == b.q)
    
    def random_q(self, from_node):
        from_q = from_node.q
        joint_lows = self.joint_ranges[:, 0]
        joint_highs = self.joint_ranges[:, 1]
        low_bounds = np.maximum(from_q - self.step_size, joint_lows)
        high_bounds = np.minimum(from_q + self.step_size, joint_highs)
        return np.random.uniform(low_bounds, high_bounds)
    
    def get_random_node(self, start_node, goal_node):
        q = self.random_q(start_node)
        rand_node = Node(q, start_node.g, start_node.q, goal_node.q)
        return rand_node

    def reached_node_in_list(self, cur_node, open_queue):
        tmp = []
        node = cur_node
        reached = False
        while(not open_queue.empty()):
            element = open_queue.get()
            tmp.append(element)
            node = element[1]
            if(self.reachedNodes(cur_node, node)):
                reached = True
                break
        for e in tmp:
            open_queue.put(e)
        return node, reached

    def dist_between_nodes(self, a, b):
        return np.linalg.norm(a.q - b.q)

    def reachedNodes(self, a, b):
        if(self.dist_between_nodes(a, b) < self.goal_tol and self.validP2P(a, b)):
            return True
        return False
    
    def validP2P(self, a, b):
        diff = b.q - a.q
        interval = diff / self.connection_resolution
        for i in range(1, self.connection_resolution):
            tmp_node = Node(a.q + i * interval, a.g, a.q, b.q)
            if(not self.is_collision_free(tmp_node)):
                return False
        return True
    
    def steer(self, from_node, to_node, goal_node):
        diff = to_node.q - from_node.q
        pre_node = from_node
        interval = diff / self.connection_resolution
        for i in range(1, self.connection_resolution):
            tmp_node = Node(from_node.q + i * interval, from_node.g, from_node.q, goal_node.q)
            if(not self.is_collision_free(tmp_node)):
                return pre_node
            pre_node = tmp_node
        return to_node

    def full_robot_config(self, node):
        full_q = np.copy(self.cur_full_q)
        arm_q = node.q
        cnt = 0
        for joint_name in self.planning_joint_list:
            full_q[self.joint_name_to_idx[joint_name]] = arm_q[cnt]
            cnt += 1
        return full_q

    def is_collision_free(self, node):
        full_robot_q = self.full_robot_config(node)
        pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, full_robot_q, True)
        return not any(r.isCollision() for r in self.collision_data.collisionResults)

    def construct_path(self, node1, node2, forward):
        path1 = []
        path2 = []
        node = node1
        while node is not None:
            path1.append(node.q)
            node = node.parent
        node = node2
        while(node is not None):
            path2.append(node.q)
            node = node.parent
        if(forward):
            path1.reverse()
            path = path1 + path2
        else:
            path2.reverse()
            path = path2 + path1
        return path
    
    def check_path_collision_free(self, path):
        for p in path:
            node = Node(p, 0, p, p)
            cf = self.is_collision_free(node)
            if(not cf):
                return cf
        return True

class RRTConnectPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.model, self.collision_model, self.visual_model = self.robot_kinematics.model, self.robot_kinematics.collision_model, self.robot_kinematics.visual_model
        self.rrt = RRTConnect(self.model, self.collision_model)

    # def plan(self, waypoints, robot_config, planning_joint_list, step_size=0.01):
    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        waypoints = task_info["waypoints"]
        robot_config = agent_feedback["dof_pos_fbk"]
        planning_joint_list = task_info["planning_joint_list"]
        step_size = task_info.get("step_size", 0.05)
        
        path = [waypoints[0]]
        for start_i in range(len(waypoints) - 1):
            if(start_i == 0):
                start = waypoints[0]
            else:
                start = path[-1]
            goal = waypoints[start_i + 1]
            new_path, status = self.rrt.plan(start, goal, robot_config, planning_joint_list)
            if(status):
                path = path + new_path[1:]
            else:
                print("Failed")
                return path
        
        dense_path = []
        for start_i in range(len(path) - 1):
            start_q = path[start_i]
            end_q = path[start_i + 1]
            resolution = int(np.ceil(np.max(abs(end_q - start_q) / step_size)))
            if resolution < 2:
                resolution = 2
            interval = (end_q - start_q) / resolution
            dense_path.append(start_q)
            for i in range(1, resolution):
                next_q = start_q + i * interval
                dense_path.append(next_q)
        dense_path.append(path[-1])
        # status = self.rrt.check_path_collision_free(dense_path)
        # print("Collision free path:", status)
        return dense_path, info

if __name__ == "__main__":

    robot_cfg = G1DualArmDynamic1Config()
    robot_kinematics = G1FDualArmBaseKinematics(robot_cfg)
    rrt_policy = RRTConnectPolicy(robot_cfg, robot_kinematics)
    cur_config = np.array([q for _, q in robot_cfg.DefaultDoFVal.items()])

    right_cart = np.array([[ 1. ,        0  ,        0.   ,       0.5],
                                [ 0.     ,     1.0      ,   0.0        , -0.6],
                                [-0.      ,      0.0,        1.0    ,     0.2],
                                [ 0.       ,   0. ,         0.     ,     1.        ]])
    left_cart = np.array([[ 1. ,        0  ,        0.   ,       0.7],
                                [ 0.     ,     1.      ,   0.        , 0.3],
                                [-0.      ,    0.  ,        1.    ,     0.5],
                                [ 0.       ,   0. ,         0.     ,     1.        ]])
    
    dof_pos_target, _ = robot_kinematics.inverse_kinematics([left_cart, right_cart], current_lr_arm_motor_q=cur_config)
    # dof_pos_target[5] = np.pi
    waypoints = [cur_config, dof_pos_target]
    model = robot_kinematics.model

    for jid, joint in enumerate(model.joints):
        print(jid, model.names[jid])
    plan_arm_joints = [model.names[idx] for idx in range(1, model.njoints) if model.joints[idx].nq > 0]
    task_info = {}
    agent_feedback = {}
    task_info["waypoints"] = waypoints
    task_info["planning_joint_list"] = plan_arm_joints
    agent_feedback["dof_pos_fbk"] = cur_config
    import ipdb; ipdb.set_trace()
    path = rrt_policy.act(agent_feedback, task_info)
    for dof_pos_target in path[0]:
        robot_kinematics.visualize(dof_pos_target)
        time.sleep(0.1)
   