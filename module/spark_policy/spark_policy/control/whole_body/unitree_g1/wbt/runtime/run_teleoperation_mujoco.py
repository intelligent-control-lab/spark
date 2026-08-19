import os

from spark_policy.control.whole_body.unitree_g1.wbt.runtime.config import Config, WBT_SPORT_MODE_CONFIG_DIR
from spark_policy.control.whole_body.unitree_g1.wbt.runtime.controllers.controller import Runner_handle_mujoco_vision  #, Runner_offline_mujoco

import time

import numpy as np
import torch

torch.set_printoptions(precision=3)
np.set_printoptions(precision=3)


def deploy_handle_mujoco(args):
    import mujoco.viewer
    import mujoco
    config_path = os.path.join(WBT_SPORT_MODE_CONFIG_DIR, args.config)
    print(config_path)
    config = Config(config_path)

    # Initialize DDS communication
    runner = Runner_handle_mujoco_vision(config, args=args)
    current_mode = "LOCOMOTION"
    print('Squat mode!')
    print('Press Left_A to start the locomotion mode!')

    with mujoco.viewer.launch_passive(runner.m, runner.d) as viewer:
        runner.last_control_timestamp = time.time()
        while True:
            if current_mode == "LOCOMOTION":
                runner.run_loco(manual=True)
                viewer.sync()
                if runner.transfer_to_squat:
                    current_mode = "SQUAT"
                    print('Squat mode!')
                    print('Press Left_A to start the locomotion mode!')

            elif current_mode == "SQUAT":
                # tv_arms()
                runner.run_squat(manual=True)
                viewer.sync()
                if runner.transfer_to_loco:
                    current_mode = "LOCOMOTION"
                    print('Locomotion mode!')
                    print('Press Right_A to start the squat mode!')


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config",
                        type=str,
                        help="config file name in the configs folder",
                        default="run_loco_squat_grasp.yaml")
    parser.add_argument("--save_data", action="store_true", help="whether saving the mujoco data")
    parser.add_argument("--save_data_dir", type=str, help="where to save the data", default="./save_mujoco_data")
    parser.add_argument("--save_image", action="store_true", help="whether saving the mujoco image")
    args = parser.parse_args()

    deploy_handle_mujoco(args)
