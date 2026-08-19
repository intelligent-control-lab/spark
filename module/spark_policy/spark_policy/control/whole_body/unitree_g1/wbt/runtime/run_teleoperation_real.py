import os

from spark_policy.control.whole_body.unitree_g1.wbt.runtime.config import Config, WBT_SPORT_MODE_CONFIG_DIR
from spark_policy.control.whole_body.unitree_g1.wbt.runtime.controllers.controller import Runner_online_real, Runner_handle_mujoco  #, Runner_offline_mujoco

import time

import numpy as np
import torch

torch.set_printoptions(precision=3)
np.set_printoptions(precision=3)

def deploy_real(args):
    # Load config
    config_path = os.path.join(WBT_SPORT_MODE_CONFIG_DIR, args.config)
    print(config_path)
    config = Config(config_path)

    runner = Runner_online_real(config, args=args)
    # Enter the zero torque state, press the start key to continue executing
    runner.damping_state()
    # # Enter the default position state, press the A key to continue executing
    runner.move_to_default_pos()
    runner.last_control_timestamp = time.time()
    current_mode = "SQUAT"
    print('Squat mode!')
    print('Press Left_A to start the locomotion mode!')
    cnt = 0
    while True:
        if current_mode == "LOCOMOTION":
            runner.run_loco(debug=args.debug, manual=True)
            if runner.transfer_to_squat:
                # current_mode = "SQUAT"
                print('Squat mode!')
                print('Press Left_A to start the locomotion mode!')

        elif current_mode == "SQUAT":
            # tv_arms()
            runner.run_squat(debug=args.debug, manual=True)
            if runner.transfer_to_loco:
                # current_mode = "LOCOMOTION"
                print('Locomotion mode!')
                print('Press Right_A to start the squat mode!')
        cnt += 1
        if cnt > 500:
            current_mode = "LOCOMOTION"
        if cnt > 2000:
            current_mode = "SQUAT"
        print("counter: ", cnt, "mode: ", current_mode)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--net", type=str, help="network interface")
    parser.add_argument("--config",
                        type=str,
                        help="config file name in the configs folder",
                        default="run_loco_squat_grasp.yaml")
    parser.add_argument("--save_data", action="store_true", help="whether saving the real data")
    parser.add_argument("--save_data_dir", type=str, help="where to save the data", default="./save_real_data")
    parser.add_argument("--save_image", action="store_true", help="whether saving the real image")
    parser.add_argument("--debug", action="store_true", help="")
    args = parser.parse_args()

    deploy_real(args)
