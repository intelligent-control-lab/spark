from run_kuka_iiwa14_teleop import run


if __name__ == "__main__":
    run(
        robot_cfg="KukaIIWA14DualArmDynamic1Config",
        agent_cfg="KukaIIWA14DualArmAgent",
        safe_algo="bypass",
        safety_index="si1",
    )
