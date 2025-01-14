from spark_pipeline import G1SafeTeleopSimPipeline, G1SafeTeleopSimPipelineConfig
from spark_utils import update_class_attributes

if __name__ == "__main__":
    
    cfg = G1SafeTeleopSimPipelineConfig()
    
    # ------------------------- change g1 model if needed ------------------------ #
    cfg.robot.cfg.class_name = "G1BasicConfig" # "G1BasicConfig", "G1RightArmConfig", etc.
    
    # --------------------- configure safe control algorithm --------------------- #
    safe_control_algo = 'ssa' # 'ssa', 'sss', 'cbf', 'pfm', 'sma'
    params = {
        'ssa': dict(
                    class_name = "SafeSetAlgorithm",
                    eta = 1.0,
                    safety_buffer = 0.1, # positive to allow hold state
                    slack_weight = 1e3,
                    control_weight = [
                        1.0, 1.0, 1.0, # waist
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # left arm
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # right arm
                        10.0, 10.0, 10.0 # locomotion
                    ]
                ),
        'sss': dict(
                    class_name = "SublevelSafeSetAlgorithm",
                    lambda_SSS = 1.0,
                    slack_weight = 1e3,
                    control_weight = [
                        1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0
                    ]
                ),
        'cbf': dict(
                    class_name = "ControlBarrierFunction",
                    lambda_cbf = 10,
                    slack_weight = 1e3,
                    control_weight = [
                        1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                        1.0, 1.0, 1.0
                    ]
                ),
        'pfm': dict(
                    class_name = "PotentialFieldMethod",
                    lambda_pfm = 0.1
                ),
        'sma': dict(
                    class_name = "SlidingModeAlgorithm",
                    c_2 = 1.0
                )
    }
    
    update_class_attributes(cfg.algo.safe_controller.safe_algo, params[safe_control_algo])
    
    # --------------------------- other configurations --------------------------- #
    cfg.env.task.enable_ros = False # if ROS is needed for receiving task info
    cfg.env.agent.obstacle_debug["manual_movement_step_size"] = 0.1 # Tune step size for keyboard controlled obstacles
    
    # ------------------------------- run pipeline ------------------------------- #
    pipeline = G1SafeTeleopSimPipeline(cfg)
    pipeline.run()