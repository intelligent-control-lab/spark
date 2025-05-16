from spark_pipeline import G1SafeTeleopPipeline as Pipeline
from spark_pipeline import G1SafeTeleopSimPipelineConfig as PipelineConfig

def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure task-related settings."""
    cfg.env.task.enable_ros = False # if ROS is needed for receiving task info
    cfg.env.task.mode = "Velocity"
    cfg.env.task.num_obstacle_task = 0
    return cfg

def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Configure agent-related settings."""
    cfg.env.agent.obstacle_debug["manual_movement_step_size"] = 0.02 # Tune step size for keyboard controlled obstacles
    cfg.env.agent.obstacle_debug["num_obstacle"] = 1
    cfg.env.agent.use_sim_dynamics = False
    cfg.env.agent.enable_viewer = True
    return cfg

def config_policy_module(cfg: PipelineConfig, **kwargs):
    """Configure policy-related settings."""
    return cfg

def config_safety_module(cfg: PipelineConfig, **kwargs):
    """Configure safety-related settings."""
    # --------------------- Config Safe Control Algorithm --------------------- #
    safe_algo = kwargs.get("safe_algo", "bypass")  # Default to 'bypass' if not provided
    match safe_algo:
        case "bypass":
            cfg.algo.safe_controller.safe_algo.class_name = "ByPassSafeControl"
        
        case "ssa":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.eta_ssa = 0.1
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0,
            ]
        
        case "rssa":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.eta_ssa = 0.1
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "sss":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicSublevelSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.lambda_sss = 10.0
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
            
        case "rsss":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedSublevelSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.lambda_sss = 10.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "cbf":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicControlBarrierFunction"
            cfg.algo.safe_controller.safe_algo.lambda_cbf = 10.0
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
            
        case "rcbf":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedControlBarrierFunction"
            cfg.algo.safe_controller.safe_algo.lambda_cbf = 10.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "pfm":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicPotentialFieldMethod"
            cfg.algo.safe_controller.safe_algo.c_pfm = 1.0
        
        case "sma":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicSlidingModeAlgorithm"
            cfg.algo.safe_controller.safe_algo.c_sma = 1.0
            
    if "FixedBase" in cfg.robot.cfg.class_name:
        cfg.algo.safe_controller.safe_algo.control_weight = cfg.algo.safe_controller.safe_algo.control_weight[:-3]
    elif "RightArm" in cfg.robot.cfg.class_name:
        cfg.algo.safe_controller.safe_algo.control_weight = cfg.algo.safe_controller.safe_algo.control_weight[3:10]

    # --------------------- Config Safe Control Index --------------------- #
    safety_index = kwargs.get("safety_index", "si1")  # Default to 'si1' if not provided
    match safety_index:
        case "si1":
            cfg.algo.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndex"
        case "si1a":
            cfg.algo.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndexApprox"
        case "si2":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 1.0
        case "si2a":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndexApprox"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 1.0
        case 'si2nn':
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 2.0,
            cfg.algo.safe_controller.safety_index.phi_k = 1.0,
            cfg.algo.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"

    cfg.algo.safe_controller.safety_index.enable_self_collision = False
    cfg.algo.safe_controller.safety_index.min_distance['environment'] = 0.01

    return cfg

def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Configure pipeline settings."""
    cfg.robot.cfg.class_name = kwargs.get("robot_cfg", "G1FixedBaseDynamic2Config")
    cfg.enable_logger = False
    cfg.enable_plotter = False
    cfg.enable_safe_zone_render = False
    cfg.metric_selection.dof_pos = True
    cfg.metric_selection.dof_vel = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.trigger_safe_controller = False
    
    return cfg

def run(**kwargs):
    """Main execution block to run the benchmark pipeline."""
    # Generate initial pipeline configuration
    cfg = PipelineConfig()
    
    # Apply configuration modules in order
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)

    # Run the pipeline
    pipeline = Pipeline(cfg)
    pipeline.run(save_path = kwargs.get("save_path", None))
    
    return

if __name__ == "__main__":
    ROBOT_CFG_LIST = [
        "G1RightArmDynamic1Config ",
        "G1FixedBaseDynamic1Config",
        "G1MobileBaseDynamic1Config",
        "G1SportModeDynamic1Config",
        
        "G1RightArmDynamic2Config", 
        "G1FixedBaseDynamic2Config",
        "G1MobileBaseDynamic2Config",
        "G1SportModeDynamic2Config"
    ]
    
    run(robot_cfg = "G1FixedBaseDynamic1Config",
        safe_algo = "rssa",
        safety_index = "si1")
    
    
    