from spark_pipeline import BenchmarkPipeline as Pipeline
from spark_pipeline import G1BenchmarkPipelineConfig as PipelineConfig

def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure task-related settings."""
    cfg.env.task.num_obstacle_task = 1
    cfg.env.task.max_episode_length = 2000
    cfg.env.task.obstacle_direction = [0.0, 1.0, 0.0]
    cfg.env.task.obstacle_init = [0.2, -1.0, 0.9]
    cfg.env.task.obstacle_velocity = 0.15
    cfg.env.task.goal_left_velocity = 0.0
    cfg.env.task.goal_right_velocity = 0.0
    cfg.env.task.base_goal_enable = False
    cfg.env.task.mode = "Velocity"

    return cfg

def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Configure agent-related settings."""
    cfg.env.agent.enable_viewer = True
    cfg.env.agent.use_sim_dynamics = False
    
    return cfg

def config_policy_module(cfg: PipelineConfig, **kwargs):
    """Configure policy-related settings."""
    # Use the default policy configuration
    
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
            cfg.algo.safe_controller.safe_algo.eta_ssa = 1.0
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "sss":
            cfg.algo.safe_controller.safe_algo.class_name = "SublevelSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.lambda_sss = 1.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "cbf":
            cfg.algo.safe_controller.safe_algo.class_name = "ControlBarrierFunction"
            cfg.algo.safe_controller.safe_algo.lambda_cbf = 1.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "pfm":
            cfg.algo.safe_controller.safe_algo.class_name = "PotentialFieldMethod"
            cfg.algo.safe_controller.safe_algo.c_pfm = 1.0
        
        case "sma":
            cfg.algo.safe_controller.safe_algo.class_name = "SlidingModeAlgorithm"
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
        case "si2":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 1.0
        case 'si2nn':
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 2,
            cfg.algo.safe_controller.safety_index.phi_k = 1,
            cfg.algo.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"

    return cfg

def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Configure pipeline settings."""
    cfg.robot.cfg.class_name = kwargs.get("robot_cfg", "G1FixedBaseDynamic2Config")
    cfg.max_num_steps = 2000
    cfg.enable_logger = True
    cfg.enable_plotter = True
    cfg.enable_safe_zone_render = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.dist_self = True
    cfg.metric_selection.dist_robot_to_env = True
    cfg.metric_selection.dist_goal_arm = True
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    
    return cfg

def run(**kwargs):
    """Main execution block to run the benchmark pipeline."""
    # List of test cases to choose from
    
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
    save_path = "./log"
    run(robot_cfg = "G1FixedBaseDynamic2Config",
        safe_algo = "rssa",
        safety_index = "si2",
        save_path = save_path)
