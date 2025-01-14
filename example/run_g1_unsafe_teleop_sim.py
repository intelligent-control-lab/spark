from spark_pipeline import G1SafeTeleopSimPipeline, G1UnsafeTeleopSimPipelineConfig

if __name__ == "__main__":
    
    cfg = G1UnsafeTeleopSimPipelineConfig()
    
    # ------------------------- change g1 model if needed ------------------------ #
    
    # --------------------- configure safe control algorithm --------------------- #
    
    # --------------------------- other configurations --------------------------- #
    cfg.env.task.enable_ros = False # if ROS is needed for receiving task info
    cfg.env.agent.obstacle_debug["manual_movement_step_size"] = 0.1 # Tune step size for keyboard controlled obstacles
    
    # ------------------------------- run pipeline ------------------------------- #
    pipeline = G1SafeTeleopSimPipeline(cfg)
    pipeline.run()