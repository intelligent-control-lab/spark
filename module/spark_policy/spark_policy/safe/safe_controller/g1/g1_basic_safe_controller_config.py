from spark_policy.safe.safe_controller import SafeControllerConfig

class G1BasicSafeControllerConfig(SafeControllerConfig):
    
    class robot( SafeControllerConfig.robot ):
        
        class cfg( SafeControllerConfig.robot.cfg ):
            class_name = "G1WholeBodyConfig"
        
        class kinematics( SafeControllerConfig.robot.kinematics ):
            class_name = "G1UpperBodyKinematics"
        
    class safety_index( SafeControllerConfig.safety_index ):
        class_name = "BasicCollisionSafetyIndex"
        min_distance = {
            "environment": 0.05,
            "self": 0.05
        }
        
        # todo move to robot config
        enable_self_collision = True
    
    class safe_algo( SafeControllerConfig.safe_algo ):
        class_name = "SafeSetAlgorithm"
        eta_ssa = 1.0
        safety_buffer = 0.1 # positive to allow hold state
        slack_weight = 1e3
        control_weight = [
            1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0
        ]
        
        # class_name = "SublevelSafeSetAlgorithm"
        # lambda_sss = 1.0
        # slack_weight = 1e3
        # control_weight = [
        #     1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0
        # ]
        
        # class_name = "ControlBarrierFunction"
        # lambda_cbf = 1
        # slack_weight = 1e3
        # control_weight = [
        #     1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        #     1.0, 1.0, 1.0
        # ]
        
        # class_name = "PotentialFieldMethod"
        # c_pfm = 0.1
        
        # class_name = "SlidingModeAlgorithm"
        # c_sma = 1.0
        