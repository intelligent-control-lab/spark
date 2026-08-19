from spark_pipeline.base.base_goal_pipeline import BaseGoalPipeline


class TeleopPipeline(BaseGoalPipeline):
    def __init__(self, cfg):
        super().__init__(cfg)
