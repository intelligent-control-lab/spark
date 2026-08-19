from __future__ import annotations

import inspect

import spark_pipeline
import spark_robot


def _exported_pipeline_configs():
    abstract_configs = {
        spark_pipeline.BasePipelineConfig,
        spark_pipeline.BenchmarkPipelineConfig,
        spark_pipeline.TeleopPipelineConfig,
    }
    for name in sorted(dir(spark_pipeline)):
        config_type = getattr(spark_pipeline, name)
        if (
            inspect.isclass(config_type)
            and issubclass(config_type, spark_pipeline.BasePipelineConfig)
            and config_type not in abstract_configs
        ):
            yield name, config_type


def test_pipeline_defaults_match_robot_backend_mappings(capsys):
    checked = 0
    for name, config_type in _exported_pipeline_configs():
        config = config_type()
        robot_name = config.robot.cfg.class_name
        agent_name = config.env.agent.class_name

        assert robot_name, f"{name} has no default robot config"
        assert agent_name, f"{name} has no default environment agent"
        assert hasattr(spark_robot, robot_name), f"{name}: unknown robot {robot_name}"

        robot_config = getattr(spark_robot, robot_name)()
        assert agent_name in set(robot_config.agent_class_names.values()), (
            f"{name}: {agent_name} is not a declared backend for {robot_name}"
        )
        checked += 1

    assert checked > 0
    assert capsys.readouterr().out == ""
