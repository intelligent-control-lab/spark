#!/usr/bin/env python3
"""Check repository hygiene and lightweight release invariants."""

from __future__ import annotations

import builtins
import inspect
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import warnings
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SETUPS = (
    ROOT / "module/spark_agent/setup.py",
    ROOT / "module/spark_env/setup.py",
    ROOT / "module/spark_policy/setup.py",
    ROOT / "module/spark_robot/setup.py",
    ROOT / "module/spark_task/setup.py",
    ROOT / "module/spark_utils/setup.py",
    ROOT / "pipeline/setup.py",
)
ROBOT_PACKAGE_DATA = (
    ROOT
    / "module/spark_robot/spark_robot/fanuc_lrmate200id/config/fanuc_lrmate200id_single_arm_collision_spheres.json",
    ROOT
    / "module/spark_robot/spark_robot/galaxea_r1lite/config/galaxea_r1lite_collision_spheres.json",
    ROOT
    / "module/spark_robot/spark_robot/kinova_gen3/config/kinova_gen3_single_arm_collision_spheres.json",
    ROOT
    / "module/spark_robot/spark_robot/kuka_iiwa14/config/kuka_iiwa14_single_arm_collision_spheres.json",
)
LEGAL_RELEASE_FILES = (
    ROOT / "THIRD_PARTY_NOTICES.md",
    ROOT
    / "module/spark_policy/spark_policy/control/whole_body/unitree_g1/wbt/runtime/LICENSE.OpenWBT",
    ROOT
    / "module/spark_policy/spark_policy/control/whole_body/unitree_g1/wbt/runtime/NOTICE.OpenWBT.md",
)
CACHE_DIR_NAMES = {
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
}
LOCAL_SCAN_EXCLUDED_DIR_NAMES = {
    ".git",
    ".venv",
    "venv",
}
OPTIONAL_IMPORT_ROOTS = {
    "curobo",
    "cyclonedds",
    "cv2",
    "isaaclab",
    "isaaclab_tasks",
    "isaacsim",
    "mujoco",
    "omni",
    "onnx",
    "onnx2torch",
    "onnxruntime",
    "rclpy",
    "rospy",
    "torch",
    "unitree_dds_wrapper",
    "unitree_sdk2py",
    "warp",
}


def fail(errors: list[str], message: str) -> None:
    errors.append(message)


def tracked_files() -> tuple[Path, ...]:
    output = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    ).stdout
    return tuple(ROOT / item.decode() for item in output.split(b"\0") if item)


def check_hygiene(errors: list[str]) -> None:
    forbidden_suffixes = {".pyc", ".pyo", ".log"}
    for path in tracked_files():
        relative = path.relative_to(ROOT)
        if any(part in CACHE_DIR_NAMES or part.endswith(".egg-info") for part in relative.parts):
            fail(errors, f"generated directory is tracked: {relative}")
        if path.suffix in forbidden_suffixes:
            fail(errors, f"generated file is tracked: {relative}")

    # A repository-local virtual environment is an intentional installation
    # target, not repository metadata. Prune it (and Git) before scanning so
    # importing this checker through ``.venv/bin/python`` cannot make the
    # hygiene result fail by recreating site-packages/__pycache__ directories.
    for directory, dir_names, _ in os.walk(ROOT, topdown=True):
        parent = Path(directory)
        retained = []
        for name in dir_names:
            if name in LOCAL_SCAN_EXCLUDED_DIR_NAMES:
                continue
            path = parent / name
            relative = path.relative_to(ROOT)
            if name in CACHE_DIR_NAMES or name.endswith(".egg-info"):
                fail(errors, f"local cache/build metadata remains: {relative}")
                continue
            retained.append(name)
        dir_names[:] = retained

    for relative in (
        "back_up",
        "backup",
        "module/spark_policy/spark_policy/control/learned",
        "module/spark_policy/spark_policy/control/whole_body/unitree_g1/wbt/runtime/teleop/robot_control/robot_hand_inspire.py",
    ):
        if (ROOT / relative).exists():
            fail(errors, f"obsolete source path remains: {relative}")


def check_local_path_leaks(errors: list[str]) -> None:
    """Reject tracked references to a developer's home directory."""
    patterns = (
        r"(^|[^[:alnum:]:])/(home|Users)/[^/[:space:]\"'<>]+(/[^[:space:]\"'<>]*)?",
        r"(^|[^[:alnum:]:])/(root)(/[^[:space:]\"'<>]*)?",
        r"(^|[^[:alnum:]])[A-Za-z]:[\\/]Users[\\/][^\\/[:space:]\"'<>]+"
        r"([\\/][^[:space:]\"'<>]*)?",
    )
    leaked_lines: list[str] = []
    for pattern in patterns:
        completed = subprocess.run(
            ["git", "grep", "-n", "-I", "-E", pattern, "--", "."],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode not in (0, 1):
            fail(
                errors, f"could not scan tracked files for local paths: {completed.stderr.strip()}"
            )
            return
        leaked_lines.extend(line for line in completed.stdout.splitlines() if line)

    for line in dict.fromkeys(leaked_lines):
        fail(errors, f"developer-local path is tracked: {line}")


def check_general_toolbox_boundary(errors: list[str]) -> None:
    course_markers = (
        re.compile(r"16[_ -]?714", re.IGNORECASE),
        re.compile(r"homework", re.IGNORECASE),
    )
    for source_root in (ROOT / "module", ROOT / "pipeline", ROOT / "example"):
        for path in source_root.rglob("*.py"):
            text = path.read_text(encoding="utf-8", errors="replace")
            if any(pattern.search(text) for pattern in course_markers):
                fail(
                    errors,
                    f"course-specific marker found in SPARK source: {path.relative_to(ROOT)}",
                )


def check_versions(errors: list[str]) -> None:
    versions: dict[Path, str] = {}
    pattern = re.compile(r'version\s*=\s*["\']([^"\']+)["\']')
    spark_requirement_pattern = re.compile(r'["\'](spark_[a-z_]+)==([^"\']+)["\']')
    for setup_path in PACKAGE_SETUPS:
        if not setup_path.is_file():
            fail(errors, f"missing package metadata: {setup_path.relative_to(ROOT)}")
            continue
        setup_text = setup_path.read_text(encoding="utf-8")
        match = pattern.search(setup_text)
        if match is None:
            fail(errors, f"missing package version: {setup_path.relative_to(ROOT)}")
            continue
        versions[setup_path] = match.group(1)
        if "find_namespace_packages" not in setup_text:
            fail(
                errors,
                f"package discovery may omit namespace directories: {setup_path.relative_to(ROOT)}",
            )
        if 'license="MIT"' not in setup_text:
            fail(
                errors,
                f"package does not declare the MIT code license: {setup_path.relative_to(ROOT)}",
            )
        if 'url="https://github.com/intelligent-control-lab/spark"' not in setup_text:
            fail(errors, f"package has no canonical project URL: {setup_path.relative_to(ROOT)}")
    if len(set(versions.values())) > 1:
        detail = ", ".join(f"{path.parent.name}={version}" for path, version in versions.items())
        fail(errors, f"SPARK package versions disagree: {detail}")
        return
    if not versions:
        return

    release_version = next(iter(versions.values()))
    for setup_path in PACKAGE_SETUPS:
        setup_text = setup_path.read_text(encoding="utf-8")
        for package_name, dependency_version in spark_requirement_pattern.findall(setup_text):
            if dependency_version != release_version:
                fail(
                    errors,
                    f"{setup_path.relative_to(ROOT)} pins {package_name}=={dependency_version}; "
                    f"expected {release_version}",
                )

    changelog_text = (ROOT / "CHANGELOG.md").read_text(encoding="utf-8")
    if f"## {release_version} -" not in changelog_text:
        fail(errors, f"CHANGELOG.md has no dated {release_version} release section")

    docs_config = (ROOT / "docs/conf.py").read_text(encoding="utf-8")
    if f'release = "{release_version}"' not in docs_config:
        fail(errors, f"docs/conf.py does not declare release {release_version}")

    for legal_path in LEGAL_RELEASE_FILES:
        if not legal_path.is_file():
            fail(errors, f"missing legal release file: {legal_path.relative_to(ROOT)}")

    policy_setup = (ROOT / "module/spark_policy/setup.py").read_text(encoding="utf-8")
    for packaged_notice in ("LICENSE.OpenWBT", "NOTICE.OpenWBT.md"):
        if packaged_notice not in policy_setup:
            fail(errors, f"spark_policy wheel omits {packaged_notice}")

    dockerfile = ROOT / "Dockerfile"
    compose_file = ROOT / "docker-compose.yml"
    for container_path in (
        dockerfile,
        compose_file,
        ROOT / ".dockerignore",
        ROOT / "docker/entrypoint.sh",
        ROOT / "run_with_docker.sh",
    ):
        if not container_path.is_file():
            fail(errors, f"missing container release file: {container_path.relative_to(ROOT)}")
    for executable_path in (ROOT / "docker/entrypoint.sh", ROOT / "run_with_docker.sh"):
        if executable_path.is_file() and not os.access(executable_path, os.X_OK):
            fail(
                errors, f"container launcher is not executable: {executable_path.relative_to(ROOT)}"
            )

    if dockerfile.is_file():
        docker_text = dockerfile.read_text(encoding="utf-8")
        if f"ARG SPARK_VERSION={release_version}" not in docker_text:
            fail(errors, f"Dockerfile does not default SPARK_VERSION to {release_version}")
    if compose_file.is_file():
        compose_text = compose_file.read_text(encoding="utf-8")
        for service in ("spark-core", "spark-default", "spark-isaac", "spark-ros", "spark-wsl"):
            if not re.search(rf"^  {re.escape(service)}:\s*$", compose_text, re.MULTILINE):
                fail(errors, f"docker-compose.yml is missing the {service} service")
        compose_versions = re.findall(r"SPARK_VERSION:\s*[\"']?([^\s\"']+)", compose_text)
        if not compose_versions:
            fail(errors, "docker-compose.yml declares no SPARK_VERSION build arguments")
        elif set(compose_versions) != {release_version}:
            fail(
                errors,
                "docker-compose.yml has inconsistent SPARK_VERSION values: "
                f"{sorted(set(compose_versions))}",
            )

    robot_setup_text = (ROOT / "module/spark_robot/setup.py").read_text(encoding="utf-8")
    for data_path in ROBOT_PACKAGE_DATA:
        if not data_path.is_file():
            fail(errors, f"missing robot package data: {data_path.relative_to(ROOT)}")
            continue
        package_name = ".".join(data_path.parent.relative_to(ROOT / "module/spark_robot").parts)
        if f'"{package_name}": ["*.json"]' not in robot_setup_text:
            fail(
                errors,
                f"robot package data is not declared for installed wheels: {package_name}",
            )


def check_optional_import_boundary(errors: list[str]) -> None:
    original_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0 and name.split(".", 1)[0] in OPTIONAL_IMPORT_ROOTS:
            raise ModuleNotFoundError(f"optional dependency blocked by release check: {name}")
        return original_import(name, globals, locals, fromlist, level)

    builtins.__import__ = guarded_import
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            for package in (
                "spark_agent",
                "spark_env",
                "spark_policy",
                "spark_policy.control.whole_body.unitree_g1",
                "spark_robot",
                "spark_task",
                "spark_utils",
                "spark_pipeline",
            ):
                __import__(package)
    except Exception as exc:
        fail(errors, f"base import requires an optional dependency: {exc}")
    finally:
        builtins.__import__ = original_import


def check_robot_contracts(errors: list[str]) -> None:
    import spark_agent
    import spark_robot
    from spark_robot import RobotConfig

    resource_root = Path(spark_robot.SPARK_ROBOT_RESOURCE_DIR)
    for name in sorted(dir(spark_robot)):
        config_type = getattr(spark_robot, name)
        if (
            not inspect.isclass(config_type)
            or config_type is RobotConfig
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
        ):
            continue
        parameters = inspect.signature(config_type).parameters.values()
        if any(
            parameter.default is inspect.Parameter.empty
            and parameter.kind not in (parameter.VAR_POSITIONAL, parameter.VAR_KEYWORD)
            for parameter in parameters
        ):
            continue

        try:
            config = config_type()
        except Exception as exc:
            fail(errors, f"robot config cannot be constructed: {name}: {exc}")
            continue

        prefix = f"robot config {name}"
        agents = dict(getattr(config, "agent_class_names", {}))
        if not agents:
            fail(errors, f"{prefix} declares no backend agent")
        for backend, agent_name in agents.items():
            if agent_name not in dir(spark_agent):
                fail(errors, f"{prefix} maps {backend!r} to unknown agent {agent_name!r}")
        capabilities = config.backend_capabilities()
        if set(capabilities) != set(agents):
            fail(errors, f"{prefix} structured backend capabilities do not match its mappings")
        for backend, capability in capabilities.items():
            if capability.agent_class_name != agents[backend]:
                fail(errors, f"{prefix} has inconsistent {backend!r} capability metadata")

        model_only = bool(getattr(config, "model_only", False))
        if model_only and set(agents) != {"dynamics"}:
            fail(errors, f"{prefix} is model-only but declares backends {sorted(agents)}")
        if not model_only and "mujoco" not in agents:
            fail(errors, f"{prefix} has no MuJoCo agent mapping")
        if not model_only and "isaac" not in agents:
            fail(errors, f"{prefix} has no Isaac agent mapping")
        if not model_only and "isaac" in capabilities:
            isaac_capability = capabilities["isaac"]
            if not isaac_capability.single_environment:
                fail(errors, f"{prefix} has no single-environment Isaac support")
            if not isaac_capability.batched or not isaac_capability.tensor_io:
                fail(errors, f"{prefix} has no batched tensor Isaac support")
            if not isaac_capability.rendering:
                fail(errors, f"{prefix} has no Isaac rendering support")

        dofs = set(config.DoFs)
        controls = set(config.Control)
        if set(config.DefaultDoFVal) != dofs:
            fail(errors, f"{prefix} DefaultDoFVal does not cover its DoFs")
        if set(config.ControlLimit) != controls:
            fail(errors, f"{prefix} ControlLimit does not cover its controls")
        control_groups = (
            set(config.NormalControl),
            set(config.WeakControl),
            set(config.DelicateControl),
        )
        if set.union(*control_groups) != controls:
            fail(errors, f"{prefix} control classes do not cover its controls")
        if any(
            control_groups[left] & control_groups[right] for left, right in ((0, 1), (0, 2), (1, 2))
        ):
            fail(errors, f"{prefix} control classes overlap")

        motors = set(config.RealMotors)
        if "real" in agents and set(config.RealMotorPosLimit) != motors:
            fail(errors, f"{prefix} real backend lacks complete motor limits")
        if set(config.MujocoDoF_to_DoF) != set(config.MujocoDoFs):
            fail(errors, f"{prefix} MuJoCo DoF mapping is incomplete")
        if not set(config.MujocoDoF_to_DoF.values()).issubset(dofs):
            fail(errors, f"{prefix} MuJoCo DoF mapping targets invalid DoFs")
        if not set(config.DoF_to_MujocoDoF).issubset(dofs):
            fail(errors, f"{prefix} inverse MuJoCo DoF mapping has invalid keys")
        if set(config.MujocoMotor_to_Control) != set(config.MujocoMotors):
            fail(errors, f"{prefix} MuJoCo motor mapping is incomplete")
        if not set(config.MujocoMotor_to_Control.values()).issubset(controls):
            fail(errors, f"{prefix} MuJoCo motor mapping targets invalid controls")
        if not set(config.RealMotor_to_Control).issubset(motors):
            fail(errors, f"{prefix} real-motor mapping has invalid keys")
        if not set(config.RealMotor_to_Control.values()).issubset(controls):
            fail(errors, f"{prefix} real-motor mapping targets invalid controls")
        if not set(config.CollisionVol).issubset(set(config.Frames)):
            fail(errors, f"{prefix} collision volumes target invalid frames")

        model_path = getattr(config, "mujoco_model_path", None)
        if "mujoco" in agents:
            if not model_path:
                fail(errors, f"{prefix} has a MuJoCo agent but no model path")
            elif not (resource_root / model_path).is_file():
                fail(errors, f"{prefix} references missing MuJoCo model {model_path}")
        kinematics_name = getattr(config, "kinematics_class_name", None)
        if not model_only and not (kinematics_name and hasattr(spark_robot, kinematics_name)):
            fail(errors, f"{prefix} references unknown kinematics {kinematics_name!r}")

        isaac_spec = config.isaac_articulation
        if isaac_spec is not None:
            if not agents.get("isaac", "").endswith("IsaacAgent"):
                fail(errors, f"{prefix} has Isaac metadata but no matching Isaac agent")
            urdf_path = resource_root / isaac_spec.urdf_path
            if not urdf_path.is_file():
                fail(errors, f"{prefix} references missing Isaac URDF {isaac_spec.urdf_path}")
            else:
                urdf_joints = {
                    joint.attrib["name"]
                    for joint in ET.parse(urdf_path).findall(".//joint")
                    if joint.attrib.get("type") != "fixed"
                }
                declared_joints = (
                    {
                        f"{instance.prefix}_{joint_name}"
                        for instance in isaac_spec.instances
                        for joint_name in urdf_joints
                    }
                    if isaac_spec.instances
                    else urdf_joints
                )
                if isaac_spec.planar_base is not None:
                    declared_joints = declared_joints | set(isaac_spec.planar_base.joint_names)
                missing_joints = set(isaac_spec.joint_names) - declared_joints
                if missing_joints:
                    fail(
                        errors,
                        f"{prefix} Isaac URDF is missing joints {sorted(missing_joints)}",
                    )
            if len(isaac_spec.joint_names) != len(config.DoFs):
                fail(errors, f"{prefix} Isaac joint metadata does not match its DoF count")

        try:
            model = config.create_dynamics_model()
            state = np.zeros(model.state_dim, dtype=float)
            control = np.zeros(model.control_dim, dtype=float)
            next_state = model.step(
                state,
                control,
                0.01,
                getattr(model, "default_integrator", "Euler"),
            )
            if np.asarray(next_state).shape != (model.state_dim,):
                fail(errors, f"{prefix} dynamics step returns the wrong shape")
            if not np.isfinite(next_state).all():
                fail(errors, f"{prefix} dynamics step returns non-finite values")
            if model.is_linear:
                state_matrix, control_matrix = model.continuous_matrices()
                if state_matrix.shape != (model.state_dim, model.state_dim):
                    fail(errors, f"{prefix} linear state matrix has the wrong shape")
                if control_matrix.shape != (model.state_dim, model.control_dim):
                    fail(errors, f"{prefix} linear control matrix has the wrong shape")
        except Exception as exc:
            fail(errors, f"{prefix} dynamics smoke failed: {exc}")


def check_pipeline_contracts(errors: list[str]) -> None:
    import spark_pipeline
    import spark_robot

    abstract_configs = {
        spark_pipeline.BasePipelineConfig,
        spark_pipeline.BenchmarkPipelineConfig,
        spark_pipeline.TeleopPipelineConfig,
    }
    checked = 0
    for name in sorted(dir(spark_pipeline)):
        config_type = getattr(spark_pipeline, name)
        if not (
            inspect.isclass(config_type)
            and issubclass(config_type, spark_pipeline.BasePipelineConfig)
            and config_type not in abstract_configs
        ):
            continue

        prefix = f"pipeline config {name}"
        try:
            config = config_type()
        except Exception as exc:
            fail(errors, f"{prefix} cannot be constructed: {exc}")
            continue

        robot_name = config.robot.cfg.class_name
        agent_name = config.env.agent.class_name
        if not robot_name:
            fail(errors, f"{prefix} has no default robot config")
            continue
        if not agent_name:
            fail(errors, f"{prefix} has no default environment agent")
            continue
        if not hasattr(spark_robot, robot_name):
            fail(errors, f"{prefix} references unknown robot {robot_name!r}")
            continue

        robot_config = getattr(spark_robot, robot_name)()
        if agent_name not in set(robot_config.agent_class_names.values()):
            fail(
                errors,
                f"{prefix} agent {agent_name!r} is not declared by {robot_name}",
            )
        checked += 1

    if checked == 0:
        fail(errors, "no concrete pipeline configurations were checked")


def check_robot_matrix_report(errors: list[str]) -> None:
    import spark_robot
    from spark_robot import RobotConfig

    expected_names = set()
    seen_types = set()
    for name in sorted(dir(spark_robot)):
        config_type = getattr(spark_robot, name)
        if (
            not inspect.isclass(config_type)
            or config_type is RobotConfig
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
            or config_type in seen_types
        ):
            continue
        # The declarative support matrix covers the 35 AgiBot, Galaxea,
        # Kinova, KUKA, and FANUC configurations. Unitree's specialized WBT,
        # Sport, SONIC, hand, and real-agent workflows have separate gates and
        # are represented by the visual matrix below.
        if name.startswith("UnitreeG1"):
            continue
        seen_types.add(config_type)
        try:
            config = config_type()
        except TypeError:
            continue
        isaac_capability = config.backend_capabilities().get("isaac")
        if (
            config.isaac_articulation is not None
            and isaac_capability is not None
            and isaac_capability.batched
        ):
            expected_names.add(name)

    matrix_specs = (
        ("robot_support_matrix_mujoco.json", {"mujoco"}, 8),
        ("robot_support_matrix_isaac.json", {"isaac"}, 8),
        ("robot_support_matrix.json", {"mujoco", "isaac"}, 16),
    )
    for filename, expected_backends, cases_per_config in matrix_specs:
        report_path = ROOT / "docs/reference" / filename
        if not report_path.is_file():
            fail(errors, f"missing release matrix: {report_path.relative_to(ROOT)}")
            continue
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            fail(errors, f"invalid release matrix {filename}: {exc}")
            continue

        results = report.get("results", [])
        reported_names = {result.get("robot_config") for result in results}
        reported_backends = {result.get("backend") for result in results}
        expected_case_count = len(expected_names) * cases_per_config
        if reported_names != expected_names:
            missing = sorted(expected_names - reported_names)
            extra = sorted(reported_names - expected_names)
            fail(
                errors,
                f"release matrix {filename} inventory is stale: missing={missing}, extra={extra}",
            )
        if reported_backends != expected_backends:
            fail(
                errors,
                f"release matrix {filename} has backends {sorted(reported_backends)}; "
                f"expected {sorted(expected_backends)}",
            )
        if report.get("case_count") != expected_case_count:
            fail(
                errors,
                f"release matrix {filename} has {report.get('case_count')} cases; "
                f"expected {expected_case_count}",
            )
        if report.get("pass_count") != expected_case_count or report.get("failure_count") != 0:
            fail(
                errors,
                f"release matrix {filename} is not fully passing: "
                f"pass={report.get('pass_count')}, failure={report.get('failure_count')}",
            )


def check_demo_gif_report(errors: list[str]) -> None:
    media_root = ROOT / "docs/media/demos/robot_matrix"
    report_path = media_root / "manifest.json"
    if not report_path.is_file():
        fail(errors, "missing robot demonstration GIF release report")
        return
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        fail(errors, f"invalid robot demonstration GIF release report: {exc}")
        return

    expected_cases = {
        ("agibot_g1__mobile_base", "mujoco_teleop"),
        ("fanuc_lrmate200id__single_arm", "mujoco_teleop"),
        ("galaxea_r1lite__mobile_base", "mujoco_teleop"),
        ("kinova_gen3__single_arm", "mujoco_teleop"),
        ("kuka_iiwa14__dual_arm", "mujoco_teleop"),
        ("unitree_g1__whole_body_sonic", "isaac_parallel"),
        ("unitree_g1__whole_body_sport", "mujoco_teleop"),
        ("unitree_g1__whole_body_wbt", "mujoco_teleop"),
    }
    expected_filenames = {f"{profile}__{mode}.gif" for profile, mode in expected_cases}
    release_paths = {path.relative_to(ROOT) for path in media_root.glob("*.gif")}
    expected_release_paths = {
        Path("docs/media/demos/robot_matrix") / filename for filename in expected_filenames
    }
    tracked_gifs = {
        path.relative_to(ROOT) for path in tracked_files() if path.suffix.lower() == ".gif"
    }
    if release_paths != expected_release_paths or tracked_gifs != expected_release_paths:
        fail(
            errors,
            "release must contain only the eight GIFs embedded in the top-level README",
        )

    readme_text = (ROOT / "README.md").read_text(encoding="utf-8")
    readme_gifs = set(re.findall(r"docs/media/demos/robot_matrix/([a-z0-9_]+\.gif)", readme_text))
    if readme_gifs != expected_filenames:
        fail(errors, "top-level README demonstration inventory is stale")

    results = report.get("results", [])
    reported_cases = {(result.get("profile"), result.get("mode")) for result in results}
    if (
        report.get("schema_version") != 1
        or report.get("expected_case_count") != len(expected_cases)
        or report.get("case_count") != len(expected_cases)
        or report.get("pass_count") != len(expected_cases)
        or len(results) != len(expected_cases)
    ):
        fail(
            errors,
            "robot demonstration GIF report must contain exactly eight passing recordings",
        )
    if reported_cases != expected_cases:
        missing = sorted(expected_cases - reported_cases)
        extra = sorted(reported_cases - expected_cases)
        fail(
            errors, f"robot demonstration GIF inventory is stale: missing={missing}, extra={extra}"
        )
    for result in results:
        profile = result.get("profile")
        mode = result.get("mode")
        if "command" in result:
            fail(errors, f"robot demonstration exposes its generation command: {profile}/{mode}")
        filename = f"{profile}__{mode}.gif"
        path = media_root / filename
        if not path.is_file() or path.stat().st_size <= 0:
            fail(errors, f"missing or empty robot demonstration GIF: {path.relative_to(ROOT)}")
        if result.get("status") != "PASS":
            fail(errors, f"robot demonstration did not pass: {profile}/{mode}")
        if result.get("resolution") != [960, 540] or result.get("fps") != 10.0:
            fail(errors, f"robot demonstration has unexpected encoding metadata: {profile}/{mode}")
        if result.get("frame_count", 0) < 144 or result.get("gif_duration_seconds", 0.0) < 14.4:
            fail(errors, f"robot demonstration is too short: {profile}/{mode}")
        if result.get("real_time_playback") is not True:
            fail(errors, f"robot demonstration is not marked real-time: {profile}/{mode}")


def main() -> int:
    errors: list[str] = []
    check_hygiene(errors)
    check_local_path_leaks(errors)
    check_general_toolbox_boundary(errors)
    check_versions(errors)
    check_optional_import_boundary(errors)
    check_robot_contracts(errors)
    check_pipeline_contracts(errors)
    check_robot_matrix_report(errors)
    check_demo_gif_report(errors)
    if errors:
        print("SPARK release check failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("SPARK release check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
