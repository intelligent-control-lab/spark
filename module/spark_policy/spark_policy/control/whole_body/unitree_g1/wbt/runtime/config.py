import numpy as np
import yaml
import os
from pathlib import Path


WBT_SPORT_MODE_ROOT = os.path.dirname(os.path.realpath(__file__))
WBT_SPORT_MODE_CONFIG_DIR = os.path.join(WBT_SPORT_MODE_ROOT, "configs")


def resolve_wbt_path(path, base_dir=None):
    if path is None or os.path.isabs(path):
        return path

    candidate = Path(path).expanduser()
    if base_dir is not None:
        base_candidate = (Path(base_dir) / candidate).resolve()
        if base_candidate.exists():
            return str(base_candidate)

    package_root = Path(WBT_SPORT_MODE_ROOT).resolve()
    package_candidate = (package_root / candidate).resolve()
    if package_candidate.exists():
        return str(package_candidate)

    repo_root = package_root.parents[5]
    repo_candidate = (repo_root / candidate).resolve()
    if repo_candidate.exists():
        return str(repo_candidate)

    return str(candidate)


class Config:

    def __init__(self, file_path) -> None:
        file_path = resolve_wbt_path(file_path)
        with open(file_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

            for key, value in config.items():
                setattr(self, key, value)
            self.simulation_dt = self.control_dt / self.control_decimation
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            # self.init_angles = np.array(config["init_angles"], dtype=np.float32)

            self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32) if "cmd_scale" in config else None

            self.max_cmd = np.array(config["max_cmd"], dtype=np.float32) if "max_cmd" in config else None
            self.cmd_debug = np.array(config["cmd_debug"], dtype=np.float32) if "cmd_debug" in config else None
            self.cmd_clip = np.array(config["cmd_clip"], dtype=np.float32) if "cmd_clip" in config else None
            if hasattr(self, "policy_path"):
                self.policy_path = resolve_wbt_path(self.policy_path, base_dir=os.path.dirname(file_path))
            # gait information
            if "gait_parameters" in config:
                self.gait_parameters = {}
                for key in config["gait_parameters"]:
                    self.gait_parameters[key] = config["gait_parameters"][key]
            else:
                self.gait_parameters = None
