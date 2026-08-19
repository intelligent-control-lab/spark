"""Keep robot-owned workflows free of references to other robot families."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

ROBOT_TOKENS = {
    "agibot_g1": ("agibot",),
    "fanuc_lrmate200id": ("fanuc", "lrmate"),
    "galaxea_r1lite": ("galaxea", "r1lite"),
    "kinova_gen3": ("kinova", "gen3"),
    "kuka_iiwa14": ("kuka", "iiwa"),
    "unitree_g1": ("unitree",),
}

TEXT_SUFFIXES = {
    ".cfg",
    ".md",
    ".py",
    ".srdf",
    ".txt",
    ".urdf",
    ".xml",
    ".yaml",
    ".yml",
}


def _workflow_paths(robot: str) -> tuple[Path, ...]:
    paths = [
        REPOSITORY_ROOT / "example" / robot,
        REPOSITORY_ROOT / "module" / "spark_robot" / "spark_robot" / robot,
        REPOSITORY_ROOT / "module" / "spark_robot" / "resources" / robot,
        REPOSITORY_ROOT / "module" / "spark_agent" / "spark_agent" / "simulation" / "isaac" / robot,
        REPOSITORY_ROOT
        / "module"
        / "spark_agent"
        / "spark_agent"
        / "simulation"
        / "mujoco"
        / robot,
        REPOSITORY_ROOT / "docs" / "modules" / "robot" / f"{robot}.md",
    ]
    paths.extend(
        path for path in (REPOSITORY_ROOT / "pipeline").rglob(f"*{robot}*") if path.is_file()
    )
    return tuple(paths)


def _text_files(path: Path):
    if path.is_file():
        if path.suffix.lower() in TEXT_SUFFIXES:
            yield path
        return
    if path.is_dir():
        for child in path.rglob("*"):
            if child.is_file() and child.suffix.lower() in TEXT_SUFFIXES:
                yield child


def test_robot_workflows_do_not_name_other_robot_families():
    violations = []
    for robot, own_tokens in ROBOT_TOKENS.items():
        alien_tokens = {
            token
            for other_robot, tokens in ROBOT_TOKENS.items()
            if other_robot != robot
            for token in tokens
        }
        for workflow_path in _workflow_paths(robot):
            for path in _text_files(workflow_path):
                text = path.read_text(encoding="utf-8", errors="ignore").lower()
                matched = sorted(token for token in alien_tokens if token in text)
                if matched:
                    violations.append(
                        f"{path.relative_to(REPOSITORY_ROOT)} references {', '.join(matched)}"
                    )

    assert not violations, "Robot workflow isolation violations:\n" + "\n".join(violations)
