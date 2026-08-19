import argparse
import os
from pathlib import Path

import coacd
import numpy as np
import trimesh


def _spark_robot_resource_dir() -> Path:
    env_path = os.environ.get("SPARK_ROBOT_RESOURCE_DIR")
    if env_path:
        return Path(env_path).expanduser().resolve()

    for parent in Path(__file__).resolve().parents:
        for candidate in (
            parent / "module" / "spark_robot" / "resources",
            parent / "spark_robot" / "resources",
        ):
            if candidate.exists():
                return candidate

    raise FileNotFoundError(
        "Could not locate spark_robot resources. Set SPARK_ROBOT_RESOURCE_DIR "
        "to the SPARK robot resources directory."
    )


def _default_mesh_path() -> Path:
    return _spark_robot_resource_dir() / "objects" / "picking_box.STL"


def _to_trimesh(part):
    if hasattr(part, "vertices") and hasattr(part, "faces"):
        vertices, faces = part.vertices, part.faces
    elif isinstance(part, (list, tuple)) and len(part) >= 2:
        vertices, faces = part[0], part[1]
    elif isinstance(part, dict) and "vertices" in part and "faces" in part:
        vertices, faces = part["vertices"], part["faces"]
    else:
        raise TypeError(f"Unexpected part type: {type(part)}")
    return trimesh.Trimesh(vertices, faces, process=False)


def run_convex_decomposition(
    mesh_path: Path, output_dir: Path, threshold: float = 0.05, scale=None
) -> int:
    mesh = trimesh.load(str(mesh_path), force="mesh")
    if scale is not None:
        mesh.apply_scale(scale)

    output_dir.mkdir(parents=True, exist_ok=True)
    coacd_mesh = coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces))
    parts = coacd.run_coacd(coacd_mesh, threshold=threshold)

    for index, part in enumerate(parts):
        decomposed_mesh = _to_trimesh(part)
        decomposed_mesh.export(output_dir / f"part_{index:03d}.stl")
    return len(parts)


def _parse_args():
    parser = argparse.ArgumentParser(description="Run COACD convex decomposition for a mesh.")
    parser.add_argument(
        "--mesh",
        type=Path,
        default=None,
        help="Input mesh path. Defaults to spark_robot/resources/objects/picking_box.STL.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("out/hulls"),
        help="Directory for decomposed STL parts. Default: out/hulls.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.05,
        help="COACD decomposition threshold. Default: 0.05.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="Optional uniform scale applied before decomposition.",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    mesh_path = args.mesh.expanduser().resolve() if args.mesh is not None else _default_mesh_path()
    output_dir = args.output_dir.expanduser()
    part_count = run_convex_decomposition(
        mesh_path, output_dir, threshold=args.threshold, scale=args.scale
    )
    print(f"Exported {part_count} convex parts to {output_dir}")


if __name__ == "__main__":
    main()
