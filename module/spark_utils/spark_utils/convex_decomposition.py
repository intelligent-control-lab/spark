import os, numpy as np, trimesh, coacd

os.makedirs("out/hulls", exist_ok=True)

mesh = trimesh.load("/home/yifan/SPACE/spark_dev/module/spark_robot/resources/objects/picking_box.STL", force="mesh")
# If your STL is in millimeters, uncomment:
# mesh.apply_scale(0.001)

cm = coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces))
parts = coacd.run_coacd(cm, threshold=0.05)   # tune threshold as needed

def to_trimesh(part):
    # coacd.Mesh-like object
    if hasattr(part, "vertices") and hasattr(part, "faces"):
        V, F = part.vertices, part.faces
    # (vertices, faces) tuple/list
    elif isinstance(part, (list, tuple)) and len(part) >= 2:
        V, F = part[0], part[1]
    # dict form (rare)
    elif isinstance(part, dict) and "vertices" in part and "faces" in part:
        V, F = part["vertices"], part["faces"]
    else:
        raise TypeError(f"Unexpected part type: {type(part)}")
    return trimesh.Trimesh(V, F, process=False)

for i, p in enumerate(parts):
    tm = to_trimesh(p)
    tm.export(f"out/hulls/part_{i:03d}.stl")   # or .obj if you prefer
