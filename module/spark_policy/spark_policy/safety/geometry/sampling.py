"""Primitive surface discretization for collision-backend validation."""

from __future__ import annotations

import math


def fibonacci_sphere(count: int):
    """Return approximately uniform unit-sphere samples as a NumPy array."""
    import numpy as np

    if count < 4:
        raise ValueError("sphere point count must be at least 4")
    index = np.arange(count, dtype=np.float64)
    z = 1.0 - 2.0 * (index + 0.5) / count
    radius = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    angle = index * math.pi * (3.0 - math.sqrt(5.0))
    return np.stack((radius * np.cos(angle), radius * np.sin(angle), z), axis=1)


def uv_sphere_mesh(latitude_segments: int = 8, longitude_segments: int = 12):
    """Return an outward-oriented closed unit-sphere triangle mesh."""
    import numpy as np

    if latitude_segments < 3 or longitude_segments < 3:
        raise ValueError("sphere mesh needs at least 3 latitude/longitude segments")
    vertices = [[0.0, 0.0, 1.0]]
    for latitude in range(1, latitude_segments):
        polar = math.pi * latitude / latitude_segments
        for longitude in range(longitude_segments):
            azimuth = 2.0 * math.pi * longitude / longitude_segments
            vertices.append(
                [
                    math.sin(polar) * math.cos(azimuth),
                    math.sin(polar) * math.sin(azimuth),
                    math.cos(polar),
                ]
            )
    bottom = len(vertices)
    vertices.append([0.0, 0.0, -1.0])

    faces = []
    first_ring = 1
    for longitude in range(longitude_segments):
        nxt = (longitude + 1) % longitude_segments
        faces.append([0, first_ring + longitude, first_ring + nxt])
    for latitude in range(latitude_segments - 2):
        first = 1 + latitude * longitude_segments
        second = first + longitude_segments
        for longitude in range(longitude_segments):
            nxt = (longitude + 1) % longitude_segments
            faces.append([first + longitude, second + longitude, second + nxt])
            faces.append([first + longitude, second + nxt, first + nxt])
    last_ring = 1 + (latitude_segments - 2) * longitude_segments
    for longitude in range(longitude_segments):
        nxt = (longitude + 1) % longitude_segments
        faces.append([last_ring + longitude, bottom, last_ring + nxt])
    return np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.int64)


def load_triangle_mesh(path, *, scale: float = 1.0):
    """Load an STL/OBJ mesh as collision-ready vertices and triangles."""
    import numpy as np
    import trimesh

    loaded = trimesh.load(str(path), force="mesh", process=True)
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"{path!s} did not contain one triangle mesh")
    if loaded.faces.shape[1] != 3:
        loaded = loaded.triangulate()
    vertices = np.asarray(loaded.vertices, dtype=np.float32) * float(scale)
    faces = np.asarray(loaded.faces, dtype=np.int64)
    if not len(vertices) or not len(faces):
        raise ValueError(f"{path!s} contains no collision triangles")
    return vertices, faces
