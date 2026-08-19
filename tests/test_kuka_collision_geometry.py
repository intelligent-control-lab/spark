"""KUKA collision-sphere topology regression tests."""

import numpy as np

import spark_robot


def test_kuka_valid_home_pose_has_no_unmasked_sphere_overlap():
    config = spark_robot.KukaIIWA14SingleArmDynamic1CollisionConfig()
    kinematics = spark_robot.KukaIIWA14SingleArmKinematics(config)
    position = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs])
    frames = kinematics.forward_kinematics(position)
    ignored = {frozenset((first, second)) for first, second in config.AdjacentCollisionVolPairs}
    minimum_clearance = np.inf
    active_pairs = 0
    volumes = list(config.CollisionVol)
    for index, first in enumerate(volumes):
        for second in volumes[index + 1 :]:
            if frozenset((first, second)) in ignored:
                continue
            active_pairs += 1
            radius_first = config.CollisionVol[first].attributes["radius"]
            radius_second = config.CollisionVol[second].attributes["radius"]
            clearance = (
                np.linalg.norm(frames[first][:3, 3] - frames[second][:3, 3])
                - radius_first
                - radius_second
            )
            minimum_clearance = min(minimum_clearance, clearance)

    assert active_pairs > 0
    assert minimum_clearance >= 0.0
