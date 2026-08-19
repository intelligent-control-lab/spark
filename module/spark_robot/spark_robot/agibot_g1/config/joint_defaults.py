"""Shared nominal joint posture for the AgiBot G1 simulation models."""

_AGIBOT_G1_LEFT_DEFAULTS = [
    -0.01408703,
    1.49600114,
    0.13254883,
    -0.12675357,
    0.02853095,
    0.07266831,
    0.0,
]

_AGIBOT_G1_RIGHT_DEFAULTS = [
    0.01410166,
    -1.49600689,
    -0.13251528,
    0.12672038,
    -0.02851918,
    -0.07263766,
    0.0,
]

AGIBOT_G1_LOCKED_JOINT_DEFAULTS = {
    "joint_lift_body": 0.25,
    "joint_body_pitch": 0.05,
    **{
        name: value
        for name, value in zip(
            ("Joint1_l", "Joint2_l", "Joint3_l", "Joint4_l", "Joint5_l", "Joint6_l", "Joint7_l"),
            _AGIBOT_G1_LEFT_DEFAULTS,
        )
    },
    **{
        name: value
        for name, value in zip(
            ("Joint1_r", "Joint2_r", "Joint3_r", "Joint4_r", "Joint5_r", "Joint6_r", "Joint7_r"),
            _AGIBOT_G1_RIGHT_DEFAULTS,
        )
    },
}
