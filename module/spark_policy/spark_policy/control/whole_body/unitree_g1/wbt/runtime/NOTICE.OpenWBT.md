# OpenWBT attribution

This runtime is adapted from
[GalaxyGeneralRobotics/OpenWBT](https://github.com/GalaxyGeneralRobotics/OpenWBT)
at commit `a0154ca822815c8eca26f5174afa6268f8e3d465` and is distributed
under Apache License 2.0; see `LICENSE.OpenWBT` in this directory.

SPARK relocates the runtime into the `spark_policy` package, changes imports
and configuration/resource lookup, and adapts the controllers to SPARK's
MuJoCo and Isaac interfaces. Modified upstream files carry a SPARK
modification notice. The bundled `loco.onnx` and `squat.onnx` checkpoints are
byte-identical to the files at that upstream revision.
