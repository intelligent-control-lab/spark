# spark-policy

Reusable planning, control, estimation, and safety policies for the
[SPARK robotics framework](https://github.com/intelligent-control-lab/spark).

Learned-policy dependencies are available through the `learned` package extra
or the repository installer's `--learned` option. The optional
`UnitreeG1NativeBatchedSonicPolicy` runs dynamic-batch SONIC encoder/decoder
exports directly on CUDA when an upstream component supplies a full-body
motion reference.
