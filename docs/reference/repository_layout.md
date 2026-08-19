# Repository layout

```text
spark/
├── module/
│   ├── spark_agent/       # Simulation and real-robot execution backends
│   ├── spark_policy/      # Control, safety, and composed policies
│   ├── spark_robot/       # Robot models, resources, and configurations
│   ├── spark_task/        # Task definitions and goals
│   └── spark_utils/       # Shared numerical and geometry utilities
├── pipeline/              # Runnable component compositions
├── example/               # User-facing example entry points
├── tests/                 # Automated verification
├── tools/                 # Maintenance and generation utilities
├── docs/                  # Documentation source
├── docker/                # Container entrypoint support
├── Dockerfile             # Shared profile-aware container build
├── docker-compose.yml     # Core, MuJoCo, Isaac, ROS, and WSL services
├── install.sh             # Host/container dependency profiles
└── THIRD_PARTY_NOTICES.md # Asset licensing and provenance sign-off
```

Keep user-facing tutorials in `docs/`, executable commands in `example/`, and
implementation details in their owning module. Documentation should link to
real examples rather than maintaining a second copy of long scripts.
