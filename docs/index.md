```{raw} html
<div class="spark-hero">
  <p class="spark-kicker">Safe and modular robot autonomy</p>
  <h1>Build robot behaviors with SPARK</h1>
  <p>
    SPARK composes robot models, tasks, control policies, safety policies, and
    simulation or deployment backends into reusable robotics pipelines.
  </p>
</div>
```

# SPARK documentation

SPARK is a research framework for developing and evaluating safe robot control
across multiple embodiments. The same task and policy structure can be used
with MuJoCo, Isaac, and supported real-robot backends.

::::{grid} 1 2 2 4
:gutter: 3

:::{grid-item-card} Get started
:link: getting_started/installation
:link-type: doc

Install a simulation profile and run your first SPARK example.
:::

:::{grid-item-card} Learn the architecture
:link: architecture/overview
:link-type: doc

Understand how pipelines connect tasks, agents, robots, and policies.
:::

:::{grid-item-card} Run a tutorial
:link: tutorials/teleoperation
:link-type: doc

Start with the Unitree G1 simulation and teleoperation workflows.
:::

:::{grid-item-card} See SPARK in action
:link: tutorials/showcase
:link-type: doc

Explore animated examples of supported robots, safety, and benchmarking.
:::

::::

```{warning}
SPARK can control physical robots. Validate configurations in simulation, keep
an emergency stop available, and follow the hardware manufacturer's safety
procedures before real-world deployment.
```

```{toctree}
:hidden:
:maxdepth: 3

getting_started/index
tutorials/index
architecture/index
modules/index
best_practices/index
reference/index
```
