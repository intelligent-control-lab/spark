# Safety module

The safety package contains two distinct concepts: **monitoring**, which
computes safety-index values and derivatives, and **filtering**, which may
modify a nominal action. They are policy components, not a universal layer in
the SPARK architecture.

## Collision safety index

For signed separation distance `d(x)` and configured minimum distance
`d_min`, collision monitors use a convention equivalent to

```{math}
\phi(x)=d_{min}-d(x),
```

so positive values indicate violation of the desired separation. Environment
and self-collision pairs have separate thresholds and masks. Adjacent links
and configured ignored volumes are removed from the constraint set.

First-order monitors use distance gradients and kinematic Jacobians.
Second-order monitors additionally use relative velocity, Jacobian-rate, and
curvature terms. Approximate variants reduce computational cost; the learned
monitor exposes the same safety-index contract.

## Distance implementations

### Analytic geometry

The NumPy path represents robot links and obstacles as framed primitives. It
computes pairwise signed displacement, distance, witness normal, relative
velocity, and curvature for supported primitive combinations. Robot volumes
are transformed to world coordinates using forward kinematics; link Jacobians
map joint velocity to collision-volume velocity.

### Point clouds

A point cloud is stored as point-spheres with positions, radii, validity masks,
velocities, and object IDs. For robot sphere center `c_r`, radius `r_r`, point
`p_j`, and point radius `r_j`, the separation is

```{math}
d_{rj}=\lVert c_r-p_j\rVert_2-r_r-r_j.
```

The nearest witness supplies the normal and gradient direction. Dynamic point
counts use a fixed-capacity tensor plus a validity mask; overflow is reported
instead of silently dropping points. The CPU monitor can use a `cKDTree` to
retain the nearest configured number of environment points per link.

Depth images are converted by pinhole unprojection,

```{math}
X=(u-c_x)Z/f_x,\qquad Y=(v-c_y)Z/f_y,
```

then rotated and translated from optical coordinates into world coordinates.
Depth range, pixel stride, and capacity control the resulting cloud.

### Triangle meshes

`TorchMeshCollisionBackend` evaluates sphere-to-oriented-triangle distance.
For every triangle it considers closest points on the three edges and the
orthogonal plane projection when its barycentric coordinates lie inside the
triangle. The minimum candidate is the environment witness. Consistent outward
face orientation determines the sign, and the sphere radius is subtracted:

```{math}
d=\operatorname{sign}(c-p^*)\lVert c-p^*\rVert_2-r.
```

The current implementation performs an exhaustive triangle reduction and is
intended for moderate meshes.

### Dense ESDF

The dense Euclidean signed-distance-field backend samples the grid with
trilinear interpolation and estimates the gradient using centered finite
differences. The normalized gradient is the contact normal; subtracting the
robot sphere radius produces surface-to-surface separation.

## Safety filters

| Filter family | Behavior |
|---|---|
| Bypass | Returns the nominal action unchanged; useful as a control case. |
| Potential field | Adds a repulsive command near obstacles, following the artificial-potential-field idea. |
| Sliding mode | Uses the safety-index sign/surface to create a corrective command. |
| Safe set | Enforces the configured safety-index derivative condition. |
| Sublevel set | Keeps a value/safety function within a selected sublevel set. |
| CBF | Enforces a control-barrier-function inequality. |
| Relaxed variants | Add slack so the optimization remains solvable and report the violation tradeoff. |

For control-affine dynamics, a typical CBF constraint is

```{math}
L_fh(x)+L_gh(x)u+\alpha(h(x))\ge 0,
```

and a least-change filter solves

```{math}
u^*=\arg\min_u\tfrac12\lVert u-u_{nom}\rVert_W^2
\quad\text{subject to the active safety constraints and control limits.}
```

CBF quadratic programs and their invariance interpretation are described by
[Ames et al.](https://arxiv.org/abs/1609.06408). Potential-field obstacle
avoidance follows the approach introduced by
[Khatib](https://journals.sagepub.com/doi/pdf/10.1177/027836498600500106).

## Choosing a representation

- Use primitives when the environment is known and can be approximated with a
  small number of spheres or other supported volumes.
- Use point clouds for dynamic perception and depth-camera data; configure
  radius, capacity, and nearest-point reduction deliberately.
- Use meshes for known moderate-complexity object surfaces with consistently
  oriented triangles.
- Use an ESDF when a dense signed field is already available and fast repeated
  queries are more important than retaining individual surface points.
