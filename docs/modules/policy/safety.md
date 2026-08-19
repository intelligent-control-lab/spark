# Safety

The `safety` hierarchy contains monitoring, filtering, geometry, and tensor
implementations. Monitoring computes safety values; filtering may modify a
nominal command. Neither is assumed to be present in every policy.

## Monitoring

Collision monitors implement first-order, approximate first-order,
second-order, approximate second-order, and learned safety indices. With signed
separation `d`, the common convention is `φ=d_min-d`.

## Filtering

Implemented families include bypass, potential field, sliding mode, safe set,
sublevel set, control barrier function, and relaxed variants. A least-change
QP filter has the form

```{math}
u^*=\arg\min_u\tfrac12\lVert u-u_{nom}\rVert_W^2
\quad\text{subject to safety constraints and control limits.}
```

## Geometry

Primitive queries compute analytic witness distances. Point clouds use
point-spheres and optional nearest-point reduction. Depth images are
unprojected with the pinhole model. Mesh queries minimize over triangle-plane
and edge candidates. Dense ESDF queries use trilinear interpolation and finite
differences for normals.

## Tensor

Tensor implementations provide first- and second-order constraints plus
projection, QP, relaxed-QP, and reactive filters for batched execution.
