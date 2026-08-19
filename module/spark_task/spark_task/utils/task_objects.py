import numpy as np


class TaskObject3D:
    def __init__(self, **kwargs):
        self.frame = kwargs.get("frame", np.eye(4))
        self.last_frame = self.frame.copy()
        self.frame_callback = self.frame.copy()
        self.velocity = kwargs.get("velocity", 0.01)
        self.bound = kwargs.get("bound", np.zeros((3, 2)))
        self.smooth_weight = kwargs.get("smooth_weight", 1.0)
        self.direction = kwargs.get("direction", np.array([0.0, 0.0, 0.0]))
        # This stores the previous displacement, not the configured unit
        # direction. Initializing it from ``direction`` made a zero-velocity
        # Brownian goal jump on its first update.
        self.last_direction = np.zeros(3, dtype=float)
        self.step_counter = 0
        self.keep_direction_step = kwargs.get("keep_direction_step", 1)
        self.dt = kwargs.get("dt", 0.01)
        self._seed = kwargs.get("_seed", 0)
        self.rs = np.random.RandomState(self._seed)

    def move(self, mode):
        if mode == "Brownian":
            if self.step_counter % self.keep_direction_step == 0:
                direction = self.rs.normal(loc=0.0, size=3)
                # ``velocity`` has the same m/s meaning as Velocity mode.
                # Brownian motion previously applied it once per control step
                # without ``dt``, making a 50 Hz task move 50x too quickly.
                norm = np.linalg.norm(direction)
                direction = (
                    self.velocity * self.dt * direction / norm
                    if norm > 0.0
                    else np.zeros(3, dtype=float)
                )
            else:
                direction = self.last_direction
            self.last_frame = self.frame.copy()
            update_step = (
                1 - self.smooth_weight
            ) * self.last_direction + self.smooth_weight * direction
            self.frame[:3, 3] += update_step
            self.last_direction = self.frame[:3, 3] - self.last_frame[:3, 3]
        elif mode == "Velocity":
            update_step = self.velocity * self.direction * self.dt
            self.frame[:3, 3] += update_step
            self.last_direction = update_step
        # Reflect at bounds instead of clamping. Clamping preserves an
        # outward velocity and makes moving objects accumulate and slide on a
        # boundary plane until their next direction resample.
        for dim in range(3):
            lower, upper = self.bound[dim]
            if lower >= upper:
                self.frame[dim, 3] = lower
                self.last_direction[dim] = 0.0
                continue
            if self.frame[dim, 3] < lower:
                self.frame[dim, 3] = lower + (lower - self.frame[dim, 3])
                self.last_direction[dim] = abs(self.last_direction[dim])
                if mode == "Velocity":
                    self.direction[dim] = abs(self.direction[dim])
            elif self.frame[dim, 3] > upper:
                self.frame[dim, 3] = upper - (self.frame[dim, 3] - upper)
                self.last_direction[dim] = -abs(self.last_direction[dim])
                if mode == "Velocity":
                    self.direction[dim] = -abs(self.direction[dim])

        self.step_counter += 1
