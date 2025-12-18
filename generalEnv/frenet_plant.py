import numpy as np
from dataclasses import dataclass

try:
    import gp_lat
    HAS_R = True
except Exception:
    gp_lat = None
    HAS_R = False


@dataclass
class FrenetLateralPlant:
    W: float = 3.0        # lane width
    V: float = 3.5        # longitudinal speed
    L: float = 2.5        # wheelbase (for rendering)
    dt: float = 0.01      # sample time = 0.01 s

    T_max: float = 20.0
    n_ref: float = 1.0    # lateral target

    # only used if gp_lat is not available
    k_n: float = 0.8
    k_b: float = 2.0
    r_clip: float = 10.0

    def __post_init__(self):
        self.reset(n0=0.0, b0=0.00, c0=0.0)

    def reset(self, n0=0.0, b0=0.0, c0=0.0):
        # x = [s, n, b, c]
        self.x = np.array([0.0, n0, b0, c0], dtype=float)
        self.t = 0.0

    def compute_r(self, x):
        _, n, b, c = x
        if HAS_R:
            r_val = gp_lat.r(self.W, self.V,
                             float(n), float(b), float(c),
                             float(self.T_max), float(self.n_ref))
        else:
            # semplice PD in fallback
            r_val = -(self.k_b * b + self.k_n * (n - self.n_ref)) * (self.V**2)

        lim = self.r_clip * (self.V**2)
        return float(np.clip(r_val, -lim, +lim))

    def step(self, r_cmd=None):
        s, n, b, c = self.x

        r = self.compute_r(self.x) if r_cmd is None else float(r_cmd)

        V  = self.V
        ds = V
        dn = b * V
        db = c * V
        dc = r / (V**2 + 1e-12)

        dx = np.array([ds, dn, db, dc], dtype=float)

        self.x = self.x + self.dt * dx
        self.t += self.dt
        return self.x


    def pose_for_render(self):
        s, n, b, c = self.x
        yaw = b
        delta = np.arctan(self.L * c)
        return [s, n, yaw, self.V], {
            "delta": float(delta),
            "n": float(n),
            "b": float(b),
            "c": float(c),
        }
