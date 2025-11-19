import numpy as np
from dataclasses import dataclass

def rk4_step(f, x, t, dt):
    k1 = f(t, x)
    k2 = f(t + 0.5*dt, x + 0.5*dt*k1)
    k3 = f(t + 0.5*dt, x + 0.5*dt*k2)
    k4 = f(t + dt,     x + dt*k3)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

@dataclass
class LongitudinalPlant:
    dt: float = 0.01

    s0: float = 0.0
    v0: float = 0.0
    a0: float = 0.0

    # reference velocity
    v_ref: float = 15.0 / 3.6  # 15 km/h ~ 4.17 m/s

    # fake gains for jerk controller: j = k_v*(v_ref - v) - k_a*a
    k_v: float = 2.0
    k_a: float = 0.8
    j_clip: float = 2.0    # jerk max [m/s^3]

    def __post_init__(self):
        self.reset(self.s0, self.v0, self.a0)

    def reset(self, s0=0.0, v0=0.0, a0=0.0):
        self.x = np.array([s0, v0, a0], dtype=float)  # [s, v, a]
        self.t = 0.0

    def j_of_t(self):
        s, v, a = self.x
        e_v = self.v_ref - v
        j = self.k_v * e_v - self.k_a * a
        return float(np.clip(j, -self.j_clip, +self.j_clip))

    def f(self, t, x):
        s, v, a = x
        j = self.j_of_t()
        ds = v
        dv = a
        da = j
        return np.array([ds, dv, da], dtype=float)

    def step(self):
        self.x = rk4_step(self.f, self.x, self.t, self.dt)
        self.t += self.dt
        return self.x

    def pose_for_render(self):
        s, v, a = self.x
        n = 0.0
        yaw = 0.0
        V = v
        # diagnostic info for HUD
        return [s, n, yaw, V], {
            "v": float(v),
            "a": float(a),
            "j": float(self.j_of_t()),
        }
