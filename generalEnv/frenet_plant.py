import numpy as np
from dataclasses import dataclass


try:
    import gp_lat
    HAS_R = True
except Exception:
    gp_lat = None
    HAS_R = False


def rk4_step(f, x, t, dt):
    k1 = f(t, x)
    k2 = f(t + 0.5*dt, x + 0.5*dt*k1)
    k3 = f(t + 0.5*dt, x + 0.5*dt*k2)
    k4 = f(t + dt,     x + dt*k3)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

@dataclass
class FrenetLateralPlant:
    W: float = 3.0
    V: float = 3.5
    L: float = 2.5
    dt: float = 0.01
    plan_every: int =5
    r_cmd:float = 0.0
    dt: float=0.005

    
    T_max: float = 20.0
    n_ref: float = 0.0

    
    k_n: float = 0.8
    k_b: float = 2.0
    r_clip: float = 10.0   

    def __post_init__(self):
        self.reset(n0=0.0, b0=0.3, c0=0.0)  

    def reset(self, n0=0.0, b0=0.0, c0=0.0):
        self.x = np.array([0.0, n0, b0, c0], dtype=float)  
        self.t = 0.0

    def r_of_t(self):
        _, n, b, c = self.x
        # richiama il planner ogni N step
        if int(self.t / self.dt) % self.plan_every == 0 and HAS_R:
            self.r_cmd = gp_lat.r(self.W, self.V, float(n), float(b), float(c),
                                float(self.T_max), float(self.n_ref))
        elif not HAS_R:
            self.r_cmd = -(self.k_b * b + self.k_n * (n - self.n_ref)) * (self.V**2)

        # barriera morbida vicino ai bordi (spinge verso il centro)
        left  = -self.W/2 + 0.15
        right = +self.W/2 - 0.15
        k_wall = 3.0                        # guadagno barriera
        wall = 0.0
        if n < left:
            wall += k_wall * (left - n) * (self.V**2)
        elif n > right:
            wall -= k_wall * (n - right) * (self.V**2)

        # clamp finale
        lim = self.r_clip * (self.V**2)
        return float(np.clip(self.r_cmd + wall, -lim, +lim))


    def f(self, t, x):
        _, n, b, c = x
        V = self.V
        r = self.r_of_t()
        ds = V
        dn = b * V
        db = c * V
        dc = r / (V**2 + 1e-12)
        return np.array([ds, dn, db, dc], dtype=float)

    def step(self):
        self.x = rk4_step(self.f, self.x, self.t, self.dt)
        # soft clamp corsia
        half = self.W * 0.5
        eps  = 1e-3
        if self.x[1] >= half - eps:
            self.x[1] = half - eps
            self.x[2] *= 0.6   # smorza b
            self.x[3] *= 0.6   # smorza c
        elif self.x[1] <= -half + eps:
            self.x[1] = -half + eps
            self.x[2] *= 0.6
            self.x[3] *= 0.6
        self.t += self.dt
        return self.x

    def pose_for_render(self):
        s, n, b, c = self.x
        yaw = b
        delta = np.arctan(self.L * c)   
        return [s, n, yaw, self.V], {"delta": float(delta), "n": float(n), "b": float(b), "c": float(c)}
