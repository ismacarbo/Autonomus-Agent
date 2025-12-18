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

    v_min: float = 0.0        
    v_max: float = 50.0       

    def __post_init__(self):
        self.reset(self.s0, self.v0, self.a0)

    def reset(self, s0=0.0, v0=0.0, a0=0.0):
        self.x = np.array([s0, v0, a0], dtype=float)  
        self.t = 0.0

    def f_cmd(self, t, x, j_cmd):
        s, v, a = x
        ds = v
        dv = a
        da = float(j_cmd)
        return np.array([ds, dv, da], dtype=float)

    def step(self, j_cmd=0.0):
        self.x = rk4_step(lambda t, x: self.f_cmd(t, x, j_cmd), self.x, self.t, self.dt)
        self.t += self.dt

        
        s, v, a = self.x
        if v < self.v_min:
            v = self.v_min
            a = 0.0
        if v > self.v_max:
            v = self.v_max
            a = 0.0
        self.x = np.array([s, v, a], dtype=float)

        return self.x

    def pose_for_render(self, j_used=0.0):
        s, v, a = self.x
        return [s, 0.0, 0.0, v], {"v": float(v), "a": float(a), "j": float(j_used)}
