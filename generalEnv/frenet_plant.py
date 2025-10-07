
import numpy as np
from dataclasses import dataclass

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
    
    k_n: float = 0.8
    k_b: float = 1.8
    n_ref: float = 0.0
    b_ref: float = 0.0
    r_max: float = 4.0     
    wall_margin: float = 0.25  
    wall_gain: float = 2.0     

    def __post_init__(self):
        self.reset(n0=0.0, b0=0.0, c0=0.0)

    def reset(self, n0=0.0, b0=0.0, c0=0.0):
        self.x = np.array([0.0, n0, b0, c0], dtype=float)  
        self.t = 0.0

    def r_of_t(self, t):
        
        _, n, b, _ = self.x
        r_pd = (self.k_b * b + self.k_n * n) * (self.V**2)

        
        left  = -self.W/2 + self.wall_margin
        right = +self.W/2 - self.wall_margin
        r_wall = 0.0
        if n < left:
            
            r_wall += self.wall_gain * (left - n) * (self.V**2)
        elif n > right:
            
            r_wall -= self.wall_gain * (n - right) * (self.V**2)

        r = np.clip(r_pd + r_wall, -self.r_max*(self.V**2), self.r_max*(self.V**2))
        return float(r)


    def f(self, t, x):
        s, n, b, c = x
        V = self.V
        r = self.r_of_t(t)
        ds = V
        dn = -b*V
        db = -c*V
        dc = -r/(V**2 + 1e-12)
        return np.array([ds, dn, db, dc], dtype=float)

    def step(self):
        self.x = rk4_step(self.f, self.x, self.t, self.dt)
        self.t += self.dt
        return self.x

    def pose_for_render(self):
        s, n, b, c = self.x
        yaw = b
        delta = np.arctan(self.L * c)
        return [s, n, yaw, self.V], {"delta": float(delta), "n": float(n), "b": float(b), "c": float(c)}
