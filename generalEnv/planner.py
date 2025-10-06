import numpy as np

class QuinticPlanner:
    def __init__(self, T=5.0):
        self.T = float(T)
        self.cx = None
        self.cy = None

    @staticmethod
    def _poly5(p0, v0, a0, p1, v1, a1, T):
        A = np.array([[T**3,   T**4,    T**5],
                      [3*T**2, 4*T**3, 5*T**4],
                      [6*T,   12*T**2, 20*T**3]], dtype=float)
        b = np.array([p1 - (p0 + v0*T + 0.5*a0*T**2),
                      v1 - (v0 + a0*T),
                      a1 - a0], dtype=float)
        c3, c4, c5 = np.linalg.solve(A, b)
        return np.array([p0, v0, 0.5*a0, c3, c4, c5], dtype=float)

    def plan(self, start, goal):
        # start, goal: [x, y, yaw, v]
        x0, y0, yaw0, v0 = start
        x1, y1, yaw1, v1 = goal
        self.cx = self._poly5(x0, v0*np.cos(yaw0), 0.0, x1, v1*np.cos(yaw1), 0.0, self.T)
        self.cy = self._poly5(y0, v0*np.sin(yaw0), 0.0, y1, v1*np.sin(yaw1), 0.0, self.T)

    def pos(self, t):
        t = np.clip(t, 0.0, self.T)
        tv = np.array([1, t, t**2, t**3, t**4, t**5], dtype=float)
        xr = float(self.cx @ tv)
        yr = float(self.cy @ tv)
        return xr, yr
