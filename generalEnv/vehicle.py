from dataclasses import dataclass
import numpy as np

from vehicle_model.vehicle_double_track_model import vehicle_double_track_model
from vehicle_model.vehicle_params import VehicleParams
from vehicle_model.pacejka_params import PacejkaParam

def rk4_step(f, X, u, dt):
    k1, _ = f(X, u)
    k2, _ = f(X + 0.5*dt*k1, u)
    k3, _ = f(X + 0.5*dt*k2, u)
    k4, _ = f(X + dt*k3, u)
    return X + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)


@dataclass
class DoubleTrackVehicle:
    params: VehicleParams
    tire: PacejkaParam
    dt: float = 0.02  # time step

    def __post_init__(self):
        self.X=self._initial_state(self.params)
        self.dX_prev=np.zeros_like(self.X)

    @staticmethod
    def _initial_state(params: VehicleParams):
        X=np.zeros(24,dtype=np.float64)
        X[0]=0.0      # x [m]
        X[1]=0.0      # y [m]
        X[2]=0.0      # psi 
        X[3]=0.0      # u
        X[4]=0.0      # v
        X[5]=0.0      # omega

        m=params.vehicle.m
        g=9.81
        Lf=params.vehicle.Lf
        Lr=params.vehicle.Lr
        Wf=params.vehicle.Wf
        Wr=params.vehicle.Wr
        Fz_front=m*g*(Lr/(Lf+Lr))
        Fz_rear=m*g*(Lf/(Lf+Lr))
        X[6]=Fz_front/2  # Fz_fl
        X[7]=Fz_front/2  # Fz_fr
        X[8]=Fz_rear/2   # Fz_rl
        X[9]=Fz_rear/2   # Fz_rr

        #should be also steer and wheels & slip & brake
        return X
    

    def reset(self):
        self.__post_init__()

    def dynamics(self,X,u):
        pedal_req, delta_req = u
        dX, extra = vehicle_double_track_model(
            self.dX_prev, X, pedal_req, delta_req, self.tire, self.params
        )
        return dX, extra
    
    def step(self, pedal_req: float, delta_req: float):
        pedal_req = float(np.clip(pedal_req, -1.0, 1.0))
        
        delta_req = float(np.clip(delta_req, -0.7, 0.7))

        X_next = rk4_step(lambda X,u: self.dynamics(X,u), self.X, (pedal_req, delta_req), self.dt)
        dX, _ = self.dynamics(X_next, (pedal_req, delta_req))

        self.X = X_next
        self.dX_prev = dX
        return self.X