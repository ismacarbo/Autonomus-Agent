import math
import pygame
from collections import deque
import gp_lat

from frenet_plant import FrenetLateralPlant
from longitudinal_plant import LongitudinalPlant
from vehicle import DoubleTrackVehicle
from vehicle_model.vehicle_params import VehicleParams
from vehicle_model.pacejka_params import PacejkaParam

from renderer import (
    draw_road,
    draw_nref,
    draw_trail,
    draw_minimap,
    draw_vehicle,
    draw_steer_arc_from_c,
    world_to_screen,
    CLR_BG,
)

WIDTH, HEIGHT = 1000, 700
SCALE = 80.0
TRACK_SCALE = 8.0
TRACK_ORIGIN = (120, HEIGHT - 120)


# ------------------------------------------------------------
#  UTIL
# ------------------------------------------------------------
def frenet_camera_origin(s: float):
    center_x_px = WIDTH * 0.35
    ox = center_x_px - s * SCALE
    oy = HEIGHT // 2
    return (ox, oy)


def state_to_poseV(X):
    """Per il double-track: [x, y, psi, u, v, ...] -> [x, y, yaw, v_abs]."""
    x, y, psi, u, v, _ = X[:6]
    v_abs = math.hypot(u, v)
    return [x, y, psi, v_abs]


# ------------------------------------------------------------
#  LATERAL SIMULATOR (FRENET)
# ------------------------------------------------------------
class LateralSim:
    MODE_NAME = "Lateral Frenet controller"

    def __init__(self):
        self.font = pygame.font.SysFont("Ubuntu Mono", 18)
        self.plant = FrenetLateralPlant(W=3.0, V=3.5, L=2.5, dt=0.01)
        self.plant.reset(n0=0.0, b0=0.01, c0=0.0)  # small initial lateral velocity
        self.plant.n_ref = 0.0

        self.paused = False
        self.cam_s = 0.0
        self.trail = deque(maxlen=150)
        self.hist = deque(maxlen=800)

    def handle_event(self, e: pygame.event.Event):
        if e.type != pygame.KEYDOWN:
            return

        if e.key == pygame.K_SPACE:
            self.paused = not self.paused
        elif e.key == pygame.K_r:
            self.plant.reset(n0=0.0, b0=0.01, c0=0.0)
            self.plant.n_ref = 0.0
            self.trail.clear()
            self.hist.clear()
        elif e.key == pygame.K_1:
            self.plant.n_ref = +(self.plant.W / 2 - 0.4)
        elif e.key == pygame.K_2:
            self.plant.n_ref = -(self.plant.W / 2 - 0.4)
        elif e.key == pygame.K_0:
            self.plant.n_ref = 0.0
        elif e.key in (pygame.K_LEFT, pygame.K_a):
            self.plant.n_ref -= 0.1
        elif e.key in (pygame.K_RIGHT, pygame.K_d):
            self.plant.n_ref += 0.1
        elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.plant.T_max = max(1.0, self.plant.T_max - 0.5)
        elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.plant.T_max = min(30.0, self.plant.T_max + 0.5)

        self.plant.n_ref = max(
            -(self.plant.W / 2 - 0.2),
            min(self.plant.W / 2 - 0.2, self.plant.n_ref)
        )

    def update(self):
        if not self.paused:
            self.plant.step()

        s, n, b, c = self.plant.x
        self.cam_s = 0.92 * self.cam_s + 0.08 * s
        self.trail.append((s, n))
        self.hist.append((s, n))

    def draw(self, screen: pygame.Surface):
        s, n, b, c = self.plant.x
        ORIGIN = frenet_camera_origin(self.cam_s)

        screen.fill(CLR_BG)
        draw_road(screen, s_center=self.cam_s, W=self.plant.W, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_nref(screen, s_center=self.cam_s, n_ref=self.plant.n_ref, W=self.plant.W, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_trail(screen, self.trail, SCALE, ORIGIN)

        pose, info = self.plant.pose_for_render()
        draw_steer_arc_from_c(screen, pose[0], pose[1], pose[2], info["c"], SCALE, ORIGIN, arc_len=12.0)
        draw_vehicle(screen, pose, SCALE, ORIGIN)
        draw_minimap(screen, self.hist, self.plant.W, self.plant.V)

        hud1 = f"[LATERAL]  V={self.plant.V:.2f} m/s | n={n:+.2f} m | b={math.degrees(b):+.1f}° | c={c:+.3f} 1/m"
        hud2 = f"n_ref={self.plant.n_ref:+.2f} m | T_max={self.plant.T_max:.1f}s | δ≈{math.degrees(info['delta']):+.1f}°"
        hud3 = "SPACE pause | R reset | 1/2 lane-change | 0 center | LEFT/RIGHT tweak ref | ESC menu"

        screen.blit(self.font.render(hud1, True, (230, 230, 230)), (16, 16))
        screen.blit(self.font.render(hud2, True, (180, 200, 230)), (16, 40))
        screen.blit(self.font.render(hud3, True, (170, 170, 170)), (16, 64))


# ------------------------------------------------------------
#  LONGITUDINAL SIMULATOR
# ------------------------------------------------------------
class LongitudinalSim:
    MODE_NAME = "Longitudinal (fake jerk controller)"

    def __init__(self):
        self.font = pygame.font.SysFont("Ubuntu Mono", 18)
        self.plant = LongitudinalPlant(dt=0.01)
        self.plant.reset(s0=0.0, v0=0.0, a0=0.0)

        self.paused = False
        self.cam_s = 0.0
        self.trail = deque(maxlen=300)
        self.hist = deque(maxlen=800)

    def handle_event(self, e: pygame.event.Event):
        if e.type != pygame.KEYDOWN:
            return

        if e.key == pygame.K_SPACE:
            self.paused = not self.paused
        elif e.key == pygame.K_r:
            self.plant.reset(s0=0.0, v0=0.0, a0=0.0)
            self.trail.clear()
            self.hist.clear()

    def update(self):
        if not self.paused:
            self.plant.step()

        s, v, a = self.plant.x
        self.cam_s = 0.90 * self.cam_s + 0.10 * s
        self.trail.append((s, 0.0))
        self.hist.append((s, 0.0))

    def draw(self, screen: pygame.Surface):
        s, v, a = self.plant.x
        ORIGIN = frenet_camera_origin(self.cam_s)

        screen.fill(CLR_BG)
        draw_road(screen, s_center=self.cam_s, W=3.0, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_trail(screen, self.trail, SCALE, ORIGIN)

        pose, info = self.plant.pose_for_render()
        draw_vehicle(screen, pose, SCALE, ORIGIN)
        draw_minimap(screen, self.hist, W=3.0, V=info["v"])

        hud1 = f"[LONGITUDINAL]  s={s:6.1f} m | v={info['v']*3.6:5.1f} km/h (v_ref={self.plant.v_ref*3.6:5.1f})"
        hud2 = f"a={info['a']:6.3f} m/s² | j={info['j']:6.3f} m/s³"
        hud3 = "UP/DOWN (W/S) change v_ref | SPACE pause | R reset | ESC menu"

        screen.blit(self.font.render(hud1, True, (230, 230, 230)), (16, 16))
        screen.blit(self.font.render(hud2, True, (180, 200, 230)), (16, 40))
        screen.blit(self.font.render(hud3, True, (170, 170, 170)), (16, 64))


# ------------------------------------------------------------
#  DOUBLE-TRACK VEHICLE SIMULATOR
# ------------------------------------------------------------
class VehicleSim:
    MODE_NAME = "Double-track vehicle playground"

    def __init__(self):
        self.font = pygame.font.SysFont("Ubuntu Mono", 18)
        self.veh = DoubleTrackVehicle(params=VehicleParams(), tire=PacejkaParam())
        self.pedal = 0.0
        self.delta_req = 0.0
        self.pedal_rate = 2.5
        self.max_delta = 0.6
        self.steer_rate = 2.5
        self.paused = False
        self.trail = []

    def handle_event(self, e: pygame.event.Event):
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_SPACE:
                self.paused = not self.paused
            elif e.key == pygame.K_r:
                self.veh.reset()
                self.pedal = 0.0
                self.delta_req = 0.0
                self.trail.clear()

    def update(self):
        keys = pygame.key.get_pressed()

        # throttle / brake
        if keys[pygame.K_w]:
            self.pedal = min(1.0, self.pedal + self.pedal_rate * self.veh.dt)
        elif keys[pygame.K_s]:
            self.pedal = max(-1.0, self.pedal - self.pedal_rate * self.veh.dt)
        else:
            if self.pedal > 0:
                self.pedal = max(0.0, self.pedal - self.pedal_rate * self.veh.dt)
            elif self.pedal < 0:
                self.pedal = min(0.0, self.pedal + self.pedal_rate * self.veh.dt)

        # steering
        if keys[pygame.K_a]:
            self.delta_req = max(-self.max_delta, self.delta_req - self.steer_rate * self.veh.dt)
        elif keys[pygame.K_d]:
            self.delta_req = min(self.max_delta, self.delta_req + self.steer_rate * self.veh.dt)
        else:
            # elastic return
            self.delta_req *= (1.0 - 5.0 * self.veh.dt)

        if not self.paused:
            X = self.veh.step(self.pedal, self.delta_req)
        else:
            X = self.veh.X

        poseV = state_to_poseV(X)
        self.trail.append((poseV[0], poseV[1]))
        if len(self.trail) > 1500:
            self.trail.pop(0)

    def draw(self, screen: pygame.Surface):
        screen.fill(CLR_BG)

        # grid
        for gx in range(0, 120, 10):
            p0 = world_to_screen(gx, 0, TRACK_SCALE, TRACK_ORIGIN)
            p1 = world_to_screen(gx, 90, TRACK_SCALE, TRACK_ORIGIN)
            pygame.draw.line(screen, (40, 40, 60), p0, p1, 1)
        for gy in range(0, 90, 10):
            p0 = world_to_screen(0, gy, TRACK_SCALE, TRACK_ORIGIN)
            p1 = world_to_screen(120, gy, TRACK_SCALE, TRACK_ORIGIN)
            pygame.draw.line(screen, (40, 40, 60), p0, p1, 1)

        # trajectory
        if len(self.trail) > 1:
            for i in range(len(self.trail) - 1):
                p0 = world_to_screen(*self.trail[i], TRACK_SCALE, TRACK_ORIGIN)
                p1 = world_to_screen(*self.trail[i + 1], TRACK_SCALE, TRACK_ORIGIN)
                pygame.draw.line(screen, (100, 140, 255), p0, p1, 2)

        X = self.veh.X
        poseV = state_to_poseV(X)
        draw_vehicle(screen, poseV, TRACK_SCALE, TRACK_ORIGIN, color=(230, 230, 230))

        u = X[3]
        v = X[4]
        yaw = X[2]
        hud1 = f"[DOUBLE-TRACK]  u={u:.2f} m/s  v={v:.2f} m/s  yaw={math.degrees(yaw):.1f}°"
        hud2 = f"pedal={self.pedal:+.2f}  delta={math.degrees(self.delta_req):+.1f}°  dt={self.veh.dt*1000:.1f} ms"
        hud3 = "W/S throttle-brake | A/D steer | SPACE pause | R reset | ESC menu"

        screen.blit(self.font.render(hud1, True, (230, 230, 230)), (20, 20))
        screen.blit(self.font.render(hud2, True, (200, 200, 200)), (20, 42))
        screen.blit(self.font.render(hud3, True, (170, 170, 170)), (20, 64))


class CombinedSim:
    MODE_NAME = "Combined sel_jr (long + lat)"

    def __init__(self):
        self.font = pygame.font.SysFont("Ubuntu Mono", 18)
        gp_lat.rerun_spawn("gp_lat_sim")
        self.cortex = False
        gp_lat.set_cortex_rerun(False)


        # plants
        self.lat = FrenetLateralPlant(W=3.0, V=3.5, L=2.5, dt=0.01)
        self.long = LongitudinalPlant(dt=0.01)

        self.lat.reset(n0=0.0, b0=0.0, c0=0.0)
        self.long.reset(s0=0.0, v0=self.lat.V, a0=0.0)

        gp_lat.set_write_csv(False)

        self.paused = False
        self.cam_s = 0.0
        self.trail = deque(maxlen=200)
        self.hist = deque(maxlen=800)

        self.goal = 50000.0
        self.la = 10.0
        self.veh_W = 0.7
        self.T_max = 20.0
        self.V_max = 10.0
        self.lat_tol = 0.3
        self.tol_obst = 0.3
        self.k_dot = 0.0

        self.last_j = 0.0
        self.last_r = 0.0

    def handle_event(self, e: pygame.event.Event):
        if e.type != pygame.KEYDOWN:
            return

        if e.key == pygame.K_SPACE:
            self.paused = not self.paused
        elif e.key == pygame.K_r:
            self.lat.reset(n0=0.0, b0=0.0, c0=0.0)
            self.long.reset(s0=0.0, v0=self.lat.V, a0=0.0)
            self.trail.clear()
            self.hist.clear()
        elif e.key == pygame.K_F1:
            self.cortex = not self.cortex
            gp_lat.set_cortex_rerun(self.cortex)

        elif e.key == pygame.K_UP:
            self.goal += 5.0
        elif e.key == pygame.K_DOWN:
            self.goal = max(0.0, self.goal - 5.0)

    def update(self):
        if self.paused:
            return

        k0 = gp_lat.states()
        k1 = gp_lat.states()

        sL, vL, aL = self.long.x
        sF, nF, bF, cF = self.lat.x

        k0.n = float(nF)
        k0.b = float(bF)
        k0.c = float(cF)

        k0.x = float(sL)
        k0.v = float(vL)
        k0.a = float(aL)

        k1.n = float(nF)
        k1.b = float(bF)
        k1.c = float(cF)

        dt = self.long.dt
        k1.x = float(sL + vL*dt)
        k1.v = float(vL + aL*dt)
        k1.a = float(aL)

        
        j_cmd, r_cmd = gp_lat.sel_jr(
            False,          
            0.0,            
            float(self.goal),
            float(self.la),
            float(self.veh_W),
            float(self.lat.W),
            k0, k1,
            float(self.T_max),
            float(self.V_max),
            float(self.lat_tol),
            float(self.tol_obst),
            float(self.k_dot),
        )

        if (not math.isfinite(j_cmd)) or (not math.isfinite(r_cmd)) or abs(j_cmd) > 1e6 or abs(r_cmd) > 1e6:
            print("sel_jr returned invalid -> resetting/holding commands")
            j_cmd, r_cmd = 0.0, 0.0

        self.last_j = float(j_cmd)
        self.last_r = float(r_cmd)

        
        self.long.step(j_cmd=self.last_j)
        self.lat.V = max(0.0, float(self.long.x[1]))
        self.lat.step(r_cmd=self.last_r)

        
        s, n, b, c = self.lat.x
        self.cam_s = 0.92*self.cam_s + 0.08*s
        self.trail.append((s, n))
        self.hist.append((s, n))

    def draw(self, screen: pygame.Surface):
        s, n, b, c = self.lat.x
        ORIGIN = frenet_camera_origin(self.cam_s)

        screen.fill(CLR_BG)
        draw_road(screen, s_center=self.cam_s, W=self.lat.W, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_trail(screen, self.trail, SCALE, ORIGIN)

        pose_lat, info_lat = self.lat.pose_for_render()
        draw_steer_arc_from_c(screen, pose_lat[0], pose_lat[1], pose_lat[2], info_lat["c"], SCALE, ORIGIN, arc_len=12.0)
        draw_vehicle(screen, pose_lat, SCALE, ORIGIN)
        draw_minimap(screen, self.hist, self.lat.W, self.lat.V)
        pose_long, info_long = self.long.pose_for_render(j_used=self.last_j)


        # long info
        sL, vL, aL = self.long.x

        hud1 = f"[COMBINED sel_jr]  j={self.last_j:+.3f}  r={self.last_r:+.3f}   cortex(F1)={'ON' if self.cortex else 'OFF'}"
        hud2 = f"LONG: s={sL:6.1f} m | v={vL*3.6:5.1f} km/h | a={aL:+.2f} m/s² | goal={self.goal:.1f}m | la={self.la:.1f}m"
        hud3 = f"LAT:  n={n:+.2f} m | b={math.degrees(b):+.1f}° | c={c:+.3f} 1/m | SPACE pause | R reset | ESC menu"

        screen.blit(self.font.render(hud1, True, (230, 230, 230)), (16, 16))
        screen.blit(self.font.render(hud2, True, (180, 200, 230)), (16, 40))
        screen.blit(self.font.render(hud3, True, (170, 170, 170)), (16, 64))


# ------------------------------------------------------------
#  MENU
# ------------------------------------------------------------
def draw_menu(screen: pygame.Surface, font: pygame.font.Font):
    screen.fill(CLR_BG)
    title = font.render("General Planner Simulator – MODE SELECTION", True, (230, 230, 230))
    screen.blit(title, (WIDTH//2 - title.get_width()//2, 80))

    lines = [
        "[1] Lateral Frenet controller",
        "[2] Longitudinal controller (fake jerk for now)",
        "[3] Double-track vehicle playground",
        "[4] Combined sel_jr (j+r together)",
        "",
        "Press ESC to quit.",
    ]
    y = 150
    for line in lines:
        surf = font.render(line, True, (200, 200, 210))
        screen.blit(surf, (WIDTH//2 - surf.get_width()//2, y))
        y += 30


# ------------------------------------------------------------
#  MAIN LOOP with state machine
# ------------------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Ubuntu Mono", 20)

    mode = "MENU"
    sim = None
    running = True

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
                break

            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                if mode == "MENU":
                    running = False
                else:
                    mode = "MENU"
                    sim = None
                continue

            # dispatch events to states
            if mode == "MENU":
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_1:
                        mode = "LATERAL"
                        sim = LateralSim()
                    elif e.key == pygame.K_2:
                        mode = "LONGITUDINAL"
                        sim = LongitudinalSim()
                    elif e.key == pygame.K_3:
                        mode = "VEHICLE"
                        sim = VehicleSim()
                    elif e.key == pygame.K_4:
                        mode = "COMBINED"
                        sim = CombinedSim()
            else:
                if sim is not None and hasattr(sim, "handle_event"):
                    sim.handle_event(e)

        if not running:
            break

        # UPDATE & DRAW
        if mode == "MENU":
            draw_menu(screen, font)
        else:
            if sim is not None:
                if hasattr(sim, "update"):
                    sim.update()
                if hasattr(sim, "draw"):
                    sim.draw(screen)
            else:
                draw_menu(screen, font)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
