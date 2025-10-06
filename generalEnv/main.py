import math
import pygame
import numpy as np

from vehicle import DoubleTrackVehicle
from vehicle_model.vehicle_params import VehicleParams
from vehicle_model.pacejka_params import PacejkaParam
from renderer import world_to_screen, draw_vehicle


WIDTH, HEIGHT = 1000,750
SCALE=8.0
ORIGIN=(120,HEIGHT-120)

def state_to_poseV(X):
    x,y,psi,u,v,_=X[:6]
    v_abs=math.hypot(u,v)
    return [x,y,psi,v_abs]


def main():
    pygame.init()
    screen=pygame.display.set_mode((WIDTH, HEIGHT))
    clock=pygame.time.Clock()
    font=pygame.font.SysFont("Arial",18)

    veh=DoubleTrackVehicle(params=VehicleParams(), tire=PacejkaParam())


    pedal=0.0
    delta_req=0.0
    pedal_rate=2.5
    max_delta=0.6
    steer_rate=2.5
    paused=False
    running=True
    trail=[]

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: running = False
                elif e.key == pygame.K_SPACE: paused = not paused
                elif e.key == pygame.K_r: veh.reset(); pedal=0.0; delta_req=0.0; trail.clear()
    
        keys=pygame.key.get_pressed()
        if keys[pygame.K_w]: pedal = min(1.0, pedal + pedal_rate*veh.dt)
        elif keys[pygame.K_s]: pedal = max(-1.0, pedal - pedal_rate*veh.dt)
        else:
            # rilascio verso 0
            if   pedal > 0: pedal = max(0.0, pedal - pedal_rate*veh.dt)
            elif pedal < 0: pedal = min(0.0, pedal + pedal_rate*veh.dt)

        if keys[pygame.K_a]:
            delta_req = max(-max_delta, delta_req - steer_rate*veh.dt)
        elif keys[pygame.K_d]:
            delta_req = min( max_delta, delta_req + steer_rate*veh.dt)
        else:
            # ritorno elastico
            delta_req *= (1.0 - 5.0*veh.dt)

        if not paused:
            X = veh.step(pedal, delta_req)
            poseV = state_to_poseV(X)
            trail.append((poseV[0], poseV[1]))
            if len(trail) > 1500: trail.pop(0)
        else:
            X = veh.X
            poseV = state_to_poseV(X)

        screen.fill((15,18,30))

        for gx in range(0, 120, 10):
            p0 = world_to_screen(gx, 0, SCALE, ORIGIN)
            p1 = world_to_screen(gx, 90, SCALE, ORIGIN)
            pygame.draw.line(screen, (40,40,60), p0, p1, 1)
        for gy in range(0, 90, 10):
            p0 = world_to_screen(0, gy, SCALE, ORIGIN)
            p1 = world_to_screen(120, gy, SCALE, ORIGIN)
            pygame.draw.line(screen, (40,40,60), p0, p1, 1)

        if len(trail) > 1:
            for i in range(len(trail)-1):
                p0 = world_to_screen(*trail[i], SCALE, ORIGIN)
                p1 = world_to_screen(*trail[i+1], SCALE, ORIGIN)
                pygame.draw.line(screen, (100,140,255), p0, p1, 2)

        draw_vehicle(screen, poseV, SCALE, ORIGIN, color=(230))

        u = X[3]; v = X[4]; yaw = X[2]
        hud1 = f"u={u:.2f} m/s  v={v:.2f} m/s  yaw={math.degrees(yaw):.1f}°"
        hud2 = f"pedal={pedal:+.2f}  delta={math.degrees(delta_req):+.1f}°  dt={veh.dt*1000:.1f} ms"
        screen.blit(font.render(hud1, True, (230,230,230)), (20, 20))
        screen.blit(font.render(hud2, True, (200,200,200)), (20, 42))
        screen.blit(font.render("W/S throttle-brake | A/D steer | SPACE pause | R reset | ESC quit", True, (170,170,170)), (20, 64))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()