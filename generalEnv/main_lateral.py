import math, pygame
from collections import deque
from frenet_plant import FrenetLateralPlant
from renderer import*

WIDTH, HEIGHT = 1000, 700
SCALE = 80.0

def camera_origin(s):
    center_x_px = WIDTH * 0.35
    ox = center_x_px - s * SCALE
    oy = HEIGHT // 2
    return (ox, oy)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("Ubuntu Mono", 18)

    plant = FrenetLateralPlant(W=3.0, V=3.5, L=2.5, dt=0.005)
    plant.reset(n0=0.0, b0=0.3, c0=0.0)
    plant.n_ref = 0.0

    paused = False
    running = True

    cam_s = 0.0
    trail = deque(maxlen=150)     # last 150 points in the world
    hist  = deque(maxlen=800)     # for mini-map

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: running = False
                elif e.key == pygame.K_SPACE: paused = not paused
                elif e.key == pygame.K_r: plant.reset(n0=0.0, b0=0.3, c0=0.0)
                elif e.key == pygame.K_1: plant.n_ref = +(plant.W/2 - 0.4)
                elif e.key == pygame.K_2: plant.n_ref = -(plant.W/2 - 0.4)
                elif e.key == pygame.K_0: plant.n_ref = 0.0
                elif e.key in (pygame.K_LEFT, pygame.K_a):  plant.n_ref -= 0.1
                elif e.key in (pygame.K_RIGHT, pygame.K_d): plant.n_ref += 0.1
                elif e.key in (pygame.K_MINUS, pygame.K_KP_MINUS): plant.T_max = max(1.0, plant.T_max - 0.5)
                elif e.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS): plant.T_max = min(30.0, plant.T_max + 0.5)
                plant.n_ref = max(-(plant.W/2 - 0.2), min(plant.W/2 - 0.2, plant.n_ref))

        if not paused:
            plant.step()

        s, n, b, c = plant.x
        cam_s = 0.92*cam_s + 0.08*s
        ORIGIN = camera_origin(cam_s)
        trail.append((s, n))
        hist.append((s, n))

        # draw
        screen.fill((15,18,30))
        draw_road(screen, s_center=cam_s, W=plant.W, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_nref(screen, s_center=cam_s, n_ref=plant.n_ref, W=plant.W, SCALE=SCALE, ORIGIN=ORIGIN)
        draw_trail(screen, trail, SCALE, ORIGIN)
        pose, info = plant.pose_for_render()
        draw_steer_arc_from_c(screen,pose[0],pose[1],pose[2],info["c"],SCALE,ORIGIN,arc_len=12.0)
        draw_vehicle(screen, pose, SCALE, ORIGIN, color=(235,235,235))
        draw_minimap(screen, hist, plant.W, plant.V)

        # HUD
        hud1 = f"V={plant.V:.2f} m/s | n={n:+.2f} m | b={math.degrees(b):+.1f}° | c={c:+.3f} 1/m"
        hud2 = f"n_ref={plant.n_ref:+.2f} m | T_max={plant.T_max:.1f}s | δ≈{math.degrees(info['delta']):+.1f}°"
        screen.blit(font.render(hud1, True, (230,230,230)), (16, 16))
        screen.blit(font.render(hud2, True, (180,200,230)), (16, 40))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
