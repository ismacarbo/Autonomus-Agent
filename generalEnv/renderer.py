import pygame
import math

def world_to_screen(x, y, scale, origin):
    ox, oy = origin
    sx = int(ox + x*scale)
    sy = int(oy - y*scale)  # invert Y for screen
    return sx, sy

def draw_vehicle(screen, state, scale, origin, color=(255,255,255)):
    x, y, yaw, v = state
    # simple triangle car
    L = 2.5
    W = 1.2
    pts = [
        ( L/2,  0.0),
        (-L/2,  W/2),
        (-L/2, -W/2),
    ]
    cos_y = math.cos(yaw); sin_y = math.sin(yaw)
    rot = lambda px,py: (x + px*cos_y - py*sin_y, y + px*sin_y + py*cos_y)
    pts_w = [rot(px,py) for (px,py) in pts]
    pts_s = [world_to_screen(px, py, scale, origin) for (px,py) in pts_w]
    pygame.draw.polygon(screen, color, pts_s, width=0)

def draw_ref_point(screen, ref, scale, origin, color=(0,200,255)):
    xr, yr = ref
    sx, sy = world_to_screen(xr, yr, scale, origin)
    pygame.draw.circle(screen, color, (sx, sy), 4)

def draw_path(screen, planner, scale, origin, color=(60, 200, 60)):
    # sample N points along planner
    N = 60
    for i in range(N-1):
        t0 = planner.T * i/(N-1)
        t1 = planner.T * (i+1)/(N-1)
        x0, y0 = planner.pos(t0)
        x1, y1 = planner.pos(t1)
        p0 = world_to_screen(x0, y0, scale, origin)
        p1 = world_to_screen(x1, y1, scale, origin)
        pygame.draw.line(screen, color, p0, p1, width=2)
