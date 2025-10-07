
import math, pygame
from frenet_plant import FrenetLateralPlant
from renderer import world_to_screen, draw_vehicle

WIDTH, HEIGHT = 900, 700
SCALE = 80.0

def camera_origin(s):
    center_x_px = WIDTH * 0.4            
    ox = center_x_px - s * SCALE         
    oy = HEIGHT // 2                     
    return (ox, oy)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("Arial", 16)

    plant = FrenetLateralPlant(W=3.0, V=3.5, L=2.5, dt=0.01)
    plant.reset(n0=0.0, b0=0.3, c0=0.0)   

    paused = False
    running = True

    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: running = False
                elif e.key == pygame.K_SPACE: paused = not paused
                elif e.key == pygame.K_r: plant.reset(n0=0.0, b0=0.3, c0=0.0)

        if not paused:
            plant.step()

        pose, info = plant.pose_for_render()
        s, n, b, c = plant.x
        ORIGIN = camera_origin(s)

        
        screen.fill((15,18,30))

        
        view_len = 12.0  
        s0, s1 = s - 2.0, s + view_len
        for off in (-plant.W/2, plant.W/2):
            p0 = world_to_screen(s0, off, SCALE, ORIGIN)
            p1 = world_to_screen(s1, off, SCALE, ORIGIN)
            pygame.draw.line(screen, (180,180,180), p0, p1, 2)

        
        draw_vehicle(screen, pose, SCALE, ORIGIN, color=(230,230,230))

        
        hud1 = f"V={plant.V:.2f} m/s | n={n:.2f} m | b={math.degrees(b):.1f}° | c={c:.3f} 1/m"
        hud2 = f"delta≈{math.degrees(info['delta']):+.1f}° | SPACE pause, R reset"
        screen.blit(font.render(hud1, True, (230,230,230)), (20, 20))
        screen.blit(font.render(hud2, True, (200,200,200)), (20, 42))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
