# renderer.py
import math
import pygame

# --- palette (soft dark) ---
CLR_BG         = (15, 18, 30)
CLR_ASPHALT    = (30, 34, 48)
CLR_LANE_EDGE  = (180, 180, 180)
CLR_LANE_DASH  = (210, 210, 210)
CLR_TICKS      = (120, 120, 130)
CLR_CAR        = (235, 235, 235)
CLR_NREF       = (90, 210, 255)
CLR_TRAIL      = (60, 130, 255)
CLR_MINI_BG    = (20, 22, 32)
CLR_MINI_BORD  = (70, 75, 95)
CLR_MINI_GRID  = (40, 45, 60)


# Prova ad usare gfxdraw se disponibile; altrimenti fallback
try:
    from pygame import gfxdraw
    HAS_GFX_AA = hasattr(gfxdraw, "aaline") and hasattr(gfxdraw, "aapolygon")
except Exception:
    gfxdraw = None
    HAS_GFX_AA = False

def draw_steer_arc(surf, pose, L, SCALE, ORIGIN, max_len=12.0):
    """
    Disegna l'arco di sterzo davanti al veicolo.
    Usa yaw=b e curvatura c dalla pose (pose_for_render).
    """
    s, n, yaw, V = pose
    # delta equivalente lo stai già mostrando come atan(L*c), ma qui usiamo direttamente la curvatura:
    # raggio = 1/|c| (se c ~ 0, approssima a linea retta)
    # traccio l'arco per +max_len metri in avanti.
    # Nota: stiamo nel frame Frenet (x=s, y=n)
    try:
        c = float(np.tan(0))  # placeholder se non hai c nella pose
    except:
        c = 0.0

    # Recupera c da pose_for_render via info esterno se preferisci.
    # Qui ipotizziamo che tu lo abbia nel dict info o rimandi b/c in pose.
    # Se non lo hai, passa 'c' come arg a questa funzione.

def draw_steer_arc_from_c(surf, s, n, yaw, c, SCALE, ORIGIN, arc_len=12.0, step=0.2):
    """
    Variante esplicita: disegna l'arco usando direttamente c (curvatura) e yaw=b.
    arc_len: lunghezza dell’arco in metri; step: risoluzione.
    """
    import math
    pts = []
    if abs(c) < 1e-6:
        # quasi dritto
        for x in np.arange(0.0, arc_len+1e-6, step):
            sx = s + x*math.cos(yaw)
            sn = n + x*math.sin(yaw)
            pts.append(world_to_screen(sx, sn, SCALE, ORIGIN))
    else:
        R = 1.0/abs(c)
        # segno curvatura: >0 curva verso +n, <0 verso -n
        turn = 1.0 if c > 0 else -1.0
        # centro rotazione nel frame (s,n)
        cx = s - turn*R*math.sin(yaw)
        cy = n + turn*R*math.cos(yaw)
        # angolo iniziale del raggio che va dal centro al veicolo
        theta0 = math.atan2(n - cy, s - cx)
        # sweeping: arco (0..arc_len)
        sweep = arc_len / R
        # direzione sweep coerente col segno di c
        theta1 = theta0 + turn * sweep

        N = max(8, int(arc_len/step))
        for i in range(N+1):
            t = i / max(1, N)
            th = theta0 + (theta1 - theta0) * t
            px = cx + R * math.cos(th)
            py = cy + R * math.sin(th)
            pts.append(world_to_screen(px, py, SCALE, ORIGIN))

    if len(pts) >= 2:
        pygame.draw.lines(surf, (140, 200, 140), False, pts, 2)



def draw_aa_line(surf, color, p0, p1, width=1):
    """Linea antialias con fallback (senza ricorsione!)."""
    # 1) antialias nativo (se c'è)
    if hasattr(pygame.draw, "aaline"):
        pygame.draw.aaline(surf, color, p0, p1)
        if width > 1:
            pygame.draw.line(surf, color, p0, p1, width)
        return
    # 2) gfxdraw (alcune build non hanno aaline)
    if HAS_GFX_AA:
        pygame.draw.line(surf, color, p0, p1, width)
        try:
            gfxdraw.aaline(surf, p0[0], p0[1], p1[0], p1[1], color)
        except Exception:
            pass
        return
    # 3) fallback semplice
    pygame.draw.line(surf, color, p0, p1, width)


def draw_aa_polygon(surf, color, pts):
    """Poligono con bordo smussato se possibile."""
    pygame.draw.polygon(surf, color, pts)
    if HAS_GFX_AA:
        try:
            gfxdraw.aapolygon(surf, pts, color)
        except Exception:
            pass


def world_to_screen(s, n, SCALE, ORIGIN):
    ox, oy = ORIGIN
    return (int(ox + s * SCALE), int(oy - n * SCALE))


def draw_vehicle(surf, pose, SCALE, ORIGIN, color=CLR_CAR):
    s, n, yaw, _ = pose
    L = 2.2
    w = 1.1
    pts_local = [(+0.75*L, 0.0), (-0.55*L, +0.5*w), (-0.55*L, -0.5*w)]
    c, ss = math.cos(yaw), math.sin(yaw)
    pts = []
    for x, y in pts_local:
        sx = s + (x*c - y*ss)
        sn = n + (x*ss + y*c)
        pts.append(world_to_screen(sx, sn, SCALE, ORIGIN))
    draw_aa_polygon(surf, color, pts)
    # “naso”
    nose0 = world_to_screen(s + 0.65*L* c, n + 0.65*L* ss, SCALE, ORIGIN)
    nose1 = world_to_screen(s + 0.85*L* c, n + 0.85*L* ss, SCALE, ORIGIN)
    draw_aa_line(surf, (255, 200, 120), nose0, nose1, 2)



def draw_road(surf, s_center, W, SCALE, ORIGIN):
    """Asfalto, bordi, tratteggio centrale, pioli ogni 5 m."""
    w_pix = int(W * SCALE)

    # banda asfalto
    w_screen = surf.get_width()
    top_left_y = int(ORIGIN[1] - (W / 2) * SCALE)
    rect = pygame.Rect(0, top_left_y, w_screen, w_pix)
    pygame.draw.rect(surf, (30, 34, 48), rect)

    # bordi corsia
    for off in (-W / 2, W / 2):
        p0 = world_to_screen(s_center - 30, off, SCALE, ORIGIN)
        p1 = world_to_screen(s_center + 30, off, SCALE, ORIGIN)
        draw_aa_line(surf, (180, 180, 180), p0, p1, 2)

    # tratteggio centrale
    dash, gap = 1.5, 1.5
    period = dash + gap
    view_back, view_fwd = 5.0, 25.0
    s0 = s_center - view_back
    s1 = s_center + view_fwd
    k0 = math.floor(s0 / period)
    s = k0 * period
    while s < s1:
        a0 = max(s, s0)
        a1 = min(s + dash, s1)
        if a1 > a0:
            p0 = world_to_screen(a0, 0.0, SCALE, ORIGIN)
            p1 = world_to_screen(a1, 0.0, SCALE, ORIGIN)
            draw_aa_line(surf, (210, 210, 210), p0, p1, 3)
        s += period

    # pioli ai bordi
    tick_ds = 5.0
    k0 = math.floor(s0 / tick_ds)
    s = k0 * tick_ds
    h_pix = 10
    col = (120, 120, 130)
    while s < s1:
        for off in (-W / 2, W / 2):
            px = world_to_screen(s, off, SCALE, ORIGIN)
            p_up = (px[0], px[1] - h_pix if off > 0 else px[1] + h_pix)
            draw_aa_line(surf, col, px, p_up, 2)
        s += tick_ds


def draw_nref(surf, s_center, n_ref, W, SCALE, ORIGIN):
    p0 = world_to_screen(s_center - 30, n_ref, SCALE, ORIGIN)
    p1 = world_to_screen(s_center + 30, n_ref, SCALE, ORIGIN)
    draw_aa_line(surf, (90, 210, 255), p0, p1, 2)


def draw_trail(surf, trail_pts, SCALE, ORIGIN, color=(60, 130, 255)):
    if len(trail_pts) < 2:
        return
    pts = [world_to_screen(s, n, SCALE, ORIGIN) for (s, n) in trail_pts]
    pygame.draw.lines(surf, color, False, pts, 2)


def draw_minimap(surf, history, W, V):
    if len(history) < 2:
        return
    # pannello
    w, h = 260, 110
    margin = 14
    x0 = surf.get_width() - w - margin
    y0 = margin
    pygame.draw.rect(surf, CLR_MINI_BG, (x0, y0, w, h), border_radius=8)
    pygame.draw.rect(surf, CLR_MINI_BORD, (x0, y0, w, h), 1, border_radius=8)

    # prendo solo la coda
    hist_tail = list(history)[-500:]
    s_vals    = [p[0] for p in hist_tail]
    smin, smax = s_vals[0], s_vals[-1]
    span = max(1e-6, smax - smin)

    # funzione di mapping (s,n) -> (px,py) nel riquadro, con margini
    def map_sn(_s, _n):
        px = x0 + 8 + int(((_s - smin)/span) * (w-16))
        # n in [-W/2, +W/2] -> [h-10 .. 10]
        py = y0 + 8 + int((0.5 - _n/W) * (h-16))
        return px, py

    # griglia orizzontale (bordi corsia e centro)
    for nn, col in ((+W/2, (70,70,90)), (0.0, (60,60,80)), (-W/2, (70,70,90))):
        p0 = map_sn(smin, nn)
        p1 = map_sn(smax, nn)
        pygame.draw.line(surf, CLR_MINI_GRID, p0, p1, 1)

    # traiettoria sfumata (alpha crescente con il tempo)
    pts = [map_sn(s, n) for (s, n) in hist_tail]
    # disegno segmenti con alpha crescente
    if len(pts) >= 2:
        segs = list(zip(pts[:-1], pts[1:]))
        N = len(segs)
        for i, (a, b) in enumerate(segs):
            t = (i+1)/N
            col = (int(CLR_TRAIL[0]*(0.6+0.4*t)),
                   int(CLR_TRAIL[1]*(0.6+0.4*t)),
                   int(CLR_TRAIL[2]*(0.6+0.4*t)))
            pygame.draw.line(surf, col, a, b, 2)

    # bordino centro corsia
    p0 = map_sn(smin, 0.0)
    p1 = map_sn(smax, 0.0)
    pygame.draw.line(surf, (90,90,110), p0, p1, 1)
