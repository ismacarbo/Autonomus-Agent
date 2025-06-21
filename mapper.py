import numpy as np
import matplotlib.pyplot as plt
import time

# --- GLOBAL PARAMETERS ---
MAP_SIZE      = 200       # grid size (number of cells per side)
RESOLUTION    = 0.1       # size of each cell in meters
MAX_RANGE     = 15.0      # maximum LiDAR range in meters
LOG_ODDS_FREE = -1.0      # log-odds increment for free cells
LOG_ODDS_OCC  = +2.0      # log-odds increment for occupied cells
LOG_ODDS_MIN  = -5.0      # minimum log-odds (almost certainly free)
LOG_ODDS_MAX  = +5.0      # maximum log-odds (almost certainly occupied)

# Initialize the log-odds grid to zero (unknown)
log_odds = np.zeros((MAP_SIZE, MAP_SIZE))

# Define walls for simulation as line segments from (x1, y1) to (x2, y2)
walls = [
    ((0, 0), (10, 0)),   # bottom wall
    ((10, 0), (10, 10)), # right wall
    ((10, 10), (0, 10)), # top wall
    ((0, 10), (0, 0)),   # left wall
    # internal obstacle: square from (3,3) to (7,7)
    ((3, 3), (7, 3)),
    ((7, 3), (7, 7)),
    ((7, 7), (3, 7)),
    ((3, 7), (3, 3)),
]

# ---------------------------------------------------------------------------------
# FUNCTION: world_to_map
# Converts world coordinates (x, y) to map cell indices (i, j).
# Computes indices and clips to grid bounds.
# ---------------------------------------------------------------------------------
def world_to_map(x, y):
    i = int((x + (MAP_SIZE * RESOLUTION) / 2) / RESOLUTION)
    j = int((y + (MAP_SIZE * RESOLUTION) / 2) / RESOLUTION)
    return np.clip(i, 0, MAP_SIZE - 1), np.clip(j, 0, MAP_SIZE - 1)

# ---------------------------------------------------------------------------------
# FUNCTION: bresenham
# Gets discrete grid cells traversed by a line between (i0, j0) and (i1, j1).
# ---------------------------------------------------------------------------------
def bresenham(i0, j0, i1, j1):
    cells = []
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i1 >= i0 else -1
    sj = 1 if j1 >= j0 else -1
    err = di - dj
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if (i == i1) and (j == j1):
            break
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj
    return cells

# ---------------------------------------------------------------------------------
# FUNCTION: intersect_ray_casting
# Computes intersection of ray with segment, returns point or None.
# ---------------------------------------------------------------------------------
def intersect_ray_casting(ray_origin, ray_dir, seg):
    p = np.array(ray_origin, dtype=float)
    r = np.array(ray_dir, dtype=float)
    a = np.array(seg[0], dtype=float)
    b = np.array(seg[1], dtype=float) - a
    M = np.column_stack((-r, b))
    try:
        t, u = np.linalg.solve(M, a - p)
    except np.linalg.LinAlgError:
        return None
    if (t >= 0) and (0 <= u <= 1):
        return p + t * r
    return None

# ---------------------------------------------------------------------------------
# FUNCTION: lidar_scan
# Simulates 360° LiDAR, returns angles and distances.
# ---------------------------------------------------------------------------------
def lidar_scan(pos, num_beams=120):
    angles = np.linspace(0, 2*np.pi, num_beams, endpoint=False)
    dists  = np.full(num_beams, MAX_RANGE)
    for i, theta in enumerate(angles):
        ray_dir = np.array([np.cos(theta), np.sin(theta)])
        closest = MAX_RANGE
        for seg in walls:
            pt = intersect_ray_casting(pos, ray_dir, seg)
            if pt is not None:
                d = np.linalg.norm(pt - pos)
                if d < closest:
                    closest = d
        dists[i] = closest
    return angles, dists

# ---------------------------------------------------------------------------------
# FUNCTION: update_map
# Integrates LiDAR scan into log-odds grid:
# 1) Compute endpoint, 2) Bresenham ray, 3) update free/occupied
# ---------------------------------------------------------------------------------
def update_map(pose, angles, dists):
    x, y, th = pose
    i0, j0 = world_to_map(x, y)
    for theta, dist in zip(angles, dists):
        r = min(dist, MAX_RANGE)
        x_end = x + r * np.cos(th + theta)
        y_end = y + r * np.sin(th + theta)
        i1, j1 = world_to_map(x_end, y_end)
        ray_cells = bresenham(i0, j0, i1, j1)
        # update free cells
        for (i, j) in ray_cells[:-1]:
            log_odds[i, j] = max(LOG_ODDS_MIN, log_odds[i, j] + LOG_ODDS_FREE)
        # update occupied cell
        if dist < MAX_RANGE:
            i_occ, j_occ = ray_cells[-1]
            log_odds[i_occ, j_occ] = min(LOG_ODDS_MAX, log_odds[i_occ, j_occ] + LOG_ODDS_OCC)

# ---------------------------------------------------------------------------------
# FUNCTION: reactive_control
# Basic obstacle avoidance: front three beams, stop and turn or go forward.
# ---------------------------------------------------------------------------------
def reactive_control(dists, angles, safe_dist=0.5, max_v=0.4, max_omega=1.0):
    N = len(dists)
    idxs = [(N//2 + k) % N for k in (-1, 0, 1)]
    f_dists = dists[idxs]
    f_angles = angles[idxs]
    idx_min = np.argmin(f_dists)
    min_a, min_d = f_angles[idx_min], f_dists[idx_min]
    if min_d < safe_dist:
        return 0.0, -np.sign(min_a) * max_omega
    v = max_v if min_d > 2*safe_dist else max_v * (min_d / (2*safe_dist))
    omega = -0.5 * min_a
    return v, omega

# ---------------------------------------------------------------------------------
# SIMULATION AND VISUALIZATION ---
pos = np.array([5.0, 1.0])
theta = np.pi/2
traj = [pos.copy()]
dt = 0.1

plt.ion()
fig, ax_map = plt.subplots(figsize=(6,6))

for step in range(400):
    angles, dists = lidar_scan(pos)
    # draw rays on grid
    endpoints = [(pos + np.array([np.cos(theta), np.sin(theta)])*dist) for theta, dist in zip(angles, dists)]

    update_map((pos[0], pos[1], theta), angles, dists)
    v, w = reactive_control(dists, angles)
    pos += np.array([np.cos(theta), np.sin(theta)]) * v * dt
    theta += w * dt
    traj.append(pos.copy())

    ax_map.clear()
    prob = 1 - 1 / (1 + np.exp(log_odds))
    ax_map.imshow(prob.T, origin='lower', cmap='gray',
                  extent=[-MAP_SIZE*RESOLUTION/2, MAP_SIZE*RESOLUTION/2,
                          -MAP_SIZE*RESOLUTION/2, MAP_SIZE*RESOLUTION/2])
    # plot rays
    for pt in endpoints:
        ax_map.plot([pos[0], pt[0]], [pos[1], pt[1]], 'r-', alpha=0.1)
    # plot trajectory and robot
    traj_arr = np.array(traj)
    ax_map.plot(traj_arr[:,0], traj_arr[:,1], 'b-', lw=1)
    ax_map.plot(pos[0], pos[1], 'ro')
    ax_map.set_title(f"Step {step}: Occupancy Grid Mapping with Rays")
    ax_map.set_xlabel('X (m)')
    ax_map.set_ylabel('Y (m)')
    plt.pause(0.01)

plt.ioff()
plt.show()
