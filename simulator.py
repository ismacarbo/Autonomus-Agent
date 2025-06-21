import numpy as np
import matplotlib.pyplot as plt
import time


#defining walls, segments of (x1, y1), (x2, y2)
walls = [
    ((0, 0), (10, 0)),   # muro basso
    ((10, 0), (10, 10)), # muro destro
    ((10, 10), (0, 10)), # muro alto
    ((0, 10), (0, 0)),   # muro sinistro
    
    ((3, 3), (7, 3)),
    ((7, 3), (7, 7)),
    ((7, 7), (3, 7)),
    ((3, 7), (3, 3)),
]

#intersect_ray-segment, rayOrigin: radius of the ray, rayDir: direction of the ray, seg is the segment
def intersect_ray_casting(rayOrigin,rayDir,seg):
    p,r=np.array(rayOrigin), np.array(rayDir)
    a,b=np.array(seg[0]), np.array(seg[1])-np.array(seg[0])
    #b is the direction vector of the segment

    #2x2 matrix of the linear system
    M=np.column_stack((-r,b))

    try:
        solution=np.linalg.solve(M,a-p)
    except np.linalg.LinAlgError:
        #singolar matrix -> no solution
        return None
    
    t,u=solution

    #valid intersection
    if t>=0 and 0<=u<=1:
        #intersection point
        intersectionPoint=p+t*r
        return intersectionPoint
    
    return None


#emulates lidar scan, rotates 360 degrees
def lidar_scan(pos,num_beans=120,max_range=15.0):
    #generates angles from 2 to 2pi
    angles=np.linspace(0,2*np.pi,num_beans,endpoint=False)
    #initialize distance array
    dists=np.full(num_beans,max_range)

    for i,theta in enumerate(angles):
        rayDir=np.array([np.cos(theta),np.sin(theta)])
        closest=max_range
        
        #for each wall check intersection
        for seg in walls:
            intersectionPoint=intersect_ray_casting(pos,rayDir,seg)
            if intersectionPoint is not None:
                dist=np.linalg.norm(intersectionPoint-pos)
                if dist<closest:
                    closest=dist

        #update distance
        dists[i]=closest

    return angles,dists

def reactive_control(dists,angles,safe_dist=0.5,max_v=0.5,max_omega=1.0):
    #find the angle with the minimum distance
    N=len(dists)
    mid=N//2

    front_idxs=[(mid-1)%N ,mid, (mid+1)%N]
    front_dists=dists[front_idxs]
    front_angles=angles[front_idxs]

    idx_min=front_dists.argmin()
    min_angle=front_angles[idx_min]
    min_dist=front_dists[idx_min]

    if min_dist<safe_dist:
        #obstacle too close: stop and turn away
        v=0.0

        omega=-np.sign(min_angle)*max_omega
    else:
        #obstacle far enough: move forward and turn to face the obstacle based on min_angle
        #if obstacle is moved lateraly then min_angle will change
        v=max_v*(min_dist/safe_dist) if min_dist<2*safe_dist else max_v
        omega=-min_angle #turn to face the obstacle
    return v,omega

#plotting
plt.ion()
fig, ax = plt.subplots(figsize=(6,6))

for seg in walls:
    (x0, y0), (x1, y1) = seg
    ax.plot([x0, x1], [y0, y1], 'k-')
ax.set_xlim(-1, 11)
ax.set_ylim(-1, 11)
ax.set_aspect('equal')
ax.set_title("LiDAR Simulation 2D")
ax.set_xlabel("X"); ax.set_ylabel("Y")

#trajectory of the robot
traj_line, = ax.plot([], [], 'b.-', lw=1)
robot_dot, = ax.plot([], [], 'ro', ms=8)

dt = 0.1
pos = np.array([5.0, 1.0])
theta = np.pi/2
trajectory = [pos.copy()]

for _ in range(200):
    angles, dists = lidar_scan(pos)
    v, omega = reactive_control(dists, angles)

    pos += np.array([np.cos(theta), np.sin(theta)]) * v * dt
    theta += omega * dt
    trajectory.append(pos.copy())

    traj = np.array(trajectory)
    traj_line.set_data(traj[:,0], traj[:,1])
    robot_dot.set_data([pos[0]], [pos[1]])

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(dt)

plt.ioff()
plt.show()
