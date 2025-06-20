import numpy as np
import matplotlib.pyplot as plt


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