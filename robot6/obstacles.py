import numpy as np

def segment_point_distance(a, b, p):
    ab = b - a
    t = np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12)
    t = np.clip(t, 0.0, 1.0)
    proj = a + t * ab
    return np.linalg.norm(p - proj)

def link_sphere_collision(a, b, center, radius, margin=0.0):
    return segment_point_distance(a, b, center) <= (radius + margin)

def robot_collision(robot, q, obstacles):
    _, Ts = robot.fk(q)
    for i in range(len(Ts) - 1):
        a = Ts[i][:3, 3]
        b = Ts[i + 1][:3, 3]
        for (c, r) in obstacles:
            if link_sphere_collision(a, b, np.array(c), r):
                return True
    return False

