import numpy as np
from .kinematics import quat_from_matrix, quat_to_matrix, quat_slerp, Pose

def interpolate_cartesian_poses(start_pose: Pose, goal_pose: Pose, steps: int = 50):
    p0, p1 = start_pose.p, goal_pose.p
    q0 = quat_from_matrix(start_pose.R)
    q1 = quat_from_matrix(goal_pose.R)
    poses = []
    for i in range(steps + 1):
        t = i / steps
        p = (1 - t) * p0 + t * p1
        q = quat_slerp(q0, q1, t)
        R = quat_to_matrix(q)
        poses.append(Pose(p, R))
    return poses

