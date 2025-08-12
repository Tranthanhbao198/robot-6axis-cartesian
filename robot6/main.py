import argparse, sys, json, math
import numpy as np
from .robot import Simple6DOF, rad
from .kinematics import Pose, rpy_to_matrix, matrix_to_rpy
from .trajectory import interpolate_cartesian_poses
from .obstacles import robot_collision
from .visualize import visualize_robot_path

def parse_args():
    ap = argparse.ArgumentParser(description="Robot 6 Axis - Cartesian Motion (no ROS)")
    ap.add_argument("--x", type=float, default=0.4, help="target x [m]")
    ap.add_argument("--y", type=float, default=0.0, help="target y [m]")
    ap.add_argument("--z", type=float, default=0.3, help="target z [m]")
    ap.add_argument("--roll", type=float, default=0.0, help="target roll [deg]")
    ap.add_argument("--pitch", type=float, default=0.0, help="target pitch [deg]")
    ap.add_argument("--yaw", type=float, default=0.0, help="target yaw [deg]")
    ap.add_argument("--steps", type=int, default=50, help="interpolation steps")
    ap.add_argument("--visualize", action="store_true", help="show matplotlib 3D visualization")
    ap.add_argument("--q0", type=float, nargs=6, default=[0, -60, 60, 0, 30, 0], help="initial joint guess [deg x6]")
    ap.add_argument("--obstacles", type=str, default="", help="JSON list of spheres: [[[cx,cy,cz], r], ...]")
    return ap.parse_args()

def main():
    args = parse_args()
    robot = Simple6DOF()
    q0 = np.array([rad(d) for d in args.q0], dtype=float)

    T0, _ = robot.fk(q0)
    current = Pose(T0[:3, 3], T0[:3, :3])
    cr, cp, cy = matrix_to_rpy(current.R)
    print(f"[INFO] Current EE pose: p={np.round(current.p, 3)} | rpy(deg)={cr*180/np.pi:.2f}, {cp*180/np.pi:.2f}, {cy*180/np.pi:.2f}")

    R_tgt = rpy_to_matrix(math.radians(args.roll), math.radians(args.pitch), math.radians(args.yaw))
    target = Pose(np.array([args.x, args.y, args.z]), R_tgt)

    ee_path = interpolate_cartesian_poses(current, target, steps=args.steps)
    obstacles = json.loads(args.obstacles) if args.obstacles else []

    q_list = [q0.copy()]
    q = q0.copy()
    for i, pose in enumerate(ee_path[1:], start=1):
        # dùng IK mặc định (weighted + fallback)
        q, ok = robot.ik(pose, q)
        if not ok:
            print(f"[WARN] IK did not fully converge at step {i}.")
        if obstacles and robot_collision(robot, q, obstacles):
            bumped = False
            for bump in [0.02, 0.04, 0.06]:
                bumped_pose = Pose(pose.p + np.array([0, 0, bump]), pose.R)
                q_try, ok2 = robot.ik(bumped_pose, q)
                if ok2 and not robot_collision(robot, q_try, obstacles):
                    q = q_try
                    bumped = True
                    break
            if not bumped:
                print(f"[WARN] Collision detected at step {i} and simple avoidance failed.")
        q_list.append(q.copy())
        print(f"Step {i:03d}/{len(ee_path)-1} | q(deg)={np.degrees(q).round(2)} | ee={pose.p.round(3)}")

    if args.visualize:
        visualize_robot_path(robot, q_list, obstacles=obstacles, ee_path=ee_path, show=True)

if __name__ == "__main__":
    sys.exit(main())

