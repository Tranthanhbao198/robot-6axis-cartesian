import argparse, sys, json, math
import numpy as np
from .robot import Simple6DOF, rad
from .kinematics import Pose, rpy_to_matrix
from .trajectory import interpolate_cartesian_poses
from .obstacles import robot_collision
from .visualize import visualize_robot_path

def segment_cartesian_move(robot, q_start, start_pose, target_pose, steps=40, obstacles=None, verbose=True):
    ee_path = interpolate_cartesian_poses(start_pose, target_pose, steps=steps)
    q_list = [q_start.copy()]
    q = q_start.copy()
    for i, pose in enumerate(ee_path[1:], start=1):
        q, ok = robot.ik(pose, q)
        if obstacles and robot_collision(robot, q, obstacles):
            bumped = False
            for bump in [0.02, 0.04, 0.06]:
                bumped_pose = Pose(pose.p + np.array([0, 0, bump]), pose.R)
                q_try, ok2 = robot.ik(bumped_pose, q)
                if ok2 and not robot_collision(robot, q_try, obstacles):
                    q = q_try; bumped = True; break
            if not bumped and verbose:
                print(f"[WARN] Collision at waypoint {i} (segment).")
        q_list.append(q.copy())
        if verbose:
            print(f"[SEG] {i:03d}/{len(ee_path)-1} | q(deg)={np.degrees(q).round(2)} | ee={pose.p.round(3)}")
    return q_list, ee_path

def main():
    ap = argparse.ArgumentParser(description="Pick & Place scenario (no ROS)")
    ap.add_argument("--q0", type=float, nargs=6, default=[0, -60, 60, 0, 30, 0], help="initial joint guess [deg x6]")
    # downward-facing (pitch=180°)
    ap.add_argument("--pick_x", type=float, default=0.45)
    ap.add_argument("--pick_y", type=float, default=0.10)
    ap.add_argument("--pick_z", type=float, default=0.20)
    ap.add_argument("--place_x", type=float, default=0.10)
    ap.add_argument("--place_y", type=float, default=0.45)
    ap.add_argument("--place_z", type=float, default=0.20)
    ap.add_argument("--approach_dz", type=float, default=0.10)
    ap.add_argument("--steps", type=int, default=35)
    ap.add_argument("--obstacles", type=str, default="")
    ap.add_argument("--visualize", action="store_true")
    args = ap.parse_args()

    robot = Simple6DOF()
    q = np.array([rad(d) for d in args.q0], dtype=float)
    T0, _ = robot.fk(q); current = Pose(T0[:3, 3], T0[:3, :3])

    R_down = rpy_to_matrix(0.0, math.pi, 0.0)  # pitch = 180°
    pick = Pose(np.array([args.pick_x, args.pick_y, args.pick_z]), R_down)
    place = Pose(np.array([args.place_x, args.place_y, args.place_z]), R_down)
    pre_pick = Pose(pick.p + np.array([0, 0, args.approach_dz]), R_down)
    pre_place = Pose(place.p + np.array([0, 0, args.approach_dz]), R_down)

    obstacles = json.loads(args.obstacles) if args.obstacles else []

    q_list_all = [q.copy()]
    ee_all = []

    print("[PHASE] Move to pre-pick")
    seg_qs, ee_path = segment_cartesian_move(robot, q, current, pre_pick, steps=args.steps, obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path

    print("[PHASE] Approach to pick")
    seg_qs, ee_path = segment_cartesian_move(robot, q, pre_pick, pick, steps=max(10, args.steps//2), obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path
    print("[ACTION] Close gripper (simulated)")

    print("[PHASE] Lift")
    seg_qs, ee_path = segment_cartesian_move(robot, q, pick, pre_pick, steps=max(10, args.steps//2), obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path

    print("[PHASE] Move to pre-place")
    seg_qs, ee_path = segment_cartesian_move(robot, q, pre_pick, pre_place, steps=args.steps, obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path

    print("[PHASE] Lower to place")
    seg_qs, ee_path = segment_cartesian_move(robot, q, pre_place, place, steps=max(10, args.steps//2), obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path
    print("[ACTION] Open gripper (simulated)")

    print("[PHASE] Retreat")
    seg_qs, ee_path = segment_cartesian_move(robot, q, place, pre_place, steps=max(10, args.steps//2), obstacles=obstacles)
    q = seg_qs[-1]; q_list_all += seg_qs[1:]; ee_all += ee_path

    if args.visualize:
        visualize_robot_path(robot, q_list_all, obstacles=obstacles, ee_path=ee_all, show=True, title="Pick & Place")
    return 0

if __name__ == "__main__":
    sys.exit(main())

