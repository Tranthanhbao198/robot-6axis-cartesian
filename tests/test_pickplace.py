import numpy as np
import math
from robot6.robot import Simple6DOF, rad
from robot6.kinematics import Pose, rpy_to_matrix

def test_pick_place_poses_are_reachable():
    """Typical pick/place with downward tool (pitch=pi) are reachable."""
    robot = Simple6DOF()

    q_guess = np.array([rad(0), rad(-90), rad(90), rad(-90), rad(-90), rad(0)], dtype=float)

    # downward-facing: pitch = 180 deg
    downward_orientation = rpy_to_matrix(0.0, math.pi, 0.0)

    pick_pose  = Pose(np.array([0.45, 0.10, 0.20]), downward_orientation)
    place_pose = Pose(np.array([0.10, 0.45, 0.20]), downward_orientation)

    q_pick, ok_pick = robot.ik(pick_pose, q_guess)
    assert ok_pick
    T_pick, _ = robot.fk(q_pick)
    assert np.linalg.norm(T_pick[:3, 3] - pick_pose.p) < 5e-3

    q_place, ok_place = robot.ik(place_pose, q_pick)
    assert ok_place
    T_place, _ = robot.fk(q_place)
    assert np.linalg.norm(T_place[:3, 3] - place_pose.p) < 5e-3

