import math
import numpy as np

# -------------------------------
# Pose utilities
# -------------------------------
class Pose:
    def __init__(self, position: np.ndarray, rotation: np.ndarray):
        self.p = np.asarray(position, dtype=float).reshape(3)
        self.R = np.asarray(rotation, dtype=float).reshape(3, 3)

    @staticmethod
    def from_xyz_rpy(x, y, z, roll, pitch, yaw):
        return Pose(np.array([x, y, z]), rpy_to_matrix(roll, pitch, yaw))

    def as_matrix(self):
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.p
        return T


def rpy_to_matrix(roll, pitch, yaw):
    # Z-Y-X (yaw-pitch-roll): R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    return R


def matrix_to_rpy(R):
    # Inverse Z-Y-X
    sy = -R[2, 0]
    cy = math.sqrt(max(1.0 - sy * sy, 0.0))
    if cy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(sy, cy)
        yaw = 0.0
    return roll, pitch, yaw


def quat_from_matrix(R):
    m = R
    t = np.trace(m)
    if t > 0:
        r = math.sqrt(1.0 + t)
        w = 0.5 * r
        r = 0.5 / r
        x = (m[2, 1] - m[1, 2]) * r
        y = (m[0, 2] - m[2, 0]) * r
        z = (m[1, 0] - m[0, 1]) * r
    else:
        i = np.argmax([m[0, 0], m[1, 1], m[2, 2]])
        if i == 0:
            r = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            x = 0.5 * r
            r = 0.5 / r
            y = (m[0, 1] + m[1, 0]) * r
            z = (m[0, 2] + m[2, 0]) * r
            w = (m[2, 1] - m[1, 2]) * r
        elif i == 1:
            r = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            y = 0.5 * r
            r = 0.5 / r
            x = (m[0, 1] + m[1, 0]) * r
            z = (m[1, 2] + m[2, 1]) * r
            w = (m[0, 2] - m[2, 0]) * r
        else:
            r = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            z = 0.5 * r
            r = 0.5 / r
            x = (m[0, 2] + m[2, 0]) * r
            y = (m[1, 2] + m[2, 1]) * r
            w = (m[1, 0] - m[0, 1]) * r
    return np.array([w, x, y, z])


def quat_to_matrix(q):
    w, x, y, z = q
    Nq = np.dot(q, q)
    if Nq < 1e-12:
        return np.eye(3)
    s = 2.0 / Nq
    X = x * s
    Y = y * s
    Z = z * s
    wX = w * X
    wY = w * Y
    wZ = w * Z
    xX = x * X
    xY = x * Y
    xZ = x * Z
    yY = y * Y
    yZ = y * Z
    zZ = z * Z
    R = np.array(
        [
            [1.0 - (yY + zZ), xY - wZ, xZ + wY],
            [xY + wZ, 1.0 - (xX + zZ), yZ - wX],
            [xZ - wY, yZ + wX, 1.0 - (xX + yY)],
        ]
    )
    return R


def quat_slerp(q0, q1, t):
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0) + (s1 * q1)


def hat(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def log_SO3(R):
    # ([omega]^)=R
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(min(cos_theta, 1.0), -1.0)
    theta = math.acos(cos_theta)
    if abs(theta) < 1e-8:
        return np.zeros(3)
    return (
        theta
        / (2 * math.sin(theta))
        * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    )

# -------------------------------
# Standard DH kinematics
# -------------------------------
def dh_transform(a, alpha, d, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    T = np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ]
    )
    return T


class DHRobot:
    def __init__(self, a, alpha, d, theta_offset, q_min, q_max, name="DH6"):
        assert len(a) == len(alpha) == len(d) == len(theta_offset) == len(q_min) == len(q_max)
        self.n = len(a)
        self.a = np.array(a, dtype=float)
        self.alpha = np.array(alpha, dtype=float)
        self.d = np.array(d, dtype=float)
        self.theta_offset = np.array(theta_offset, dtype=float)
        self.q_min = np.array(q_min, dtype=float)
        self.q_max = np.array(q_max, dtype=float)
        self.name = name

    def fk(self, q):
        T = np.eye(4)
        Ts = [T.copy()]
        for i in range(self.n):
            T_i = dh_transform(self.a[i], self.alpha[i], self.d[i], q[i] + self.theta_offset[i])
            T = T @ T_i
            Ts.append(T.copy())
        return T, Ts

    def jacobian(self, q):
        _, Ts = self.fk(q)
        o_n = Ts[-1][:3, 3]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            z = Ts[i][:3, 2]
            o_i = Ts[i][:3, 3]
            J[:3, i] = np.cross(z, (o_n - o_i))
            J[3:, i] = z
        return J

    def within_limits(self, q):
        return np.all(q >= self.q_min - 1e-9) and np.all(q <= self.q_max + 1e-9)

    def clamp_to_limits(self, q):
        return np.minimum(np.maximum(q, self.q_min), self.q_max)

    def ik(
        self,
        target_pose: Pose,
        q0,
        tol_pos=1e-3,      # 1 mm
        tol_rot=2e-3,
        max_iters=800,
        damping=5e-3,
        step_scale=0.7,
        wR=0.2,            # ưu tiên vị trí
        fallback=True,
    ):

        def solve_once(q_init, wR_local, iters=max_iters, lam=damping, step=step_scale,
                       stop_pos=tol_pos, stop_rot=tol_rot):
            q = np.array(q_init, dtype=float).copy()
            for _ in range(iters):
                T, _ = self.fk(q)
                p = T[:3, 3]
                R = T[:3, :3]
                e_p = target_pose.p - p
                R_err = R.T @ target_pose.R
                e_o = log_SO3(R_err)

                if np.linalg.norm(e_p) < stop_pos and (np.linalg.norm(e_o) < stop_rot or wR_local == 0.0):
                    return self.clamp_to_limits(q), True

                J = self.jacobian(q)
                Jp = J[:3, :]
                Jo = J[3:, :]
                Jw = np.vstack([Jp, wR_local * Jo])
                errw = np.concatenate([e_p, wR_local * e_o])

                JT = Jw.T
                H = Jw @ JT + (lam**2) * np.eye(6)
                dq = JT @ np.linalg.solve(H, errw)

                n = np.linalg.norm(dq)
                if n > 0.25:
                    dq *= 0.25 / n

                q = q + step * dq
                q = self.clamp_to_limits(q)


            T, _ = self.fk(q)
            p = T[:3, 3]
            R = T[:3, :3]
            e_p = target_pose.p - p
            R_err = R.T @ target_pose.R
            e_o = log_SO3(R_err)
            ok = np.linalg.norm(e_p) < stop_pos and (np.linalg.norm(e_o) < stop_rot or wR_local == 0.0)
            return self.clamp_to_limits(q), ok

        def polish_position(q_init, stop_pos=8e-4):

            q = np.array(q_init, dtype=float).copy()
            lam = 1e-3
            step = 0.5
            for _ in range(300):
                T, _ = self.fk(q)
                p = T[:3, 3]
                e_p = target_pose.p - p
                if np.linalg.norm(e_p) < stop_pos:
                    return self.clamp_to_limits(q)
                J = self.jacobian(q)
                Jp = J[:3, :]                 # 3x6
                JT = Jp.T                     # 6x3
                H = Jp @ JT + (lam**2) * np.eye(3)   # 3x3
                dq = JT @ np.linalg.solve(H, e_p)    # 6x1

                n = np.linalg.norm(dq)
                if n > 0.2:
                    dq *= 0.2 / n
                q = self.clamp_to_limits(q + step * dq)
            return self.clamp_to_limits(q)

   
        q_sol, ok = solve_once(q0, wR)
        wR_used = wR  # mặc định

        if not ok and fallback:
            # Fallback: vị trí-only, thử vài seed
            seeds = [
                q0,
                self.clamp_to_limits(q0 + np.array([0, 0, 0, 0, math.pi, 0])),  # wrist flip
                self.clamp_to_limits(q0 + np.array([0, -0.3, +0.3, 0, 0, 0])),  # elbow tweak
                self.clamp_to_limits(q0 + np.array([0, +0.3, -0.3, 0, 0, 0])),
            ]
            for s in seeds:
                q_try, ok2 = solve_once(s, 0.0)
                if ok2:
                    q_sol, ok = q_try, True
                    wR_used = 0.0
                    break


        q_sol = polish_position(q_sol)

        # Đánh giá cuối theo wR_used thực sự
        T_final, _ = self.fk(q_sol)
        pos_err = np.linalg.norm(T_final[:3, 3] - target_pose.p)
        rot_err = np.linalg.norm(log_SO3(T_final[:3, :3].T @ target_pose.R))
        final_ok = (pos_err < tol_pos) and (rot_err < tol_rot or wR_used == 0.0)

        return q_sol, final_ok

