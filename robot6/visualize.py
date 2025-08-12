import numpy as np
from .kinematics import Pose

def _plot_sphere(ax, center, radius, resolution=16):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))  # đúng: sin(v), không phải cos(v)
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, alpha=0.2, linewidth=0)

def visualize_robot_path(robot, q_list, obstacles=None, ee_path=None, show=True, title="Robot 6-DOF Path"):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    pts_ee = []
    for q in q_list:
        _, Ts = robot.fk(q)
        xs, ys, zs = [], [], []
        for T in Ts:
            x, y, z = T[:3, 3]
            xs.append(x); ys.append(y); zs.append(z)
        ax.plot(xs, ys, zs, marker="o")
        pts_ee.append(Ts[-1][:3, 3])
    pts_ee = np.array(pts_ee)

    if ee_path is not None:
        ee_pts = np.array([p.p for p in ee_path])
        ax.plot(ee_pts[:, 0], ee_pts[:, 1], ee_pts[:, 2], linestyle="--")

    if obstacles:
        for (c, r) in obstacles:
            _plot_sphere(ax, np.array(c), r)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    ax.set_box_aspect([1, 1, 1])
    if show:
        plt.show()
    return fig, ax

