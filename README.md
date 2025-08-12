# 6-Axis Robot Arm Cartesian Control


This project provides a Python-based solution for controlling a 6-axis stationary robot arm. It implements forward and inverse kinematics, smooth Cartesian trajectory generation, visualization, simple obstacle avoidance, and a complete pick-and-place scenario, all without relying on ROS.

---

<details>
<summary><strong>🇻🇳 Phiên bản Tiếng Việt</strong></summary>

Dự án này cung cấp một giải pháp dựa trên Python để điều khiển một cánh tay robot 6 trục tĩnh. Dự án cài đặt các chức năng động học thuận và nghịch, sinh quỹ đạo mượt trong không gian Cartesian, trực quan hóa, tránh vật cản đơn giản, và một kịch bản gắp-và-đặt hoàn chỉnh mà không phụ thuộc vào ROS.

</details>

---

## Key Features

-   **Forward & Inverse Kinematics:** A robust numerical IK solver based on the Jacobian matrix (Damped Least Squares).
-   **Smooth Cartesian Trajectory Generation:** Linear interpolation for position and Spherical Linear Interpolation (SLERP) for orientation to ensure smooth, predictable end-effector paths.
-   **3D Visualization:** Uses Matplotlib to visualize the robot's configurations and the end-effector's path in 3D space.
-   **Reactive Obstacle Avoidance:** A simple but effective algorithm to detect collisions with spherical obstacles and attempt to navigate around them by adjusting the path.
-   **Pick-and-Place Scenario:** A complete, multi-segment trajectory plan for a pick-and-place task, including approach, grasp, lift, and retreat phases.
-   **Unit Tested:** Core functionalities like kinematics are verified with `pytest` to ensure correctness and reliability.

## Project Structure

```
robot_6axis_assignment/
├── .venv/
├── robot6/
│   ├── __init__.py
│   ├── kinematics.py       # Core FK, IK, Pose, and rotation math
│   ├── trajectory.py       # Cartesian path interpolation
│   ├── robot.py            # Robot model definition (DH parameters)
│   ├── visualize.py        # 3D Matplotlib visualization
│   ├── obstacles.py        # Collision detection logic
│   ├── main.py             # Main script for single Cartesian moves
│   └── pick_place.py       # Script for the pick & place scenario
├── tests/
│   ├── test_kinematics.py
│   └── test_pickplace.py
├── requirements.txt
└── README.md
```

## Setup and Installation

### Prerequisites
-   Ubuntu 22.04 or a compatible Linux distribution.
-   Python 3.10+

### Installation Steps

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Tranthanhbao198/robot-6axis-cartesian.git
    cd robot-6axis-cartesian
    ```

2.  **Create and activate a Python virtual environment:**
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

3.  **Install the required dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

## How to Run

Ensure you have activated the virtual environment (`source .venv/bin/activate`) before running any commands.

### 1. Basic Cartesian Movement (`main.py`)

This script moves the robot's end-effector from its current pose to a specified target Cartesian pose.

**Usage:**
```bash
python -m robot6.main [options]
```

**Example:** Move the end-effector to `(x=0.4, y=0.1, z=0.3)` with a downward orientation, avoiding a specified obstacle, and visualize the result.
```bash
python -m robot6.main --x 0.4 --y 0.1 --z 0.3 --roll 180 --pitch 0 --yaw 0 --visualize --obstacles "[[[0.3, 0.0, 0.2], 0.1]]"
```

### 2. Pick and Place Scenario (`pick_place.py`)

This script executes a pre-defined, multi-segment trajectory to simulate picking an object from one location and placing it at another.

**Usage:**
```bash
python -m robot6.pick_place [options]
```

**Example:** Run the default pick-and-place scenario and visualize the entire path.
```bash
python -m robot6.pick_place --visualize
```

## How to Test

This project uses `pytest` for unit testing. The tests verify the correctness of the kinematics implementation.

To run the tests, execute the following command from the project's root directory:
```bash
pytest
```

---

## Design Decisions

As per the challenge requirements, this section documents the rationale behind key design choices.

### 1. Kinematics Solver: Numerical vs. Analytical IK

-   **Question:** Should the Inverse Kinematics (IK) be solved analytically (deriving closed-form equations) or numerically?
-   **Options:**
    1.  **Analytical IK:** Very fast and provides all possible solutions, but is extremely difficult to derive and is specific to one exact robot geometry. A small change in a link length would invalidate the entire solution.
    2.  **Numerical IK (Jacobian-based):** An iterative approach that works for any robot configuration defined by its forward kinematics. It's more flexible and easier to implement than a full analytical solution.
-   **Reasoning:** The **numerical Jacobian-based approach (Damped Least Squares)** was chosen for its flexibility and generality. It allows the robot's DH parameters in `robot.py` to be easily modified without needing to re-derive a complex mathematical solution. This makes the code more modular and extensible. The implemented version is robust, handling singularities and joint limits gracefully.

### 2. Trajectory Generation: Interpolation Method

-   **Question:** How should the intermediate points for smooth movement be generated?
-   **Options:**
    1.  **Joint Space Interpolation:** Linearly interpolate each of the 6 joint angles. This is simple and always avoids obstacles if the start and end points are collision-free, but the end-effector path is a complex curve and not intuitive.
    2.  **Cartesian Space Interpolation:** Linearly interpolate the XYZ position and spherically interpolate the orientation (using Quaternions/SLERP). This produces a straight-line path for the end-effector, which is predictable and desirable for tasks like approaching an object.
-   **Reasoning:** **Cartesian Space Interpolation** was chosen as it directly addresses the challenge's focus on Cartesian movement. The combination of linear interpolation for position and SLERP for orientation is the standard and correct way to generate smooth, straight-line motions for the end-effector in 3D space.

### 3. Obstacle Avoidance Strategy

-   **Question:** How should the robot avoid obstacles without using complex motion planners?
-   **Options:**
    1.  **Global Planners (e.g., RRT*, PRM):** Powerful algorithms that find a guaranteed collision-free path. However, they are complex to implement and computationally expensive.
    2.  **Potential Fields:** A method where the robot is "pushed" away from obstacles. Can get stuck in local minima.
    3.  **Simple Reactive Strategy:** Follow the straight-line path. If a future state is in collision, try a small, local deviation to get around it.
-   **Reasoning:** A **Simple Reactive Strategy** was implemented as it directly fits the spirit of the challenge, which focuses on Cartesian control rather than advanced motion planning. The chosen method checks for collisions at each step of the interpolated path. If a collision is detected, it attempts to find a valid solution by "bumping" the target pose slightly (e.g., moving up or sideways). This is a pragmatic and effective solution for simple, uncluttered environments. Its limitations (it cannot solve complex mazes) are documented and accepted for this specific task.
