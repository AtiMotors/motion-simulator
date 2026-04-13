"""
Unicycle 3-segment turn simulator

Features
- Unicycle kinematics
- Constant speed magnitude
- No pure rotation
- 3 parameterized segments:
    1) Forward + Left  by alpha degrees over distance d1
    2) Reverse + Right by beta  degrees over distance d2
    3) Forward + Left  by gamma degrees over distance d3
- Robot body shown as a rectangle
- ROBOT_WIDTH and ROBOT_LENGTH read from config.toml
- Animation included

config.toml example
-------------------
ROBOT_WIDTH = 0.8
ROBOT_LENGTH = 1.2

alpha = 90
beta = 1
gamma = 90

d1 = 1.25
d2 = 1.75
d3 = 1.25

v = 1.0
dt = 0.02
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # pip install tomli


DEFAULT_CONFIG = {
    "ROBOT_WIDTH": 0.8,
    "ROBOT_LENGTH": 1.2,
    "alpha": 90.0,
    "beta": 1.0,
    "gamma": 90.0,
    "d1": 1.25,
    "d2": 1.75,
    "d3": 1.25,
    "v": 1.0,
    "dt": 0.02,
}


def load_config(path: str = "config.toml") -> dict:
    cfg = DEFAULT_CONFIG.copy()
    p = Path(path)
    if p.exists():
        with open(p, "rb") as f:
            user_cfg = tomllib.load(f)
        cfg.update(user_cfg)
    return cfg


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Unicycle:
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def state(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta], dtype=float)

    def step(self, v: float, omega: float, dt: float) -> None:
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = wrap_angle(self.theta + omega * dt)


def segment_controls(
    distance: float,
    angle_deg: float,
    v_mag: float,
    motion: str,
    turn: str,
) -> tuple[float, float, float]:
    """
    Returns constant (v, omega, duration) for a segment.

    distance : path length magnitude
    angle_deg: heading change magnitude in degrees
    motion   : "forward" or "reverse"
    turn     : "left" or "right"
    """
    if distance <= 0:
        raise ValueError("distance must be positive")
    if v_mag <= 0:
        raise ValueError("v must be positive")

    v = v_mag if motion == "forward" else -v_mag
    duration = distance / abs(v)

    angle_rad = math.radians(abs(angle_deg))
    if angle_rad == 0:
        omega = 0.0
    else:
        omega_mag = angle_rad / duration
        omega = omega_mag if turn == "left" else -omega_mag

    return v, omega, duration


def simulate_segment(
    robot: Unicycle,
    v: float,
    omega: float,
    duration: float,
    dt: float,
) -> np.ndarray:
    steps = max(1, int(math.ceil(duration / dt)))
    traj = [robot.state().copy()]

    for _ in range(steps):
        robot.step(v, omega, dt)
        traj.append(robot.state().copy())

    return np.array(traj)


def rectangle_corners(
    x: float,
    y: float,
    theta: float,
    length: float,
    width: float,
) -> np.ndarray:
    """
    Rectangle centered at (x, y), oriented by theta.
    Length is along heading direction.
    """
    hl = length / 2.0
    hw = width / 2.0

    local = np.array(
        [
            [hl,  hw],
            [hl, -hw],
            [-hl, -hw],
            [-hl,  hw],
        ],
        dtype=float,
    )

    c = math.cos(theta)
    s = math.sin(theta)
    rot = np.array([[c, -s], [s, c]], dtype=float)

    world = local @ rot.T
    world[:, 0] += x
    world[:, 1] += y
    return world


def heading_marker(
    x: float,
    y: float,
    theta: float,
    length: float,
) -> np.ndarray:
    tip_x = x + 0.5 * length * math.cos(theta)
    tip_y = y + 0.5 * length * math.sin(theta)
    return np.array([[x, y], [tip_x, tip_y]], dtype=float)


def main() -> None:
    cfg = load_config("config.toml")

    robot_width = float(cfg["ROBOT_WIDTH"])
    robot_length = float(cfg["ROBOT_LENGTH"])

    alpha = float(cfg["alpha"])
    beta = float(cfg["beta"])
    gamma = float(cfg["gamma"])

    d1 = float(cfg["d1"])
    d2 = float(cfg["d2"])
    d3 = float(cfg["d3"])

    v_mag = float(cfg["v"])
    dt = float(cfg["dt"])

    robot = Unicycle(0.0, 0.0, 0.0)

    specs = [
        ("forward", "left",  d1, alpha, "Segment 1: Forward + Left"),
        ("reverse", "left", d2, beta,  "Segment 2: Reverse + Right"),
        ("forward", "left",  d3, gamma, "Segment 3: Forward + Left"),
    ]

    segments = []
    full_traj_parts = []

    for idx, (motion, turn, distance, angle_deg, label) in enumerate(specs):
        v, omega, duration = segment_controls(
            distance=distance,
            angle_deg=angle_deg,
            v_mag=v_mag,
            motion=motion,
            turn=turn,
        )

        seg_traj = simulate_segment(robot, v, omega, duration, dt)
        segments.append(
            {
                "traj": seg_traj,
                "label": label,
                "v": v,
                "omega": omega,
                "duration": duration,
            }
        )

        if idx == 0:
            full_traj_parts.append(seg_traj)
        else:
            full_traj_parts.append(seg_traj[1:])

    traj = np.vstack(full_traj_parts)

    initial_heading_deg = math.degrees(traj[0, 2])
    final_heading_deg = math.degrees(traj[-1, 2])
    heading_change_deg = math.degrees(wrap_angle(traj[-1, 2] - traj[0, 2]))

    print("Loaded configuration")
    print("--------------------")
    print(f"ROBOT_WIDTH  = {robot_width}")
    print(f"ROBOT_LENGTH = {robot_length}")
    print(f"alpha = {alpha} deg, beta = {beta} deg, gamma = {gamma} deg")
    print(f"d1 = {d1}, d2 = {d2}, d3 = {d3}")
    print(f"v = {v_mag}, dt = {dt}")
    print()
    print(f"Initial heading = {initial_heading_deg:.2f} deg")
    print(f"Final heading   = {final_heading_deg:.2f} deg")
    print(f"Net heading change = {heading_change_deg:.2f} deg")
    print()

    for s in segments:
        print(
            f"{s['label']}: "
            f"v={s['v']:+.3f}, omega={s['omega']:+.3f}, duration={s['duration']:.3f}"
        )

    # Static plot
    fig1, ax1 = plt.subplots(figsize=(8, 8))
    colors = ["tab:blue", "tab:orange", "tab:green"]

    for s, color in zip(segments, colors):
        seg = s["traj"]
        ax1.plot(seg[:, 0], seg[:, 1], linewidth=2, color=color, label=s["label"])

    start_pose = traj[0]
    end_pose = traj[-1]

    start_body = Polygon(
        rectangle_corners(
            start_pose[0], start_pose[1], start_pose[2], robot_length, robot_width
        ),
        closed=True,
        fill=False,
        linewidth=2,
    )
    end_body = Polygon(
        rectangle_corners(
            end_pose[0], end_pose[1], end_pose[2], robot_length, robot_width
        ),
        closed=True,
        fill=False,
        linewidth=2,
    )

    ax1.add_patch(start_body)
    ax1.add_patch(end_body)

    start_head = heading_marker(start_pose[0], start_pose[1], start_pose[2], robot_length)
    end_head = heading_marker(end_pose[0], end_pose[1], end_pose[2], robot_length)
    ax1.plot(start_head[:, 0], start_head[:, 1], linewidth=2)
    ax1.plot(end_head[:, 0], end_head[:, 1], linewidth=2)

    ax1.scatter([start_pose[0]], [start_pose[1]], label="Start")
    ax1.scatter([end_pose[0]], [end_pose[1]], label="End")

    ax1.set_title(f"aisle width = {d2}")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.axis("equal")
    ax1.grid(True)
    ax1.legend()

    pose_text = (
        f"Initial pose: x={start_pose[0]:.1f}, y={start_pose[1]:.1f}, "
        f"theta={math.degrees(start_pose[2]):.1f} deg\n"
        f"Final pose:   x={end_pose[0]:.1f}, y={end_pose[1]:.1f}, "
        f"theta={math.degrees(end_pose[2]):.1f} deg"
    )
    ax1.text(
        0.02,
        0.02,
        pose_text,
        transform=ax1.transAxes,
        va="bottom",
        ha="left",
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
    )

    # Animated plot
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    ax2.set_title("Animated Unicycle 3-Segment Turn")
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.axis("equal")
    ax2.grid(True)

    pad = max(robot_length, robot_width, 0.5)
    ax2.set_xlim(np.min(traj[:, 0]) - pad, np.max(traj[:, 0]) + pad)
    ax2.set_ylim(np.min(traj[:, 1]) - pad, np.max(traj[:, 1]) + pad)

    path_line, = ax2.plot([], [], linewidth=2)
    body_patch = Polygon(
        rectangle_corners(
            traj[0, 0], traj[0, 1], traj[0, 2], robot_length, robot_width
        ),
        closed=True,
        fill=False,
        linewidth=2,
    )
    ax2.add_patch(body_patch)

    heading_line, = ax2.plot([], [], linewidth=2)

    def init():
        path_line.set_data([], [])
        body_patch.set_xy(
            rectangle_corners(
                traj[0, 0], traj[0, 1], traj[0, 2], robot_length, robot_width
            )
        )
        hm = heading_marker(traj[0, 0], traj[0, 1], traj[0, 2], robot_length)
        heading_line.set_data(hm[:, 0], hm[:, 1])
        return path_line, body_patch, heading_line

    def update(frame: int):
        x = traj[: frame + 1, 0]
        y = traj[: frame + 1, 1]
        px, py, ptheta = traj[frame]

        path_line.set_data(x, y)
        body_patch.set_xy(
            rectangle_corners(px, py, ptheta, robot_length, robot_width)
        )

        hm = heading_marker(px, py, ptheta, robot_length)
        heading_line.set_data(hm[:, 0], hm[:, 1])

        return path_line, body_patch, heading_line

    ani = FuncAnimation(
        fig2,
        update,
        frames=len(traj),
        init_func=init,
        interval=max(1, int(dt * 1000)),
        blit=False,
        repeat=False,
    )

    # Keep a live reference so animation is not garbage collected.
    _ = ani

    plt.show()


if __name__ == "__main__":
    main()
