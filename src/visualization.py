import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


F16_SURFACES = [
    (
        "nose",
        "#d7dde8",
        np.array([
            [2.35, 0.00, 0.00],
            [1.35, -0.16, -0.08],
            [1.35, 0.16, -0.08],
        ]),
    ),
    (
        "upper_fuselage",
        "#9ca3af",
        np.array([
            [1.35, -0.20, 0.06],
            [1.35, 0.20, 0.06],
            [-1.65, 0.16, 0.05],
            [-1.65, -0.16, 0.05],
        ]),
    ),
    (
        "lower_fuselage",
        "#64748b",
        np.array([
            [1.35, -0.17, -0.10],
            [-1.55, -0.13, -0.12],
            [-1.55, 0.13, -0.12],
            [1.35, 0.17, -0.10],
        ]),
    ),
    (
        "left_wing",
        "#475569",
        np.array([
            [0.58, 0.18, 0.00],
            [-0.62, 1.52, -0.02],
            [-1.02, 1.42, -0.03],
            [-0.25, 0.18, -0.01],
        ]),
    ),
    (
        "right_wing",
        "#475569",
        np.array([
            [0.58, -0.18, 0.00],
            [-0.25, -0.18, -0.01],
            [-1.02, -1.42, -0.03],
            [-0.62, -1.52, -0.02],
        ]),
    ),
    (
        "left_stabilator",
        "#334155",
        np.array([
            [-1.28, 0.13, 0.00],
            [-1.86, 0.82, -0.02],
            [-2.10, 0.72, -0.03],
            [-1.68, 0.12, -0.01],
        ]),
    ),
    (
        "right_stabilator",
        "#334155",
        np.array([
            [-1.28, -0.13, 0.00],
            [-1.68, -0.12, -0.01],
            [-2.10, -0.72, -0.03],
            [-1.86, -0.82, -0.02],
        ]),
    ),
    (
        "vertical_tail",
        "#1e293b",
        np.array([
            [-1.22, 0.00, 0.04],
            [-1.88, 0.00, 0.04],
            [-1.62, 0.00, 0.92],
        ]),
    ),
    (
        "canopy",
        "#0ea5e9",
        np.array([
            [0.78, -0.12, 0.08],
            [0.78, 0.12, 0.08],
            [0.10, 0.10, 0.25],
            [0.10, -0.10, 0.25],
        ]),
    ),
    (
        "inlet",
        "#0f172a",
        np.array([
            [0.72, -0.13, -0.12],
            [0.72, 0.13, -0.12],
            [0.12, 0.10, -0.26],
            [0.12, -0.10, -0.26],
        ]),
    ),
    (
        "nose_radome",
        "#111827",
        np.array([
            [2.35, 0.00, 0.00],
            [1.88, -0.08, -0.04],
            [1.88, 0.08, -0.04],
        ]),
    ),
    (
        "left_strake",
        "#64748b",
        np.array([
            [1.02, 0.15, 0.00],
            [0.22, 0.48, -0.02],
            [-0.25, 0.18, -0.01],
            [0.60, 0.15, 0.00],
        ]),
    ),
    (
        "right_strake",
        "#64748b",
        np.array([
            [1.02, -0.15, 0.00],
            [0.60, -0.15, 0.00],
            [-0.25, -0.18, -0.01],
            [0.22, -0.48, -0.02],
        ]),
    ),
    (
        "left_wingtip_rail",
        "#1e293b",
        np.array([
            [-0.55, 1.54, 0.00],
            [-1.06, 1.49, -0.01],
            [-1.08, 1.58, -0.01],
            [-0.58, 1.63, 0.00],
        ]),
    ),
    (
        "right_wingtip_rail",
        "#1e293b",
        np.array([
            [-0.55, -1.54, 0.00],
            [-0.58, -1.63, 0.00],
            [-1.08, -1.58, -0.01],
            [-1.06, -1.49, -0.01],
        ]),
    ),
    (
        "left_ventral_fin",
        "#334155",
        np.array([
            [-1.38, 0.10, -0.12],
            [-1.88, 0.15, -0.12],
            [-1.72, 0.18, -0.55],
        ]),
    ),
    (
        "right_ventral_fin",
        "#334155",
        np.array([
            [-1.38, -0.10, -0.12],
            [-1.72, -0.18, -0.55],
            [-1.88, -0.15, -0.12],
        ]),
    ),
    (
        "engine_nozzle",
        "#18181b",
        np.array([
            [-1.65, -0.14, 0.04],
            [-2.13, -0.11, 0.03],
            [-2.13, 0.11, 0.03],
            [-1.65, 0.14, 0.04],
        ]),
    ),
]


CONCEPT_FIGHTER_SURFACES = [
    (
        "nose",
        "#cbd5e1",
        np.array([
            [2.15, 0.00, 0.00],
            [1.18, -0.22, -0.06],
            [1.18, 0.22, -0.06],
        ]),
    ),
    (
        "diamond_fuselage",
        "#64748b",
        np.array([
            [1.18, -0.24, 0.06],
            [0.10, -0.34, 0.10],
            [-1.75, -0.20, 0.04],
            [-1.75, 0.20, 0.04],
            [0.10, 0.34, 0.10],
            [1.18, 0.24, 0.06],
        ]),
    ),
    (
        "left_delta_wing",
        "#334155",
        np.array([
            [0.52, 0.22, 0.00],
            [-1.00, 1.70, -0.03],
            [-1.58, 1.42, -0.04],
            [-0.30, 0.20, -0.01],
        ]),
    ),
    (
        "right_delta_wing",
        "#334155",
        np.array([
            [0.52, -0.22, 0.00],
            [-0.30, -0.20, -0.01],
            [-1.58, -1.42, -0.04],
            [-1.00, -1.70, -0.03],
        ]),
    ),
    (
        "left_canted_tail",
        "#0f172a",
        np.array([
            [-1.18, 0.30, 0.06],
            [-1.90, 0.62, 0.08],
            [-1.62, 0.42, 0.78],
        ]),
    ),
    (
        "right_canted_tail",
        "#0f172a",
        np.array([
            [-1.18, -0.30, 0.06],
            [-1.62, -0.42, 0.78],
            [-1.90, -0.62, 0.08],
        ]),
    ),
    (
        "canopy",
        "#38bdf8",
        np.array([
            [0.72, -0.14, 0.10],
            [0.72, 0.14, 0.10],
            [-0.12, 0.12, 0.28],
            [-0.12, -0.12, 0.28],
        ]),
    ),
]


NOSE_LINE = np.array([
    [2.35, 0.0, 0.0],
    [1.35, 0.0, 0.0],
    [-2.10, 0.0, 0.0],
])

LEFT_WINGTIP = np.array([[-0.76, 1.52, -0.02]])
RIGHT_WINGTIP = np.array([[-0.76, -1.52, -0.02]])
TAILPIPE = np.array([[-2.10, 0.0, -0.03]])

AIRCRAFT_MODELS = {
    "f16c": {
        "title": "F-16C-Inspired Lateral-Directional Response",
        "surfaces": F16_SURFACES,
        "nose_line": NOSE_LINE,
        "left_wingtip": LEFT_WINGTIP,
        "right_wingtip": RIGHT_WINGTIP,
        "tailpipe": TAILPIPE,
    },
    "concept": {
        "title": "Concept Fighter Lateral-Directional Response",
        "surfaces": CONCEPT_FIGHTER_SURFACES,
        "nose_line": np.array([[2.15, 0.0, 0.0], [1.18, 0.0, 0.0], [-1.95, 0.0, 0.0]]),
        "left_wingtip": np.array([[-1.05, 1.70, -0.03]]),
        "right_wingtip": np.array([[-1.05, -1.70, -0.03]]),
        "tailpipe": np.array([[-1.95, 0.0, -0.02]]),
    },
}


def _rotation_matrix(roll: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(yaw), np.sin(yaw)

    r_roll = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])
    r_yaw = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])
    return r_yaw @ r_roll


def _transform(points: np.ndarray, roll: float, yaw: float, position: np.ndarray) -> np.ndarray:
    return points @ _rotation_matrix(roll, yaw).T + position


def create_aircraft_animation(
    history: dict,
    out_path: str = "outputs/f16_attitude_3d.gif",
    fps: int = 30,
    max_frames: int = 240,
    aircraft_model: str = "f16c",
) -> str:
    """Create an F-16-inspired 3D attitude animation from simulation history."""
    if aircraft_model not in AIRCRAFT_MODELS:
        supported = ", ".join(sorted(AIRCRAFT_MODELS))
        raise ValueError(f"aircraft_model must be one of: {supported}")

    model = AIRCRAFT_MODELS[aircraft_model]
    surfaces = model["surfaces"]
    nose_line_points = model["nose_line"]
    left_wingtip = model["left_wingtip"]
    right_wingtip = model["right_wingtip"]
    tailpipe_point = model["tailpipe"]

    time_s = np.asarray(history["time_s"], dtype=float)
    phi = np.asarray(history["phi_rad"], dtype=float)
    r = np.asarray(history["r_rad_s"], dtype=float)

    if time_s.ndim != 1 or phi.shape != time_s.shape or r.shape != time_s.shape:
        raise ValueError("history must contain aligned time_s, phi_rad, and r_rad_s arrays")
    if len(time_s) < 2:
        raise ValueError("history must contain at least two samples")

    dt = np.gradient(time_s)
    yaw = np.cumsum(r * dt)
    speed = 0.55
    x_path = np.cumsum(np.cos(yaw) * speed * dt)
    y_path = np.cumsum(np.sin(yaw) * speed * dt)
    z_path = 0.16 * np.sin(0.35 * time_s) + 0.06 * np.sin(phi)
    stride = max(1, int(np.ceil(len(time_s) / max_frames)))
    frames = np.arange(0, len(time_s), stride)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    fig = plt.figure(figsize=(8, 6), facecolor="#f8fafc")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(model["title"], pad=12)
    ax.set_xlim(float(np.min(x_path)) - 2.8, float(np.max(x_path)) + 2.8)
    ax.set_ylim(float(np.min(y_path)) - 3.0, float(np.max(y_path)) + 3.0)
    ax.set_zlim(-1.1, 1.4)
    ax.set_xlabel("Downrange")
    ax.set_ylabel("Crossrange")
    ax.set_zlabel("Relative altitude")
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.25)
    ax.xaxis.pane.set_facecolor((0.93, 0.97, 1.0, 0.35))
    ax.yaxis.pane.set_facecolor((0.93, 0.97, 1.0, 0.35))
    ax.zaxis.pane.set_facecolor((0.94, 0.96, 0.94, 0.35))

    ground_x = np.linspace(float(np.min(x_path)) - 2.0, float(np.max(x_path)) + 2.0, 8)
    for y_grid in np.linspace(float(np.min(y_path)) - 2.5, float(np.max(y_path)) + 2.5, 6):
        ax.plot(ground_x, np.full_like(ground_x, y_grid), np.full_like(ground_x, -0.95),
                color="#cbd5e1", linewidth=0.5, alpha=0.55)

    trail_line, = ax.plot([], [], [], color="#2563eb", linewidth=2.2, alpha=0.85)
    nose_line, = ax.plot([], [], [], color="#0f172a", linewidth=1.8)
    left_vapor, = ax.plot([], [], [], color="#93c5fd", linewidth=1.2, alpha=0.55)
    right_vapor, = ax.plot([], [], [], color="#93c5fd", linewidth=1.2, alpha=0.55)
    afterburner, = ax.plot([], [], [], color="#f97316", linewidth=4.0, alpha=0.85)
    surface_artists = []
    for _, color, _ in surfaces:
        artist = Poly3DCollection(
            [np.zeros((3, 3))],
            facecolor=color,
            edgecolor="#020617",
            linewidth=0.45,
            alpha=0.96,
        )
        ax.add_collection3d(artist)
        surface_artists.append(artist)

    time_text = ax.text2D(0.03, 0.94, "", transform=ax.transAxes, color="#0f172a")
    note_text = ax.text2D(
        0.03,
        0.03,
        "State-driven visualization from simplified lateral-directional model",
        transform=ax.transAxes,
        color="#475569",
        fontsize=8,
    )

    def update(frame_index: int):
        i = int(frames[frame_index])
        roll = float(phi[i])
        yaw_i = float(yaw[i])
        position = np.array([x_path[i], y_path[i], z_path[i]])

        nose = _transform(nose_line_points, roll, yaw_i, position)
        nose_line.set_data(nose[:, 0], nose[:, 1])
        nose_line.set_3d_properties(nose[:, 2])

        for artist, (_, _, points) in zip(surface_artists, surfaces):
            artist.set_verts([_transform(points, roll, yaw_i, position)])

        left_tip = _transform(left_wingtip, roll, yaw_i, position)[0]
        right_tip = _transform(right_wingtip, roll, yaw_i, position)[0]
        tailpipe = _transform(tailpipe_point, roll, yaw_i, position)[0]
        exhaust_direction = _rotation_matrix(roll, yaw_i) @ np.array([-1.0, 0.0, -0.04])
        flame_scale = 0.52 + 0.18 * np.sin(7.0 * time_s[i])
        flame_end = tailpipe + exhaust_direction * flame_scale

        vapor_start = max(0, i - int(0.11 * len(time_s)))
        vapor_decay = np.linspace(0.0, 1.0, i - vapor_start + 1)
        left_vapor_x = x_path[vapor_start:i + 1] + (left_tip[0] - position[0]) * (1.0 - 0.18 * vapor_decay)
        left_vapor_y = y_path[vapor_start:i + 1] + (left_tip[1] - position[1]) * (1.0 - 0.18 * vapor_decay)
        left_vapor_z = z_path[vapor_start:i + 1] + (left_tip[2] - position[2]) * (1.0 - 0.18 * vapor_decay)
        right_vapor_x = x_path[vapor_start:i + 1] + (right_tip[0] - position[0]) * (1.0 - 0.18 * vapor_decay)
        right_vapor_y = y_path[vapor_start:i + 1] + (right_tip[1] - position[1]) * (1.0 - 0.18 * vapor_decay)
        right_vapor_z = z_path[vapor_start:i + 1] + (right_tip[2] - position[2]) * (1.0 - 0.18 * vapor_decay)

        left_vapor.set_data(left_vapor_x, left_vapor_y)
        left_vapor.set_3d_properties(left_vapor_z)
        right_vapor.set_data(right_vapor_x, right_vapor_y)
        right_vapor.set_3d_properties(right_vapor_z)
        afterburner.set_data([tailpipe[0], flame_end[0]], [tailpipe[1], flame_end[1]])
        afterburner.set_3d_properties([tailpipe[2], flame_end[2]])

        start = max(0, i - int(0.30 * len(time_s)))
        trail_line.set_data(x_path[start:i + 1], y_path[start:i + 1])
        trail_line.set_3d_properties(z_path[start:i + 1])
        time_text.set_text(
            f"t = {time_s[i]:.1f} s | bank = {np.degrees(roll):.1f} deg | yaw rate = {np.degrees(r[i]):.1f} deg/s"
        )
        return [
            trail_line,
            nose_line,
            left_vapor,
            right_vapor,
            afterburner,
            time_text,
            note_text,
            *surface_artists,
        ]

    animation = FuncAnimation(fig, update, frames=len(frames), interval=1000 / fps, blit=False)
    animation.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    return os.path.abspath(out_path)
