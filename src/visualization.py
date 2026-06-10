import json
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


def create_lateral_flight_3d_html(
    history: dict,
    out_path: str = "outputs/f16_lateral_flight_3d.html",
    max_samples: int = 900,
    aircraft_model: str = "f16c",
) -> str:
    """Create a self-contained browser animation of lateral F-16 flight."""
    if aircraft_model not in AIRCRAFT_MODELS:
        supported = ", ".join(sorted(AIRCRAFT_MODELS))
        raise ValueError(f"aircraft_model must be one of: {supported}")
    if max_samples < 2:
        raise ValueError("max_samples must be at least 2")

    time_s = np.asarray(history["time_s"], dtype=float)
    phi = np.asarray(history["phi_rad"], dtype=float)
    r = np.asarray(history["r_rad_s"], dtype=float)
    beta = np.asarray(history["beta_rad"], dtype=float)
    p = np.asarray(history["p_rad_s"], dtype=float)

    if (
        time_s.ndim != 1
        or phi.shape != time_s.shape
        or r.shape != time_s.shape
        or beta.shape != time_s.shape
        or p.shape != time_s.shape
    ):
        raise ValueError("history must contain aligned time_s, beta_rad, p_rad_s, r_rad_s, and phi_rad arrays")
    if len(time_s) < 2:
        raise ValueError("history must contain at least two samples")

    dt = np.gradient(time_s)
    yaw = np.cumsum(r * dt)
    speed = 1.0
    x_path = np.cumsum(np.cos(yaw + beta) * speed * dt)
    y_path = np.cumsum(np.sin(yaw + beta) * speed * dt)
    z_path = 0.22 * np.sin(0.35 * time_s) + 0.08 * np.sin(phi)

    stride = max(1, int(np.ceil(len(time_s) / max_samples)))
    sample_idx = np.arange(0, len(time_s), stride)

    model = AIRCRAFT_MODELS[aircraft_model]
    payload = {
        "title": model["title"],
        "surfaces": [
            {"name": name, "color": color, "points": points.tolist()}
            for name, color, points in model["surfaces"]
        ],
        "noseLine": model["nose_line"].tolist(),
        "leftWingtip": model["left_wingtip"].reshape(-1, 3)[0].tolist(),
        "rightWingtip": model["right_wingtip"].reshape(-1, 3)[0].tolist(),
        "tailpipe": model["tailpipe"].reshape(-1, 3)[0].tolist(),
        "time": time_s[sample_idx].round(5).tolist(),
        "roll": phi[sample_idx].round(7).tolist(),
        "yaw": yaw[sample_idx].round(7).tolist(),
        "rollRate": p[sample_idx].round(7).tolist(),
        "yawRate": r[sample_idx].round(7).tolist(),
        "x": x_path[sample_idx].round(6).tolist(),
        "y": y_path[sample_idx].round(6).tolist(),
        "z": z_path[sample_idx].round(6).tolist(),
    }

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    html = _build_lateral_flight_html(payload)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(html)
    return os.path.abspath(out_path)


def _build_lateral_flight_html(payload: dict) -> str:
    payload_json = json.dumps(payload, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>F-16 Lateral Flight 3D Animation</title>
<style>
html,body{{margin:0;width:100%;height:100%;background:#e5edf6;color:#0f172a;font-family:Arial,Helvetica,sans-serif;overflow:hidden}}
canvas{{display:block;width:100vw;height:100vh;background:linear-gradient(#dbeafe,#f8fafc 58%,#d9f99d 59%,#bef264)}}
.hud{{position:fixed;left:18px;top:16px;background:rgba(248,250,252,.82);border:1px solid rgba(15,23,42,.18);border-radius:8px;padding:12px 14px;box-shadow:0 12px 32px rgba(15,23,42,.16);backdrop-filter:blur(8px)}}
.hud h1{{font-size:15px;margin:0 0 8px;font-weight:700;letter-spacing:0}}
.grid{{display:grid;grid-template-columns:auto auto;gap:5px 14px;font-size:12px}}
.label{{color:#475569}}
.controls{{position:fixed;right:18px;bottom:16px;display:flex;gap:8px}}
button{{height:34px;border:1px solid rgba(15,23,42,.22);border-radius:8px;background:#f8fafc;color:#0f172a;font-weight:700;cursor:pointer;box-shadow:0 8px 20px rgba(15,23,42,.12)}}
button:hover{{background:#e0f2fe}}
.speed{{position:fixed;left:18px;bottom:16px;background:rgba(248,250,252,.82);border:1px solid rgba(15,23,42,.18);border-radius:8px;padding:10px 12px;box-shadow:0 8px 20px rgba(15,23,42,.12);display:grid;grid-template-columns:auto 150px auto;gap:10px;align-items:center;font-size:12px}}
input[type=range]{{accent-color:#2563eb}}
</style>
</head>
<body>
<canvas id="scene"></canvas>
<div class="hud">
  <h1>F-16 Lateral Flight</h1>
  <div class="grid">
    <div class="label">time</div><div id="time">0.0 s</div>
    <div class="label">bank</div><div id="bank">0.0 deg</div>
    <div class="label">roll rate</div><div id="rollRate">0.0 deg/s</div>
    <div class="label">yaw rate</div><div id="yawRate">0.0 deg/s</div>
  </div>
</div>
<div class="speed">
  <label for="speed">speed</label>
  <input id="speed" type="range" min="0.1" max="1.5" step="0.05" value="0.35">
  <span id="speedValue">0.35x</span>
</div>
<div class="controls">
  <button id="playPause" type="button">Pause</button>
  <button id="restart" type="button">Restart</button>
</div>
<script>
const DATA = {payload_json};
const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
const hud = {{
  time: document.getElementById("time"),
  bank: document.getElementById("bank"),
  rollRate: document.getElementById("rollRate"),
  yawRate: document.getElementById("yawRate")
}};
let width = 0, height = 0, dpr = 1;
let simTime = DATA.time[0], playing = true, lastStamp = 0, cameraOrbit = 0;
let playbackSpeed = 0.35;
const duration = DATA.time[DATA.time.length - 1] - DATA.time[0];

function resize() {{
  dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));
  width = window.innerWidth;
  height = window.innerHeight;
  canvas.width = Math.floor(width * dpr);
  canvas.height = Math.floor(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}}
window.addEventListener("resize", resize);
resize();

function rotateBody(point, roll, yaw) {{
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const x1 = point[0];
  const y1 = point[1] * cr - point[2] * sr;
  const z1 = point[1] * sr + point[2] * cr;
  return [x1 * cy - y1 * sy, x1 * sy + y1 * cy, z1];
}}

function worldToCamera(point, focus) {{
  const orbit = -0.72 + 0.08 * Math.sin(cameraOrbit);
  const elev = 0.46;
  const dx = point[0] - focus[0];
  const dy = point[1] - focus[1];
  const dz = point[2] - focus[2];
  const co = Math.cos(orbit), so = Math.sin(orbit);
  const ce = Math.cos(elev), se = Math.sin(elev);
  const x = dx * co - dy * so;
  const y = dx * so + dy * co;
  const z = dz;
  return [x, y * ce - z * se, y * se + z * ce + 16];
}}

function project(point, focus) {{
  const c = worldToCamera(point, focus);
  const scale = Math.min(width, height) * 0.78 / Math.max(4, c[2]);
  return [width * 0.52 + c[0] * scale, height * 0.52 - c[1] * scale, c[2]];
}}

function transformLocal(point, i) {{
  const local = rotateBody(point, i.roll, i.yaw);
  return [local[0] + i.x, local[1] + i.y, local[2] + i.z];
}}

function interpolateSample(t) {{
  if (t <= DATA.time[0]) return sampleAt(0, 0);
  if (t >= DATA.time[DATA.time.length - 1]) return sampleAt(DATA.time.length - 1, DATA.time.length - 1);
  let hi = 1;
  while (hi < DATA.time.length && DATA.time[hi] < t) hi++;
  const lo = hi - 1;
  const span = Math.max(1e-6, DATA.time[hi] - DATA.time[lo]);
  return sampleAt(lo, hi, (t - DATA.time[lo]) / span);
}}

function lerp(a, b, u) {{
  return a + (b - a) * u;
}}

function sampleAt(lo, hi, u = 0) {{
  return {{
    index: lo,
    time: lerp(DATA.time[lo], DATA.time[hi], u),
    roll: lerp(DATA.roll[lo], DATA.roll[hi], u),
    yaw: lerp(DATA.yaw[lo], DATA.yaw[hi], u),
    rollRate: lerp(DATA.rollRate[lo], DATA.rollRate[hi], u),
    yawRate: lerp(DATA.yawRate[lo], DATA.yawRate[hi], u),
    x: lerp(DATA.x[lo], DATA.x[hi], u),
    y: lerp(DATA.y[lo], DATA.y[hi], u),
    z: lerp(DATA.z[lo], DATA.z[hi], u)
  }};
}}

function drawPath(sample, focus) {{
  ctx.lineWidth = 3;
  ctx.strokeStyle = "rgba(37,99,235,.82)";
  ctx.beginPath();
  const i = sample.index;
  const start = Math.max(0, i - 220);
  for (let k = start; k <= i; k++) {{
    const p = project([DATA.x[k], DATA.y[k], DATA.z[k]], focus);
    if (k === start) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
  }}
  ctx.stroke();
}}

function drawGround(focus) {{
  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(71,85,105,.20)";
  for (let gx = -18; gx <= 18; gx += 3) {{
    ctx.beginPath();
    for (let gy = -14; gy <= 14; gy += 2) {{
      const p = project([focus[0] + gx, focus[1] + gy, -1.2], focus);
      if (gy === -14) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }}
    ctx.stroke();
  }}
  for (let gy = -14; gy <= 14; gy += 2) {{
    ctx.beginPath();
    for (let gx = -18; gx <= 18; gx += 3) {{
      const p = project([focus[0] + gx, focus[1] + gy, -1.2], focus);
      if (gx === -18) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
    }}
    ctx.stroke();
  }}
}}

function shade(color, depth) {{
  const alpha = Math.max(.70, Math.min(1, 1.12 - depth * .018));
  return color + Math.round(alpha * 255).toString(16).padStart(2, "0");
}}

function drawAircraft(sample, focus) {{
  const polys = DATA.surfaces.map(surface => {{
    const world = surface.points.map(point => transformLocal(point, sample));
    const screen = world.map(point => project(point, focus));
    const depth = screen.reduce((sum, point) => sum + point[2], 0) / screen.length;
    return {{screen, depth, color: surface.color}};
  }}).sort((a, b) => b.depth - a.depth);

  for (const poly of polys) {{
    ctx.beginPath();
    poly.screen.forEach((p, idx) => idx ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]));
    ctx.closePath();
    ctx.fillStyle = shade(poly.color, poly.depth);
    ctx.strokeStyle = "rgba(2,6,23,.72)";
    ctx.lineWidth = 1;
    ctx.fill();
    ctx.stroke();
  }}

  const nose = DATA.noseLine.map(point => project(transformLocal(point, sample), focus));
  ctx.strokeStyle = "#020617";
  ctx.lineWidth = 2;
  ctx.beginPath();
  nose.forEach((p, idx) => idx ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]));
  ctx.stroke();

  const tail = transformLocal(DATA.tailpipe, sample);
  const flameEnd = transformLocal([-2.78 - 0.15 * Math.sin(sample.time * 8.0), 0, -0.07], sample);
  const a = project(tail, focus), b = project(flameEnd, focus);
  ctx.strokeStyle = "rgba(249,115,22,.88)";
  ctx.lineWidth = 6;
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
}}

function draw(timestamp) {{
  if (!lastStamp) lastStamp = timestamp;
  const elapsed = timestamp - lastStamp;
  lastStamp = timestamp;
  if (playing) {{
    simTime += elapsed * 0.001 * playbackSpeed;
    if (simTime > DATA.time[DATA.time.length - 1]) simTime = DATA.time[0] + ((simTime - DATA.time[0]) % duration);
  }}
  cameraOrbit += elapsed * 0.00018;
  const sample = interpolateSample(simTime);
  const focus = [sample.x, sample.y, sample.z];

  ctx.clearRect(0, 0, width, height);
  drawGround(focus);
  drawPath(sample, focus);
  drawAircraft(sample, focus);

  const deg = 180 / Math.PI;
  hud.time.textContent = `${{sample.time.toFixed(1)}} s`;
  hud.bank.textContent = `${{(sample.roll * deg).toFixed(1)}} deg`;
  hud.rollRate.textContent = `${{(sample.rollRate * deg).toFixed(1)}} deg/s`;
  hud.yawRate.textContent = `${{(sample.yawRate * deg).toFixed(1)}} deg/s`;
  requestAnimationFrame(draw);
}}

document.getElementById("playPause").addEventListener("click", event => {{
  playing = !playing;
  event.currentTarget.textContent = playing ? "Pause" : "Play";
}});
document.getElementById("restart").addEventListener("click", () => {{
  simTime = DATA.time[0];
  playing = true;
  document.getElementById("playPause").textContent = "Pause";
}});
document.getElementById("speed").addEventListener("input", event => {{
  playbackSpeed = Number(event.currentTarget.value);
  document.getElementById("speedValue").textContent = `${{playbackSpeed.toFixed(2)}}x`;
}});
requestAnimationFrame(draw);
</script>
</body>
</html>
"""
