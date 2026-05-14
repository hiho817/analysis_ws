"""
visualize_vicon_3d.py — Interactive 3D VICON marker visualizer with time slider.

Markers
-------
  O1–O4 (orange) : body hip markers, all pairs connected
  G1–G4 (orange) : foot markers, G1→O1, G2→O2, G3→O3, G4→O4
  Tigger (orange) : trigger marker → connected to all O when visible
  Ground1–Ground4 (blue) : ground plane markers, all pairs connected

Output
------
  vicon_3d_viz.html  — standalone Plotly animation with time slider
"""

import sys
import os
import numpy as np
import pandas as pd
import plotly.graph_objects as go

# ─── CSV path ─────────────────────────────────────────────────────────────────
CSV_PATH = os.path.join(os.path.dirname(__file__),
                        "../vicon/walk_2m_01.csv")
OUTPUT_HTML = os.path.join(os.path.dirname(__file__), "vicon_3d_viz.html")

# Subsample: every N-th trajectory frame (500Hz → effective fps = 500/STEP)
STEP = 10   # 50 Hz effective → smooth enough, ~2500 frames for a 50s recording

# ─── Parsing helpers ──────────────────────────────────────────────────────────

def find_section_row(filepath, section_name):
    with open(filepath, "r") as f:
        for i, line in enumerate(f):
            if line.strip().startswith(section_name):
                return i
    raise ValueError(f"Section '{section_name}' not found")


def build_marker_col_map(csv_path, traj_section_row):
    raw = pd.read_csv(csv_path, skiprows=traj_section_row + 2,
                      nrows=1, header=None, sep="\t").iloc[0].tolist()
    marker_map = {}
    col = 2
    while col < len(raw):
        name = str(raw[col]).strip()
        if pd.notna(raw[col]) and name and name != "nan":
            short = name.split(":")[-1]
            marker_map[short] = [col, col + 1, col + 2]
        col += 3
    return marker_map


# ─── Load trajectories ────────────────────────────────────────────────────────
print("Loading CSV …")
traj_row = find_section_row(CSV_PATH, "Trajectories")
traj_df  = pd.read_csv(CSV_PATH, skiprows=traj_row + 5,
                       header=None, sep="\t", low_memory=False)
mcm      = build_marker_col_map(CSV_PATH, traj_row)
print(f"  Markers found: {list(mcm.keys())}")
print(f"  Total frames : {len(traj_df)}")

fs = 500.0  # Hz

def get_xyz(name):
    cols = mcm[name]
    return traj_df[cols].values.astype(float)

# ─── Ground-plane alignment ───────────────────────────────────────────────────
gpts = []
for m in ["Ground1", "Ground2", "Ground3", "Ground4"]:
    xyz = get_xyz(m)
    v = ~np.isnan(xyz).any(axis=1)
    if v.any():
        gpts.append(xyz[v][0])
gpts = np.array(gpts)
centroid_g = gpts.mean(axis=0)
_, _, Vt = np.linalg.svd(gpts - centroid_g)
n_ground = Vt[-1]
if n_ground[2] < 0:
    n_ground = -n_ground

def _rot_align_z(n):
    n = n / np.linalg.norm(n)
    z = np.array([0., 0., 1.])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    c = np.dot(n, z)
    if s < 1e-10:
        return np.eye(3)
    Vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3) + Vx + Vx @ Vx * ((1 - c) / s ** 2)

R_ground = _rot_align_z(n_ground)

def to_world(p_mm):
    return (R_ground @ (p_mm - centroid_g).T).T

# ─── Robot-centric heading alignment ─────────────────────────────────────────
O1w = to_world(get_xyz("O1"))
O4w = to_world(get_xyz("O4"))
tigger_raw   = get_xyz("Tigger")
tigger_valid = ~np.isnan(tigger_raw).any(axis=1)
frame_trig   = int(np.where(tigger_valid)[0][0])

# Use trigger frame as reference; fall back if hip markers missing
O2w = to_world(get_xyz("O2"))
O3w = to_world(get_xyz("O3"))
valid_hip = ~(np.isnan(O1w).any(1) | np.isnan(O2w).any(1) |
              np.isnan(O3w).any(1) | np.isnan(O4w).any(1))
ref = frame_trig if valid_hip[frame_trig] else int(np.where(valid_hip)[0][0])

centroid_robot = np.stack([O1w[ref], O2w[ref], O3w[ref], O4w[ref]]).mean(axis=0)
hdg = O1w[ref] - O4w[ref]
hdg[2] = 0.
hdg /= np.linalg.norm(hdg)
ang = np.arctan2(hdg[1], hdg[0])
cs, sn = np.cos(-ang), np.sin(-ang)
R_heading = np.array([[cs, -sn, 0.], [sn, cs, 0.], [0., 0., 1.]])

def to_robot(p_mm):
    return (R_heading @ (to_world(p_mm) - centroid_robot).T).T

# ─── Pre-compute all marker positions in robot frame ────────────────────────
print("Transforming markers …")
MARKERS = ["O1", "O2", "O3", "O4", "Tigger", "G1", "G2", "G3", "G4",
           "Ground1", "Ground2", "Ground3", "Ground4"]

pos = {}  # marker → (N, 3) robot-frame mm
for m in MARKERS:
    pos[m] = to_robot(get_xyz(m))

# Time axis
t_total = np.arange(len(traj_df)) / fs - frame_trig / fs

# ─── Subsampled frame indices ─────────────────────────────────────────────────
# Start from 5 s before trigger to end of recording
start_fi = max(0, frame_trig - int(5 * fs))
frame_idx = np.arange(start_fi, len(traj_df), STEP)
print(f"  Animation frames: {len(frame_idx)} (step={STEP}, start_fi={start_fi})")

# ─── Colour constants ─────────────────────────────────────────────────────────
ORANGE = "darkorange"
BLUE   = "royalblue"
MARKER_SIZE = 6

# ─── Helper: build Scatter3d (lines + markers) for a set of point pairs ───────

def _pts_to_xyz(pts):
    """pts: list of (x,y,z) or None → flat arrays with NaN gaps for Plotly lines."""
    xs, ys, zs = [], [], []
    for p in pts:
        if p is None or np.isnan(p).any():
            xs += [None]; ys += [None]; zs += [None]
        else:
            xs.append(float(p[0])); ys.append(float(p[1])); zs.append(float(p[2]))
    return xs, ys, zs


def line_trace(pt_pairs, color, name, showlegend=True):
    """
    pt_pairs: list of (p_start, p_end) each (3,) or None.
    Returns a Scatter3d trace that draws all segments.
    """
    xs, ys, zs = [], [], []
    for (a, b) in pt_pairs:
        a_ok = a is not None and not np.isnan(a).any()
        b_ok = b is not None and not np.isnan(b).any()
        if a_ok and b_ok:
            xs += [float(a[0]), float(b[0]), None]
            ys += [float(a[1]), float(b[1]), None]
            zs += [float(a[2]), float(b[2]), None]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=3),
        name=name,
        showlegend=showlegend,
    )


def dot_trace(points_dict, color, name, showlegend=True):
    """
    points_dict: {label: (3,)} — scatter dots.
    """
    xs, ys, zs, texts = [], [], [], []
    for lbl, p in points_dict.items():
        if p is not None and not np.isnan(p).any():
            xs.append(float(p[0])); ys.append(float(p[1])); zs.append(float(p[2]))
            texts.append(lbl)
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="markers+text",
        text=texts,
        textposition="top center",
        marker=dict(size=MARKER_SIZE, color=color),
        name=name,
        showlegend=showlegend,
    )


# ─── Build one frame's worth of traces ───────────────────────────────────────

def build_traces_for_frame(fi):
    """Return list of Plotly traces for frame index fi."""
    p = {m: pos[m][fi] for m in MARKERS}

    traces = []

    # 1. O1–O4 all pairs (orange lines)
    o_pairs = [(p["O1"], p["O2"]), (p["O1"], p["O3"]), (p["O1"], p["O4"]),
               (p["O2"], p["O3"]), (p["O2"], p["O4"]), (p["O3"], p["O4"])]
    traces.append(line_trace(o_pairs, ORANGE, "Body frame", showlegend=True))

    # 2. Tigger → all O (orange), only when visible
    tigger_visible = not np.isnan(p["Tigger"]).any()
    if tigger_visible:
        tg_pairs = [(p["Tigger"], p["O1"]), (p["Tigger"], p["O2"]),
                    (p["Tigger"], p["O3"]), (p["Tigger"], p["O4"])]
        traces.append(line_trace(tg_pairs, ORANGE, "Trigger", showlegend=True))

    # 3. Gx → Ox foot links (orange)
    foot_pairs = [(p["G1"], p["O1"]), (p["G2"], p["O2"]),
                  (p["G3"], p["O3"]), (p["G4"], p["O4"])]
    traces.append(line_trace(foot_pairs, ORANGE, "Foot links", showlegend=True))

    # 4. Ground pairs (blue)
    gnd_pts = [p["Ground1"], p["Ground2"], p["Ground3"], p["Ground4"]]
    gnd_valid = [g for g in gnd_pts if not np.isnan(g).any()]
    gnd_pairs = []
    for i in range(len(gnd_valid)):
        for j in range(i + 1, len(gnd_valid)):
            gnd_pairs.append((gnd_valid[i], gnd_valid[j]))
    traces.append(line_trace(gnd_pairs, BLUE, "Ground", showlegend=True))

    # 5. Scatter dots — body (orange)
    body_pts = {k: p[k] for k in ["O1", "O2", "O3", "O4"]}
    traces.append(dot_trace(body_pts, ORANGE, "Hip markers"))

    # 6. Scatter dots — feet (orange)
    foot_pts = {k: p[k] for k in ["G1", "G2", "G3", "G4"]}
    traces.append(dot_trace(foot_pts, ORANGE, "Foot markers", showlegend=False))

    # 7. Scatter dot — trigger (yellow-green when visible)
    if tigger_visible:
        traces.append(dot_trace({"Tigger": p["Tigger"]}, "limegreen", "Tigger"))

    # 8. Scatter dots — ground (blue)
    gnd_dict = {}
    for gm in ["Ground1", "Ground2", "Ground3", "Ground4"]:
        if not np.isnan(p[gm]).any():
            gnd_dict[gm] = p[gm]
    traces.append(dot_trace(gnd_dict, BLUE, "Ground markers", showlegend=False))

    return traces


# ─── Build axis limits (over all frames) ──────────────────────────────────────
print("Computing axis limits …")
all_pts = np.concatenate(
    [pos[m][~np.isnan(pos[m]).any(axis=1)] for m in MARKERS
     if not np.isnan(pos[m]).all()],
    axis=0
)
lo = np.nanpercentile(all_pts, 1, axis=0)
hi = np.nanpercentile(all_pts, 99, axis=0)
pad = (hi - lo) * 0.15
lo -= pad; hi += pad

xrange = [lo[0], hi[0]]
yrange = [lo[1], hi[1]]
zrange = [lo[2], hi[2]]

# ─── Build Plotly animation frames ────────────────────────────────────────────
print("Building animation frames …")

# Use trigger frame as initial display (body markers are guaranteed visible)
init_fi = frame_idx[np.searchsorted(frame_idx, frame_trig)]
init_traces = build_traces_for_frame(init_fi)

frames = []
for fi in frame_idx:
    traces = build_traces_for_frame(fi)
    frames.append(go.Frame(
        data=traces,
        name=str(fi),
    ))

# ─── Slider steps ────────────────────────────────────────────────────────────
slider_steps = []
for k, fi in enumerate(frame_idx):
    t_s = float(t_total[fi])
    slider_steps.append(dict(
        args=[[str(fi)],
              {"frame": {"duration": 0, "redraw": True},
               "mode": "immediate",
               "transition": {"duration": 0}}],
        label=f"{t_s:.2f}s",
        method="animate",
    ))

sliders = [dict(
    active=int(np.searchsorted(frame_idx, frame_trig)),
    currentvalue={"prefix": "t = ", "suffix": " s", "visible": True,
                  "font": {"size": 14}},
    pad={"b": 10, "t": 50},
    len=0.95,
    x=0.025,
    steps=slider_steps,
)]

updatemenus = [dict(
    type="buttons",
    showactive=False,
    y=1.08, x=0.1,
    xanchor="right",
    yanchor="top",
    pad={"t": 20, "r": 10},
    buttons=[
        dict(label="▶ Play",
             method="animate",
             args=[None, {"frame": {"duration": 50, "redraw": True},
                          "fromcurrent": True,
                          "transition": {"duration": 0}}]),
        dict(label="⏸ Pause",
             method="animate",
             args=[[None], {"frame": {"duration": 0, "redraw": False},
                            "mode": "immediate",
                            "transition": {"duration": 0}}]),
    ],
)]

# ─── Layout ───────────────────────────────────────────────────────────────────
layout = go.Layout(
    title=dict(text="CORGI VICON 3D Marker Visualizer", font=dict(size=16)),
    scene=dict(
        xaxis=dict(title="X (mm) forward", range=xrange),
        yaxis=dict(title="Y (mm) left",    range=yrange),
        zaxis=dict(title="Z (mm) up",      range=zrange),
        aspectmode="cube",
        camera=dict(
            eye=dict(x=1.5, y=-1.5, z=1.2),
        ),
    ),
    legend=dict(x=0.01, y=0.99),
    margin=dict(l=0, r=0, b=120, t=60),
    sliders=sliders,
    updatemenus=updatemenus,
)

fig = go.Figure(data=init_traces, layout=layout, frames=frames)

# ─── Write HTML ───────────────────────────────────────────────────────────────
print(f"Writing {OUTPUT_HTML} …")
fig.write_html(OUTPUT_HTML, include_plotlyjs="cdn", auto_open=False)
print(f"Done. Open in browser:\n  {os.path.abspath(OUTPUT_HTML)}")
