"""
vicon_loader.py — load VICON Nexus CSV for CORGI real-robot experiments.

Expected markers
----------------
Ground1–Ground4 (default) OR custom list via ground_markers=   ground plane fit
Tigger            trigger synchronisation marker (first valid frame = t=0)
O1–O4             hip markers (body pose)
G1–G4             foot markers (LF=G1, RF=G2, RH=G3, LH=G4)

New in 20260513 format
-----------------------
- CSV separator auto-detected (comma or tab)
- ground_markers parameter allows custom marker list for ground plane fit
- VICONData.frame_trig_end / t_trigger_end  (last valid Tigger frame = trigger OFF)

Usage
-----
    from corgi_analysis.vicon_loader import load_vicon

    vi = load_vicon('walk_2m_01.csv', contact_threshold_m=0.005)
    # vi.t_traj      — (N,) time axis, trigger = t=0  [s]
    # vi.pos_m       — (N,3) centroid position, robot-centric  [m]
    # vi.v_body      — (N,3) body-frame velocity  [m/s]
    # vi.rpy         — (N,3) roll/pitch/yaw  [rad]
    # vi.foot_heights — dict {leg: (N,) height above ground}  [m]
    # vi.contact     — dict {leg: (N,) bool}
    # vi.valid_hip   — (N,) bool

No ROS2 dependency.
"""

import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from typing import Dict
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter


# ──────────────────────────────────────────────────────────────────────────────
# Result container
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class VICONData:
    # Time
    t_traj: np.ndarray           # (N,) trigger = t=0  [s]
    t_vicon_trigger: float       # trigger frame time  [s]
    frame_trig: int              # trigger frame index

    # Kinematics (robot-centric, metres)
    pos_m: np.ndarray            # (N,3) centroid XYZ
    v_body: np.ndarray           # (N,3) body-frame velocity
    rpy: np.ndarray              # (N,3) roll/pitch/yaw [rad]

    # Contact
    foot_heights: Dict[str, np.ndarray]   # {leg: (N,) metres}
    contact: Dict[str, np.ndarray]        # {leg: (N,) bool}
    contact_threshold_m: float

    # Validity
    valid_hip: np.ndarray        # (N,) bool — all four O markers visible

    # Trigger OFF (last valid Tigger frame)
    frame_trig_end: int          # last valid Tigger frame index (-1 if no end)
    t_trigger_end: float         # time of trigger OFF, relative to trigger ON [s]

    # Internals (for custom marker queries)
    fs: float
    _R_ground: np.ndarray        # (3,3)
    _centroid_g: np.ndarray      # (3,)
    _R_heading: np.ndarray       # (3,3)  robot X direction
    _centroid_robot: np.ndarray  # (3,)  robot origin in world frame
    _traj_df: pd.DataFrame
    _marker_col_map: dict

    def to_world(self, p_mm: np.ndarray) -> np.ndarray:
        """VICON raw → ground-aligned world frame (units: mm)."""
        return (self._R_ground @ (p_mm - self._centroid_g).T).T

    def to_robot(self, p_mm: np.ndarray) -> np.ndarray:
        """VICON raw → robot-centric frame (units: mm, origin at trigger)."""
        return (self._R_heading @ (self.to_world(p_mm) - self._centroid_robot).T).T

    def get_xyz(self, marker: str) -> np.ndarray:
        """Raw VICON XYZ for a marker (N,3), mm."""
        cols = self._marker_col_map[marker]
        return self._traj_df[cols].values.astype(float)

    def markers(self):
        return list(self._marker_col_map.keys())


# ──────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ──────────────────────────────────────────────────────────────────────────────

def _find_section_row(filepath: str, section_name: str) -> int:
    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            if line.strip().startswith(section_name):
                return i
    raise ValueError(f"Section '{section_name}' not found in {filepath}")


def _build_marker_col_map(csv_path: str, traj_section_row: int) -> dict:
    raw = pd.read_csv(csv_path, skiprows=traj_section_row + 2,
                      nrows=1, header=None, sep=None, engine='python').iloc[0].tolist()
    marker_map = {}
    col = 2
    while col < len(raw):
        name = str(raw[col]).strip()
        if pd.notna(raw[col]) and name and name != 'nan':
            short = name.split(':')[-1]
            marker_map[short] = [col, col + 1, col + 2]
        col += 3
    return marker_map


def _rotation_to_align_z(n: np.ndarray) -> np.ndarray:
    """Rotation matrix that aligns vector n to +Z."""
    n = n / np.linalg.norm(n)
    z = np.array([0., 0., 1.])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    c = np.dot(n, z)
    if s < 1e-10:
        return np.eye(3)
    Vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + Vx + Vx @ Vx * ((1 - c) / s ** 2)


def _sg_velocity(pos: np.ndarray, fs: float = 500.0,
                 window: int = 11, poly: int = 3) -> np.ndarray:
    """Savitzky-Golay derivative velocity (handles NaN gaps)."""
    vel = np.full_like(pos, np.nan)
    valid = ~np.isnan(pos).any(axis=1)
    edges = np.diff(valid.astype(int), prepend=0, append=0)
    for st, en in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        seg = pos[st:en]
        if len(seg) >= window:
            for ax in range(3):
                vel[st:en, ax] = savgol_filter(
                    seg[:, ax], window, poly, deriv=1, delta=1.0 / fs)
    return vel


# ──────────────────────────────────────────────────────────────────────────────
# Public API
# ──────────────────────────────────────────────────────────────────────────────

def load_vicon(csv_path: str,
               contact_threshold_m: float = 0.005,
               fs: float = 500.0,
               sg_window: int = 11,
               sg_poly: int = 3,
               ground_markers: list = None) -> VICONData:
    """
    Load a VICON Nexus CSV and compute kinematics in the robot-centric frame.

    Parameters
    ----------
    csv_path : str
    contact_threshold_m : float
        Foot height threshold for contact detection (metres). Default 5 mm.
    fs : float
        VICON capture rate (Hz). Default 500.
    sg_window / sg_poly : int
        Savitzky-Golay parameters for velocity estimation.
    ground_markers : list of str, optional
        Names of markers (after namespace stripping) used for ground plane fit.
        Must be 3 or 4 markers. Defaults to ['Ground1','Ground2','Ground3','Ground4'].

    Returns
    -------
    VICONData
    """
    if ground_markers is None:
        ground_markers = ['Ground1', 'Ground2', 'Ground3', 'Ground4']
    traj_row = _find_section_row(csv_path, 'Trajectories')
    traj_df  = pd.read_csv(csv_path, skiprows=traj_row + 5, header=None,
                            sep=None, engine='python')
    marker_col_map = _build_marker_col_map(csv_path, traj_row)
    print(f'[vicon_loader] Markers: {list(marker_col_map.keys())}')

    def get_xyz(name):
        # Nexus capitalization differs across captures (ground1 vs Ground1).
        key = marker_col_map.get(name)
        if key is None:
            canonical = {k.lower(): k for k in marker_col_map}
            key = marker_col_map[canonical[name.lower()]]
        return traj_df[key].values.astype(float)

    # ── Ground plane ──────────────────────────────────────────────────────────
    gpts = []
    for m in ground_markers:
        xyz = get_xyz(m)
        v   = ~np.isnan(xyz).any(axis=1)
        if v.any():
            gpts.append(xyz[v][0])
    if len(gpts) < 3:
        raise ValueError(f'Ground plane fit requires ≥3 valid markers, '
                         f'got {len(gpts)} from {ground_markers}')
    gpts = np.array(gpts)
    centroid_g = gpts.mean(axis=0)
    _, _, Vt = np.linalg.svd(gpts - centroid_g)
    n_ground = Vt[-1]
    if n_ground[2] < 0:
        n_ground = -n_ground
    R_ground = _rotation_to_align_z(n_ground)

    def to_world(p):
        return (R_ground @ (p - centroid_g).T).T

    # ── Trigger frame ─────────────────────────────────────────────────────────
    tigger_raw   = get_xyz('Tigger')
    tigger_valid = ~np.isnan(tigger_raw).any(axis=1)
    all_trig_frames = np.where(tigger_valid)[0]
    frame_trig   = int(all_trig_frames[0])   # trigger ON  = first valid frame
    frame_trig_end = int(all_trig_frames[-1]) # trigger OFF = last valid frame
    t_vicon_trigger = frame_trig / fs
    # t_trigger_end is relative to trigger ON (t=0)
    t_trigger_end_sec = (frame_trig_end - frame_trig) / fs

    # ── Hip markers in world frame ────────────────────────────────────────────
    O1w = to_world(get_xyz('O1')); O2w = to_world(get_xyz('O2'))
    O3w = to_world(get_xyz('O3')); O4w = to_world(get_xyz('O4'))
    valid_hip = ~(
        np.isnan(O1w).any(1) | np.isnan(O2w).any(1) |
        np.isnan(O3w).any(1) | np.isnan(O4w).any(1)
    )

    # ── Robot-centric frame (origin = centroid at trigger) ────────────────────
    ref = frame_trig if valid_hip[frame_trig] else int(np.where(valid_hip)[0][0])
    centroid_robot = np.array([O1w[ref], O2w[ref], O3w[ref], O4w[ref]]).mean(axis=0)
    hdg = O1w[ref] - O4w[ref]
    hdg[2] = 0.
    hdg /= np.linalg.norm(hdg)
    ang = np.arctan2(hdg[1], hdg[0])
    cs, sn = np.cos(-ang), np.sin(-ang)
    R_heading = np.array([[cs, -sn, 0.], [sn, cs, 0.], [0., 0., 1.]])

    def to_robot(p):
        return (R_heading @ (to_world(p) - centroid_robot).T).T

    # ── Position (centroid, metres) ───────────────────────────────────────────
    N = len(traj_df)
    O1r = to_robot(get_xyz('O1')); O2r = to_robot(get_xyz('O2'))
    O3r = to_robot(get_xyz('O3')); O4r = to_robot(get_xyz('O4'))
    centroid = np.full((N, 3), np.nan)
    centroid[valid_hip] = (O1r[valid_hip] + O2r[valid_hip] +
                           O3r[valid_hip] + O4r[valid_hip]) / 4.0
    pos_m = centroid / 1000.0   # mm → m

    # ── Body rotation matrix ──────────────────────────────────────────────────
    R_body = np.full((N, 3, 3), np.nan)
    for i in np.where(valid_hip)[0]:
        pts  = np.stack([O1r[i], O2r[i], O3r[i], O4r[i]])
        _, _, Vt2 = np.linalg.svd(pts - pts.mean(0))
        Zb = Vt2[-1]
        if Zb[2] < 0:
            Zb = -Zb
        xr = O1r[i] - O4r[i]
        xr -= np.dot(xr, Zb) * Zb
        Xb = xr / np.linalg.norm(xr)
        Yb = np.cross(Zb, Xb)
        Yb /= np.linalg.norm(Yb)
        R_body[i] = np.column_stack([Xb, Yb, Zb])

    valid_rot = ~np.isnan(R_body).any(axis=(1, 2))
    rpy = np.full((N, 3), np.nan)
    for i in np.where(valid_rot)[0]:
        rpy[i] = Rotation.from_matrix(R_body[i]).as_euler('ZYX')[::-1]

    # ── Velocity (Savitzky-Golay, body frame) ─────────────────────────────────
    v_world = _sg_velocity(pos_m, fs=fs, window=sg_window, poly=sg_poly)
    v_body  = np.full_like(v_world, np.nan)
    ok = valid_rot & ~np.isnan(v_world).any(1)
    v_body[ok] = np.einsum('nij,nj->ni', R_body[ok].transpose(0, 2, 1), v_world[ok])

    # ── Foot heights (world frame Z, metres) ──────────────────────────────────
    foot_heights = {}
    contact      = {}
    for leg, marker in [('LF', 'G1'), ('RF', 'G2'), ('RH', 'G3'), ('LH', 'G4')]:
        h_mm = to_world(get_xyz(marker))[:, 2]   # Z above ground plane (mm)
        fh = h_mm / 1000.0
        foot_heights[leg] = fh
        contact[leg]      = fh < contact_threshold_m

    # ── Time axis ─────────────────────────────────────────────────────────────
    t_traj = np.arange(N) / fs - t_vicon_trigger

    print(f'[vicon_loader] N={N} frames, trigger frame={frame_trig} '
          f'({t_vicon_trigger:.3f} s), '
          f'trigger_end frame={frame_trig_end} ({t_trigger_end_sec:.3f} s after ON), '
          f'contact_threshold={contact_threshold_m*1000:.0f} mm')

    return VICONData(
        t_traj=t_traj,
        t_vicon_trigger=t_vicon_trigger,
        frame_trig=frame_trig,
        frame_trig_end=frame_trig_end,
        t_trigger_end=t_trigger_end_sec,
        pos_m=pos_m,
        v_body=v_body,
        rpy=rpy,
        foot_heights=foot_heights,
        contact=contact,
        contact_threshold_m=contact_threshold_m,
        valid_hip=valid_hip,
        fs=fs,
        _R_ground=R_ground,
        _centroid_g=centroid_g,
        _R_heading=R_heading,
        _centroid_robot=centroid_robot,
        _traj_df=traj_df,
        _marker_col_map=marker_col_map,
    )
