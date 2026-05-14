"""
metrics.py — CORGI experiment metric helpers.

Functions
---------
rmse(a, b, mask=None)
bias_stats(data, axis, n_init=50, n_ss=200)
contact_metrics(leg, vi, gmo, t_walk_end)
ekf_metrics(ekf, ekf_rpy, vicon_interp, mask_walk)

No ROS2 dependency.
"""

import numpy as np
from scipy.interpolate import interp1d


# ──────────────────────────────────────────────────────────────────────────────
# Basic
# ──────────────────────────────────────────────────────────────────────────────

def rmse(a: np.ndarray, b: np.ndarray, mask: np.ndarray = None) -> float:
    """Root-mean-square error between arrays a and b, ignoring NaN."""
    d = a - b
    if mask is not None:
        d = d[mask]
    v = ~np.isnan(d)
    return float(np.sqrt(np.mean(d[v] ** 2))) if v.any() else float('nan')


def bias_stats(data: dict, axis: str,
               n_init: int = 50, n_ss: int = 200):
    """
    Return (initial_mean, steady_state_mean, steady_state_std) for a bias axis.

    Parameters
    ----------
    data  : dict with key `axis` → numpy array
    axis  : 'x', 'y', or 'z'
    n_init: number of samples averaged for initial value
    n_ss  : number of samples averaged at the end for steady-state
    """
    v    = data[axis]
    init = float(v[:n_init].mean())  if len(v) >= n_init else float('nan')
    ss   = float(v[-n_ss:].mean())   if len(v) >= n_ss   else float('nan')
    std  = float(v[-n_ss:].std())    if len(v) >= n_ss   else float('nan')
    return init, ss, std


# ──────────────────────────────────────────────────────────────────────────────
# Contact detection
# ──────────────────────────────────────────────────────────────────────────────

def contact_metrics(leg: str, vi, gmo: dict, t_walk_end: float) -> dict:
    """
    Compute GMO vs VICON contact metrics for one leg over the walking phase.

    Parameters
    ----------
    leg        : 'LF', 'RF', 'RH', or 'LH'
    vi         : VICONData instance (from vicon_loader.load_vicon)
    gmo        : dict(t, LF, RF, RH, LH) from bag_loader
    t_walk_end : end of walking phase [s]

    Returns
    -------
    dict with: precision, recall, stance_ratio [%],
               mean_stance_ms [ms], mean_latency_ms [ms]
    """
    fs   = vi.fs
    t_cm = np.linspace(0.0, t_walk_end, int(t_walk_end * fs))

    # VICON contact on common grid
    h    = vi.foot_heights[leg]
    nnan = ~np.isnan(h)
    t_vi = vi.t_traj[nnan]
    c_vi_raw = vi.contact[leg][nnan].astype(float)
    c_vi = np.interp(t_cm, t_vi, c_vi_raw) > 0.5

    # GMO contact on common grid
    c_gm = np.interp(t_cm, gmo['t'], gmo[leg].astype(float)) > 0.5

    tp   = np.sum( c_vi &  c_gm)
    fp   = np.sum(~c_vi &  c_gm)
    fn   = np.sum( c_vi & ~c_gm)

    prec = tp / (tp + fp + 1e-9)
    rec  = tp / (tp + fn + 1e-9)

    # Stance ratio (VICON ground truth)
    stance_ratio = float(np.mean(c_vi)) * 100.0

    # Mean stance duration (VICON ground truth)
    edges  = np.diff(c_vi.astype(int), prepend=0, append=0)
    starts = np.where(edges ==  1)[0]
    ends   = np.where(edges == -1)[0]
    durations = [(ends[i] - starts[i]) / fs * 1000
                 for i in range(min(len(starts), len(ends)))]
    mean_dur = float(np.mean(durations)) if durations else 0.0

    # Mean detection latency (VICON rising edge → GMO rising edge)
    v_edges = np.diff(c_vi.astype(int),  prepend=0)
    g_edges = np.diff(c_gm.astype(int), prepend=0)
    lats = []
    for vs in np.where(v_edges == 1)[0]:
        window = g_edges[max(0, vs - 25): vs + 25]
        rel    = np.where(window == 1)[0]
        if len(rel):
            lats.append((rel[0] - min(25, vs)) / fs * 1000)
    mean_lat = float(np.mean(lats)) if lats else float('nan')

    return {
        'precision':      prec,
        'recall':         rec,
        'stance_ratio':   stance_ratio,
        'mean_stance_ms': mean_dur,
        'mean_latency_ms': mean_lat,
    }


def contact_metrics_all(vi, gmo: dict, t_walk_end: float) -> dict:
    """Run contact_metrics for all four legs. Returns dict keyed by leg."""
    return {leg: contact_metrics(leg, vi, gmo, t_walk_end)
            for leg in ['LF', 'RF', 'RH', 'LH']}


# ──────────────────────────────────────────────────────────────────────────────
# EKF accuracy
# ──────────────────────────────────────────────────────────────────────────────

def interp_vicon_to_ekf(vi, ekf_t: np.ndarray) -> dict:
    """
    Interpolate all VICON signals onto the EKF time axis.

    Returns dict with keys:
        px, py, pz, vx, vy, vz, roll, pitch, yaw
    """
    def _interp(arr):
        vm = ~np.isnan(arr)
        if vm.sum() < 2:
            return np.full(len(ekf_t), np.nan)
        return interp1d(vi.t_traj[vm], arr[vm],
                        bounds_error=False, fill_value=np.nan)(ekf_t)

    return {
        'px':    _interp(vi.pos_m[:, 0]),
        'py':    _interp(vi.pos_m[:, 1]),
        'pz':    _interp(vi.pos_m[:, 2]),
        'vx':    _interp(vi.v_body[:, 0]),
        'vy':    _interp(vi.v_body[:, 1]),
        'vz':    _interp(vi.v_body[:, 2]),
        'roll':  _interp(vi.rpy[:, 0]),
        'pitch': _interp(vi.rpy[:, 1]),
        'yaw':   _interp(vi.rpy[:, 2]),
    }


def ekf_metrics(ekf: dict, ekf_rpy: np.ndarray,
                vicon_i: dict, mask_walk: np.ndarray) -> dict:
    """
    Compute standard accuracy metrics against interpolated VICON ground truth.

    Parameters
    ----------
    ekf       : dict from bag_loader (t, px, py, pz, vx, vy, vz, ...)
    ekf_rpy   : (N,3) roll/pitch/yaw from EKF quaternion [rad]
    vicon_i   : dict from interp_vicon_to_ekf (px,py,pz,vx,vy,vz,roll,pitch,yaw)
    mask_walk : (N,) bool, True inside walking phase

    Returns
    -------
    dict with pos_rmse_{x,y,z,3d}, pos_max_3d, vel_rmse_{x,y,z},
              att_rmse_{roll,pitch,yaw}, vel_peak_vx
    """
    err_x = ekf['px'] - vicon_i['px']
    err_y = ekf['py'] - vicon_i['py']
    err_z = ekf['pz'] - vicon_i['pz']
    err_3d = np.sqrt(np.where(
        ~np.isnan(err_x) & ~np.isnan(err_y) & ~np.isnan(err_z),
        err_x**2 + err_y**2 + err_z**2, np.nan))
    valid_pos = ~np.isnan(err_3d) & mask_walk

    def _vr(key):
        return mask_walk & ~np.isnan(vicon_i[key])

    return {
        'pos_rmse_x':   rmse(ekf['px'], vicon_i['px'], valid_pos),
        'pos_rmse_y':   rmse(ekf['py'], vicon_i['py'], valid_pos),
        'pos_rmse_z':   rmse(ekf['pz'], vicon_i['pz'], valid_pos),
        'pos_rmse_3d':  float(np.sqrt(np.nanmean(err_3d[valid_pos]**2)))
                        if valid_pos.any() else float('nan'),
        'pos_max_3d':   float(np.nanmax(err_3d[valid_pos]))
                        if valid_pos.any() else float('nan'),
        'vel_rmse_x':   rmse(ekf['vx'], vicon_i['vx'], _vr('vx')),
        'vel_rmse_y':   rmse(ekf['vy'], vicon_i['vy'], _vr('vy')),
        'vel_rmse_z':   rmse(ekf['vz'], vicon_i['vz'], _vr('vz')),
        'att_rmse_roll':  rmse(ekf_rpy[:, 0], vicon_i['roll'],  _vr('roll')),
        'att_rmse_pitch': rmse(ekf_rpy[:, 1], vicon_i['pitch'], _vr('pitch')),
        'att_rmse_yaw':   rmse(ekf_rpy[:, 2], vicon_i['yaw'],   _vr('yaw')),
        'vel_peak_vx':  float(np.nanmax(np.abs(
                            vicon_i['vx'][_vr('vx')])))
                        if _vr('vx').any() else float('nan'),
        'err_3d':       err_3d,   # raw array for plotting
    }
