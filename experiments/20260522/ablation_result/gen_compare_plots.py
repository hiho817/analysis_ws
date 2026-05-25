#!/usr/bin/env python3
"""Generate comparison overlay plots: ESEKF (exp2) vs Legacy (exp3)"""
import sys, os, sqlite3
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

sys.path.insert(0, '/home/hiho817/analysis_ws/tools')
from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader import load_fusion_bag, load_legacy_bag
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUTDIR = os.path.dirname(os.path.abspath(__file__))

def interp_to(tsrc, ysrc, tdst):
    f = interp1d(tsrc, ysrc, axis=0, bounds_error=False, fill_value='extrapolate')
    return f(tdst)

# ── ESEKF exp2 ──────────────────────────────────────────────────────────────
BAG_E2 = ('/home/hiho817/analysis_ws/experiments/20260522/exp2/bags/'
          'mpc_esekf_20260522_193236/mpc_esekf_20260522_193236_0.db3')
VIC_E2 = '/home/hiho817/analysis_ws/experiments/20260522/exp2/vicon/EXP2.csv'

vi_e2  = load_vicon(VIC_E2, contact_threshold_m=0.015,
                    ground_markers=['ground1','ground2','ground3','ground4'])
bag_e2 = load_fusion_bag(BAG_E2, rate=1.0)
T_END_E2 = min(vi_e2.t_trigger_end, bag_e2['t_trigger_end'])

mask_ve2   = (vi_e2.t_traj >= 0) & (vi_e2.t_traj <= T_END_E2)
t_vic_e2   = vi_e2.t_traj[mask_ve2]
pos_vic_e2 = vi_e2.pos_m[mask_ve2]    # (N,3) [X,Y,Z]
vel_vic_e2 = vi_e2.v_body[mask_ve2]   # (N,3)

ekf = bag_e2['ekf']
mask_e = (ekf['t'] >= 0) & (ekf['t'] <= T_END_E2)
t_ekf   = ekf['t'][mask_e]
pos_ekf = np.column_stack([ekf['px'][mask_e], ekf['py'][mask_e], ekf['pz'][mask_e]])
vx_ekf  = ekf['vx'][mask_e]

od = bag_e2['odom']
mask_o  = (od['t'] >= 0) & (od['t'] <= T_END_E2)
t_odom  = od['t'][mask_o]
pos_odom = np.column_stack([od['px'][mask_o], od['py'][mask_o], od['pz'][mask_o]])

pv_e = np.column_stack([interp_to(t_vic_e2, pos_vic_e2[:,0], t_ekf),
                         interp_to(t_vic_e2, pos_vic_e2[:,1], t_ekf),
                         interp_to(t_vic_e2, pos_vic_e2[:,2], t_ekf)])
pv_o = np.column_stack([interp_to(t_vic_e2, pos_vic_e2[:,0], t_odom),
                         interp_to(t_vic_e2, pos_vic_e2[:,1], t_odom),
                         interp_to(t_vic_e2, pos_vic_e2[:,2], t_odom)])
vv_e = interp_to(t_vic_e2, vel_vic_e2[:,0], t_ekf)

# ── Legacy exp3 ─────────────────────────────────────────────────────────────
BAG_L3 = ('/home/hiho817/analysis_ws/experiments/20260522/exp3/bags/'
          'mpc_legacy_20260522_193533/mpc_legacy_20260522_193533_0.db3')
VIC_L3 = '/home/hiho817/analysis_ws/experiments/20260522/exp3/vicon/EXP3.csv'

vi_l3  = load_vicon(VIC_L3, contact_threshold_m=0.015,
                    ground_markers=['ground1','ground2','ground3','ground4'])
bag_l3 = load_legacy_bag(BAG_L3, rate=1.0)
T_END_L3 = min(vi_l3.t_trigger_end, bag_l3['t_trigger_end'])

mask_vl3 = (vi_l3.t_traj >= 0) & (vi_l3.t_traj <= T_END_L3)
t_vic_l3   = vi_l3.t_traj[mask_vl3]
p_vl3_raw  = vi_l3.pos_m[mask_vl3]
pos_vic_l3 = p_vl3_raw.copy()  # same Y convention as Legacy — no flip needed
vel_vic_l3 = vi_l3.v_body[mask_vl3]

pos_d = bag_l3['pos']
mask_l = (pos_d['t'] >= 0) & (pos_d['t'] <= T_END_L3)
t_leg  = pos_d['t'][mask_l]
pos_leg = np.column_stack([pos_d['x'][mask_l], pos_d['y'][mask_l], pos_d['z'][mask_l]])

vel_d = bag_l3['vel']
mask_lv = (vel_d['t'] >= 0) & (vel_d['t'] <= T_END_L3)
t_vel_l = vel_d['t'][mask_lv]; vx_l = vel_d['x'][mask_lv]

pv_l = np.column_stack([interp_to(t_vic_l3, pos_vic_l3[:,0], t_leg),
                         interp_to(t_vic_l3, pos_vic_l3[:,1], t_leg),
                         interp_to(t_vic_l3, pos_vic_l3[:,2], t_leg)])
vv_l = interp_to(t_vic_l3, vel_vic_l3[:,0], t_vel_l)

# ── Errors ──────────────────────────────────────────────────────────────────
z_off_e = pos_ekf[0, 2] - pv_e[0, 2]   # ~0.20 m body height vs VICON frame
z_off_l = pos_leg[0, 2] - pv_l[0, 2]
pos_ekf_adj  = pos_ekf.copy();  pos_ekf_adj[:, 2]  -= z_off_e
pos_odom_adj = pos_odom.copy(); pos_odom_adj[:, 2] -= z_off_e
pos_leg_adj  = pos_leg.copy();  pos_leg_adj[:, 2]  -= z_off_l
err_e = pos_ekf_adj - pv_e; err_o = pos_odom_adj - pv_o; err_l = pos_leg_adj - pv_l
err_e_2d = np.sqrt(err_e[:,0]**2 + err_e[:,1]**2)
err_o_2d = np.sqrt(err_o[:,0]**2 + err_o[:,1]**2)
err_l_2d = np.sqrt(err_l[:,0]**2 + err_l[:,1]**2)
rmse_e2d = np.sqrt(np.mean(err_e_2d**2))*100
rmse_o2d = np.sqrt(np.mean(err_o_2d**2))*100
rmse_l2d = np.sqrt(np.mean(err_l_2d**2))*100
print(f"ESEKF EKF 2D RMSE:          {rmse_e2d:.2f} cm")
print(f"ESEKF odom_mapping 2D RMSE: {rmse_o2d:.2f} cm")
print(f"Legacy IF 2D RMSE:          {rmse_l2d:.2f} cm")

# ── Figure 1: Trajectory ────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
ax = axes[0]
ax.plot(pos_vic_e2[:,1], pos_vic_e2[:,0], 'k-', lw=1.5, label='VICON')
ax.plot(pos_ekf[:,1], pos_ekf[:,0], 'b-', lw=1, label='Inner EKF')
ax.plot(pos_odom[:,1], pos_odom[:,0], 'c--', lw=1, label='odom_mapping')
ax.set_xlabel('Y (m)'); ax.set_ylabel('X (m)'); ax.set_title('ESEKF (exp2)')
ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(pos_vic_l3[:,1], pos_vic_l3[:,0], 'k-', lw=1.5, label='VICON')
ax.plot(pos_leg[:,1], pos_leg[:,0], 'r-', lw=1, label='Legacy IF')
ax.set_xlabel('Y (m)'); ax.set_ylabel('X (m)'); ax.set_title('Legacy (exp3)')
ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

ax = axes[2]
ax.plot(pos_vic_e2[:,1], pos_vic_e2[:,0], 'k-', lw=1.5, label='VICON')
ax.plot(pos_odom[:,1], pos_odom[:,0], 'c--', lw=1,
        label=f'ESEKF odom ({rmse_o2d:.2f}cm)')
ax.plot(pos_leg[:,1], pos_leg[:,0], 'r-', lw=1,
        label=f'Legacy IF ({rmse_l2d:.2f}cm)')
ax.set_xlabel('Y (m)'); ax.set_ylabel('X (m)'); ax.set_title('ESEKF vs Legacy')
ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'fig_compare_traj.png'), dpi=150)
plt.close(); print("Saved fig_compare_traj.png")

# ── Figure 2: Error + Velocity ──────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 9))
ax = axes[0]
ax.plot(t_ekf, err_e[:,0]*100, 'b-',  lw=1, label='ESEKF ΔX')
ax.plot(t_ekf, err_e[:,1]*100, 'b--', lw=1, label='ESEKF ΔY')
ax.plot(t_leg, err_l[:,0]*100, 'r-',  lw=1, label='Legacy ΔX')
ax.plot(t_leg, err_l[:,1]*100, 'r--', lw=1, label='Legacy ΔY')
ax.axhline(0, color='k', lw=0.5)
ax.set_ylabel('Error (cm)'); ax.set_title('X/Y Position Error vs VICON')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(t_ekf,  err_e_2d*100, 'b-',  lw=1.2, label=f'ESEKF Inner EKF (RMSE={rmse_e2d:.2f}cm)')
ax.plot(t_odom, err_o_2d*100, 'c-',  lw=1.2, label=f'ESEKF odom_mapping (RMSE={rmse_o2d:.2f}cm)')
ax.plot(t_leg,  err_l_2d*100, 'r-',  lw=1.2, label=f'Legacy IF (RMSE={rmse_l2d:.2f}cm)')
ax.set_ylabel('2D Error (cm)'); ax.set_title('2D Position Error vs VICON')
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

ax = axes[2]
ax.plot(t_vic_e2, vel_vic_e2[:,0], 'k-', lw=1.5, label='VICON vx', alpha=0.7)
ax.plot(t_ekf, vx_ekf, 'b-', lw=1, label='ESEKF vx')
ax.plot(t_vel_l, vx_l, 'r-', lw=1, label='Legacy vx')
ax.set_xlabel('Time (s)'); ax.set_ylabel('vx (m/s)')
ax.set_title('Forward Velocity (vx) vs VICON')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'fig_compare_error.png'), dpi=150)
plt.close(); print("Saved fig_compare_error.png")

# ── Figure 3: Contact ────────────────────────────────────────────────────────
def _get_trg_ts0(db_path):
    conn = sqlite3.connect(db_path); cur = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    cur.execute(f"SELECT timestamp FROM messages WHERE topic_id={tmap['/trigger']} ORDER BY timestamp LIMIT 1")
    ts0 = cur.fetchone()[0]; conn.close(); return ts0

def _load_contact(bag_db, topic, trg_ts0):
    conn = sqlite3.connect(bag_db); cur = conn.cursor()
    cur.execute("SELECT name, id FROM topics")
    tmap = {r[0]: r[1] for r in cur.fetchall()}
    tid = tmap.get(topic)
    if not tid: conn.close(); return None
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
    rows = cur.fetchall(); conn.close()
    from corgi_msgs.msg import ContactStateStamped
    ts_list, lf_list, rf_list, rh_list, lh_list = [], [], [], [], []
    for ts, data in rows:
        msg = deserialize_message(bytes(data), ContactStateStamped)
        t = (ts - trg_ts0) / 1e9
        ts_list.append(t)
        lf_list.append(1.0 if msg.module_a.contact else 0.0)
        rf_list.append(1.0 if msg.module_b.contact else 0.0)
        rh_list.append(1.0 if msg.module_c.contact else 0.0)
        lh_list.append(1.0 if msg.module_d.contact else 0.0)
    return dict(t=np.array(ts_list), LF=np.array(lf_list), RF=np.array(rf_list),
                RH=np.array(rh_list), LH=np.array(lh_list))

# GMO already loaded in bag_e2['gmo']
gmo_ct = bag_e2['gmo']
leg_ct = _load_contact(BAG_L3, '/odometry/legacy/contact', _get_trg_ts0(BAG_L3))

legs_list = [('LF','Left Front'), ('RF','Right Front'), ('RH','Right Hind'), ('LH','Left Hind')]
fig, axes = plt.subplots(4, 2, figsize=(14, 10))
for i, (lkey, llbl) in enumerate(legs_list):
    ax = axes[i][0]
    tg = gmo_ct['t']; mg = (tg >= 0) & (tg <= T_END_E2)
    tve = vi_e2.t_traj; mve = (tve >= 0) & (tve <= T_END_E2)
    ax.plot(tve[mve], vi_e2.contact[lkey][mve].astype(float)+0.05, 'k-', lw=0.8, alpha=0.5, label='VICON')
    ax.step(tg[mg], gmo_ct[lkey][mg], 'b-', lw=1.0, where='post', label='GMO')
    ax.set_ylim(-0.15,1.3); ax.set_yticks([0,1]); ax.set_yticklabels(['OFF','ON'])
    ax.set_title(f'{llbl} ({lkey}) — ESEKF GMO'); ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
    if i==3: ax.set_xlabel('Time (s)')

    ax = axes[i][1]
    tlc = leg_ct['t']; mlc = (tlc >= 0) & (tlc <= T_END_L3)
    tvl = vi_l3.t_traj; mvl = (tvl >= 0) & (tvl <= T_END_L3)
    ax.plot(tvl[mvl], vi_l3.contact[lkey][mvl].astype(float)+0.05, 'k-', lw=0.8, alpha=0.5, label='VICON')
    ax.step(tlc[mlc], leg_ct[lkey][mlc], 'r-', lw=1.0, where='post', label='Fixed Gait')
    ax.set_ylim(-0.15,1.3); ax.set_yticks([0,1]); ax.set_yticklabels(['OFF','ON'])
    ax.set_title(f'{llbl} ({lkey}) — Legacy Fixed-Gait'); ax.legend(fontsize=7); ax.grid(True, alpha=0.2)
    if i==3: ax.set_xlabel('Time (s)')

plt.suptitle('Contact Detection: ESEKF GMO vs Legacy Fixed-Gait', fontsize=12)
plt.tight_layout()
plt.savefig(os.path.join(OUTDIR, 'fig_compare_contact.png'), dpi=150, bbox_inches='tight')
plt.close(); print("Saved fig_compare_contact.png")

print("\nAll comparison figures done.")
