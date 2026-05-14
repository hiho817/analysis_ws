#!/usr/bin/env python3
"""
CORGI Experiment Full Analysis — 20260513 walk_2m_01

Follows corgi-data-analysis skill Steps 1-4:
  Step 1: Contact Detection (RF/RH, region-filtered, Prec/Rec/F1/Acc + Latency)
  Step 2: Inner EKF (position, velocity, attitude, ba, bw)
  Step 3: Outer Fusion Node (odom_mapping, fusion/bv)
  Step 4: LiDAR Input Quality (/lidar_odom)

NOTE: source ~/corgi_ws/corgi_ros2_ws/install/setup.bash before running.
"""

import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial import ConvexHull, Delaunay
from scipy.spatial.transform import Rotation

_TOOLS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '..', '..', '..', '..', 'tools'))
sys.path.insert(0, _TOOLS)

from corgi_analysis.vicon_loader import load_vicon
from corgi_analysis.bag_loader   import load_fusion_bag

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE      = os.path.dirname(os.path.abspath(__file__))
RESULTS   = BASE
BAG_DB    = os.path.join(BASE, '..', 'bags',
                         'odom_fusion20260512_205637',
                         'odom_fusion20260512_205637_0.db3')
VICON_CSV = os.path.join(BASE, '..', 'vicon', 'walk_2m_01.csv')
TRIAL     = 'walk_2m_01'
DATE      = '20260513'
CONTACT_THRESHOLD_M = 0.012   # 12 mm

# ─── Helpers ──────────────────────────────────────────────────────────────────
def interp_to(src_t, src_v, tgt_t):
    f = interp1d(src_t, src_v, bounds_error=False, fill_value=np.nan)
    return f(tgt_t)

def quat_to_rpy_deg(qw, qx, qy, qz):
    r = Rotation.from_quat(np.column_stack([qx, qy, qz, qw]))
    return np.degrees(r.as_euler('ZYX')[:, ::-1])   # [roll, pitch, yaw]

def rmse(d):
    return float(np.sqrt(np.nanmean(np.asarray(d) ** 2)))

def _shade(ax, t, c, color, alpha, hatch=None):
    prev = False; t0 = 0
    for i in range(len(t)):
        if c[i] and not prev: t0 = t[i]
        elif not c[i] and prev:
            kw = dict(color=color, alpha=alpha, lw=0)
            if hatch: kw['hatch'] = hatch
            ax.axvspan(t0, t[i-1], **kw)
        prev = bool(c[i])
    if prev:
        kw = dict(color=color, alpha=alpha, lw=0)
        if hatch: kw['hatch'] = hatch
        ax.axvspan(t0, t[-1], **kw)

# ─── Coordinate check helpers ─────────────────────────────────────────────────
def verify_transforms(vi):
    print('\n=== Coordinate Transform Verification ===')
    print('\n[1] Ground markers at Z=0 (world frame):')
    errs = []
    for m in ['groundB1', 'groundB2', 'groundA3', 'groundA4']:
        xyz = vi.get_xyz(m)
        valid = ~np.isnan(xyz).any(axis=1)
        if not valid.any(): print(f'  [WARN] {m} no valid frames'); continue
        pw = vi.to_world(xyz[valid][0:1])[0]
        pr = vi.to_robot(xyz[valid][0:1])[0]
        errs.append(abs(pw[2]))
        print(f'  {m} → world Z={pw[2]:.1f}mm (≈0) | robot Z={pr[2]:.1f}mm (≈-body_ht)')
    status = '[OK]' if max(errs) <= 20 else '[WARN]'
    print(f'  {status} max world Z = {max(errs):.1f}mm')
    print('\n[2] Robot centroid at origin:')
    ok = np.where(vi.valid_hip)[0]
    ref = vi.frame_trig if vi.valid_hip[vi.frame_trig] else ok[0]
    c = sum(vi.to_robot(vi.get_xyz(f'O{i}'))[ref] for i in range(1,5)) / 4.0
    print(f'  Centroid: X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f} mm')
    print(f'  {"[OK]" if abs(c[0])<5 else "[WARN]"} centroid at origin')
    print('\n[3] Robot moves in +X:')
    c0 = sum(vi.to_robot(vi.get_xyz(f'O{i}'))[ok[0]] for i in range(1,5)) / 4.0
    cN = sum(vi.to_robot(vi.get_xyz(f'O{i}'))[ok[-1]] for i in range(1,5)) / 4.0
    dx = (cN - c0) / 1000.0
    print(f'  ΔX={dx[0]:.3f} ΔY={dx[1]:.3f} m  {"[OK]" if dx[0]>0.2 else "[WARN]"}')

# ─── Region helpers ───────────────────────────────────────────────────────────
def build_region_hulls(vi):
    def xy(m):
        xyz = vi.get_xyz(m)
        v = ~np.isnan(xyz).any(axis=1)
        return vi.to_robot(xyz[v][0:1])[0, :2]
    pts_A = np.array([xy(f'groundA{i}') for i in range(1,5)])
    pts_B = np.array([xy(f'groundB{i}') for i in range(1,5)])
    print('\n=== Ground Region (robot-centric XY, mm) ===')
    for i,(a,b) in enumerate(zip(pts_A,pts_B),1):
        print(f'  A{i}:({a[0]:.0f},{a[1]:.0f})  B{i}:({b[0]:.0f},{b[1]:.0f})')
    return Delaunay(pts_A), Delaunay(pts_B), pts_A, pts_B

def in_region(xy_mm, hA, hB):
    return (hA.find_simplex(xy_mm) >= 0) | (hB.find_simplex(xy_mm) >= 0)

# ─── Load ─────────────────────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
print('='*60)
vi  = load_vicon(VICON_CSV, contact_threshold_m=CONTACT_THRESHOLD_M,
                 ground_markers=['groundB1','groundB2','groundA3','groundA4'])
bag = load_fusion_bag(BAG_DB, rate=1.0)
ekf = bag['ekf']; ba = bag['ba']; bw = bag['bw']
gmo = bag['gmo']; odom = bag['odom']; fv = bag['fv']; lidar = bag['lidar']

verify_transforms(vi)

T_END = min(vi.t_trigger_end, bag['t_trigger_end'])
print(f'\nAnalysis window: t ∈ [0, {T_END:.2f}] s')

mask_win_vi = (vi.t_traj >= 0.0) & (vi.t_traj <= T_END)
t_win = vi.t_traj[mask_win_vi]
pos_vicon = vi.pos_m[mask_win_vi]
v_body_vi = vi.v_body[mask_win_vi]
rpy_vicon  = vi.rpy[mask_win_vi]

# EKF window
ekf_mask = (ekf['t'] >= 0.0) & (ekf['t'] <= T_END)
et = ekf['t'][ekf_mask]
epx=ekf['px'][ekf_mask]; epy=ekf['py'][ekf_mask]; epz=ekf['pz'][ekf_mask]
evx=ekf['vx'][ekf_mask]; evy=ekf['vy'][ekf_mask]; evz=ekf['vz'][ekf_mask]
eqw=ekf['qw'][ekf_mask]; eqx=ekf['qx'][ekf_mask]
eqy=ekf['qy'][ekf_mask]; eqz=ekf['qz'][ekf_mask]
ecov_px=ekf['cov_px'][ekf_mask]; ecov_py=ekf['cov_py'][ekf_mask]; ecov_pz=ekf['cov_pz'][ekf_mask]
rpy_ekf_deg = quat_to_rpy_deg(eqw,eqx,eqy,eqz)

# VICON valid indices
vi_valid = ~np.isnan(pos_vicon).any(1)
vi_t_v = t_win[vi_valid]
rpy_vi_valid = ~np.isnan(rpy_vicon).any(1)
rpy_vi_t_v = t_win[rpy_vi_valid]

def vi2ekf(src): return interp_to(vi_t_v, src[vi_valid], et)

vi_px_e=vi2ekf(pos_vicon[:,0]); vi_py_e=vi2ekf(pos_vicon[:,1]); vi_pz_e=vi2ekf(pos_vicon[:,2])
vi_vx_e=vi2ekf(v_body_vi[:,0]); vi_vy_e=vi2ekf(v_body_vi[:,1]); vi_vz_e=vi2ekf(v_body_vi[:,2])
vi_roll_e =interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid,0]), et)
vi_pitch_e=interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid,1]), et)
vi_yaw_e  =interp_to(rpy_vi_t_v, np.degrees(rpy_vicon[rpy_vi_valid,2]), et)

# ─── T_{odom←camera_init} ─────────────────────────────────────────────────────
print('\n=== T_{odom←camera_init} via Procrustes ===')
lx=interp_to(lidar['t'],lidar['px'],odom['t'])
ly=interp_to(lidar['t'],lidar['py'],odom['t'])
lz=interp_to(lidar['t'],lidar['pz'],odom['t'])
vl=~np.isnan(lx)
lpts=np.column_stack([lx[vl],ly[vl],lz[vl]])
opts=np.column_stack([odom['px'][vl],odom['py'][vl],odom['pz'][vl]])
lc=lpts.mean(0); oc=opts.mean(0)
H=(lpts-lc).T@(opts-oc)
U,S,Vt2=np.linalg.svd(H)
R_CO=Vt2.T@U.T
if np.linalg.det(R_CO)<0: Vt2[-1]*=-1; R_CO=Vt2.T@U.T
t_CO=oc-R_CO@lc
rpy_CO=np.degrees(Rotation.from_matrix(R_CO).as_euler('ZYX')[::-1])
p_chk=(R_CO@lpts.T).T+t_CO; resid=np.linalg.norm(p_chk-opts,axis=1)
print(f'  t_CO={t_CO}  RPY={rpy_CO}°')
print(f'  Residual: mean={resid.mean()*100:.1f}cm max={resid.max()*100:.1f}cm')
lidar_xyz_odom=(R_CO@np.column_stack([lidar['px'],lidar['py'],lidar['pz']]).T).T+t_CO
lidar['px_odom']=lidar_xyz_odom[:,0]
lidar['py_odom']=lidar_xyz_odom[:,1]
lidar['pz_odom']=lidar_xyz_odom[:,2]

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 1: Contact Detection
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 1: Contact Detection')
hA,hB,pts_A,pts_B = build_region_hulls(vi)

def interp_gmo(gmo_t, gmo_leg, t_tgt):
    mk=(gmo_t>=-0.5)&(gmo_t<=T_END+0.5)
    t_g=gmo_t[mk]; c_g=gmo_leg[mk].astype(float)
    if len(t_g)<2: return np.zeros(len(t_tgt),dtype=bool)
    return interp1d(t_g,c_g,kind='nearest',bounds_error=False,fill_value=0.)(t_tgt)>0.5

contact_results={}
for leg,gm in [('RF','G2'),('RH','G3')]:
    hf=vi.foot_heights[leg]; cf=vi.contact[leg]
    fxyz=vi.get_xyz(gm)
    fxy=np.full((len(vi.t_traj),2),np.nan)
    vf=~np.isnan(fxyz).any(1)
    if vf.any(): fxy[vf]=vi.to_robot(fxyz[vf])[:,:2]
    rmask=np.zeros(len(vi.t_traj),dtype=bool)
    if vf.any(): rmask[vf]=in_region(fxy[vf],hA,hB)
    amask=mask_win_vi&rmask
    ta=vi.t_traj[amask]; cva=cf[amask]; ha=hf[amask]
    cga=interp_gmo(gmo['t'],gmo[leg],ta)
    nm=~np.isnan(ha); cvv=cva[nm]; cgv=cga[nm]
    TP=int(np.sum(cvv&cgv)); TN=int(np.sum(~cvv&~cgv))
    FP=int(np.sum(~cvv&cgv)); FN=int(np.sum(cvv&~cgv)); N=TP+TN+FP+FN
    acc=((TP+TN)/N) if N else float('nan')
    prec=(TP/(TP+FP)) if (TP+FP) else float('nan')
    rec=(TP/(TP+FN)) if (TP+FN) else float('nan')
    f1=(2*TP/(2*TP+FP+FN)) if (2*TP+FP+FN) else float('nan')
    vi_on=ta[np.diff(cva.astype(int),prepend=0)==1]
    gmo_on=ta[np.diff(cga.astype(int),prepend=0)==1]
    lats=[]
    for vt in vi_on:
        d=gmo_on-vt; near=d[np.abs(d)<0.1]
        if len(near): lats.append(near[np.argmin(np.abs(near))])
    lat_ms=float(np.mean(lats)*1000) if lats else float('nan')
    contact_results[leg]=dict(TP=TP,TN=TN,FP=FP,FN=FN,N=N,
        accuracy=acc,precision=prec,recall=rec,f1=f1,mean_latency_ms=lat_ms,
        t_analysis=ta,c_vi_analysis=cva,c_gmo_analysis=cga,h_analysis=ha,
        region_mask_full=rmask,foot_xy_robot=fxy,n_in_region=int(np.sum(amask)))
    print(f'  {leg}({gm}): N={N} TP={TP} TN={TN} FP={FP} FN={FN}  '
          f'Acc={acc*100:.1f}% Prec={prec*100:.1f}% Rec={rec*100:.1f}% '
          f'F1={f1:.4f} Lat={lat_ms:.1f}ms')

C_VI='#1E88E5'; C_GMO='#FF5722'; C_REG='#4CAF50'

# Plot 1a: contact timeline
fig,axes=plt.subplots(4,1,figsize=(14,9),sharex=True,
                      gridspec_kw={'height_ratios':[1,1,1.5,1.5]})
ax_rfc,ax_rhc,ax_rfh,ax_rhh=axes
for ax,axh,leg,gm in[(ax_rfc,ax_rfh,'RF','G2'),(ax_rhc,ax_rhh,'RH','G3')]:
    r=contact_results[leg]
    ta=r['t_analysis']; cva=r['c_vi_analysis'].astype(float); cga=r['c_gmo_analysis'].astype(float)
    _shade(ax,ta,cva,C_VI,0.35); _shade(ax,ta,cga,C_GMO,0.25,hatch='//')
    ax.set_ylabel(leg,rotation=0,labelpad=25,fontsize=10,fontweight='bold')
    ax.set_ylim(-0.1,1.1); ax.set_yticks([])
    ax.axvline(T_END,color='gray',ls=':',lw=1)
    ax.set_title(f'{leg}: Acc={r["accuracy"]*100:.1f}% Prec={r["precision"]*100:.1f}% '
                 f'Rec={r["recall"]*100:.1f}% F1={r["f1"]:.4f} Lat={r["mean_latency_ms"]:.1f}ms',
                 fontsize=8,loc='left')
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=C_VI,alpha=0.4,label='VICON truth'),
                        Patch(color=C_GMO,alpha=0.3,label='GMO estimate',hatch='//')],
              loc='upper right',fontsize=7)
    axh.plot(vi.t_traj[mask_win_vi],vi.foot_heights[leg][mask_win_vi]*1000,
             lw=0.8,color='#555',label=f'{leg} height')
    axh.axhline(CONTACT_THRESHOLD_M*1000,color='red',ls='--',lw=1,
                label=f'thr {CONTACT_THRESHOLD_M*1000:.0f}mm')
    _shade(axh,vi.t_traj[mask_win_vi],r['region_mask_full'][mask_win_vi].astype(float),
           C_REG,0.15)
    axh.set_ylabel(f'{leg} Z [mm]',fontsize=8); axh.legend(loc='upper right',fontsize=7)
    axh.grid(True,alpha=0.3); axh.axvline(T_END,color='gray',ls=':',lw=1)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL}({DATE}) — Step 1: Contact Detection VICON vs GMO\n'
             f'Region: union(groundA,groundB)  Window:[0,{T_END:.1f}]s  Thr:{CONTACT_THRESHOLD_M*1000:.0f}mm')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_contact_timeseries.png'),dpi=150); plt.close()
print('Saved: fig_contact_timeseries.png')

# Plot 1b: region map
fig2,ax2=plt.subplots(figsize=(8,6))
from matplotlib.patches import Polygon as MplPoly
for pts,lbl,col in[(pts_A,'groundA','#1E88E5'),(pts_B,'groundB','#E53935')]:
    ch=ConvexHull(pts)
    ax2.add_patch(MplPoly(pts[ch.vertices]/1000,closed=True,fill=True,alpha=0.15,
                           edgecolor=col,facecolor=col,lw=1.5,label=f'{lbl} region'))
    for i,(x,y) in enumerate(pts,1):
        ax2.scatter(x/1000,y/1000,s=60,color=col,zorder=3)
        ax2.annotate(f'{lbl[-1]}{i}',(x/1000,y/1000),textcoords='offset points',xytext=(4,4),fontsize=8)
for leg,col in[('RF','#FF5722'),('RH','#9C27B0')]:
    r=contact_results[leg]; fxy=r['foot_xy_robot']
    v=~np.isnan(fxy[:,0]); win=mask_win_vi&v
    ax2.plot(fxy[win,0]/1000,fxy[win,1]/1000,lw=0.7,alpha=0.7,color=col,label=f'{leg} foot')
    ir=r['region_mask_full']&win
    ax2.scatter(fxy[ir,0]/1000,fxy[ir,1]/1000,s=2,color=col,alpha=0.4)
vm=vi.valid_hip&mask_win_vi
ax2.plot(vi.pos_m[vm,0],vi.pos_m[vm,1],lw=1.5,color='k',label='Robot centroid')
ax2.scatter(vi.pos_m[vm,0][0],vi.pos_m[vm,1][0],s=80,color='k',marker='>',zorder=3,label='Start')
ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]')
ax2.set_aspect('equal'); ax2.legend(fontsize=8); ax2.grid(True,alpha=0.3)
ax2.set_title(f'{TRIAL} — Top View: Ground Regions & Foot Trajectories')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_region_map.png'),dpi=150); plt.close()
print('Saved: fig_region_map.png')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 2: Inner EKF Analysis
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 2: Inner EKF Analysis')

err_x=epx-vi_px_e; err_y=epy-vi_py_e; err_z=epz-vi_pz_e
err_3d=np.sqrt(err_x**2+err_y**2+err_z**2); vp=~np.isnan(err_3d)
m21={'RMSE_X':rmse(err_x[vp]),'RMSE_Y':rmse(err_y[vp]),'RMSE_Z':rmse(err_z[vp]),
     'RMSE_3D':rmse(err_3d[vp]),'MAX_3D':float(np.max(err_3d[vp])),
     'final_EKF':(float(epx[-1]),float(epy[-1])),
     'final_VICON':(float(vi_px_e[vp][-1]),float(vi_py_e[vp][-1]))}
# Velocity RMSE restricted to t=12-17s (VICON data stable in this window)
_vw=(et>=12.)&(et<=17.)
m22={'RMSE_vx':rmse(evx[_vw]-vi_vx_e[_vw]),'RMSE_vy':rmse(evy[_vw]-vi_vy_e[_vw]),
     'RMSE_vz':rmse(evz[_vw]-vi_vz_e[_vw]),'peak_fwd':float(np.nanmax(evx)),
     'vel_window':'12-17s'}
m23={'RMSE_roll_deg':rmse(rpy_ekf_deg[:,0]-vi_roll_e),
     'RMSE_pitch_deg':rmse(rpy_ekf_deg[:,1]-vi_pitch_e),
     'RMSE_yaw_deg':rmse(rpy_ekf_deg[:,2]-vi_yaw_e),
     'final_yaw_EKF':float(rpy_ekf_deg[-1,2]),
     'final_yaw_VICON':float(vi_yaw_e[vp][-1]) if vp.any() else float('nan')}

ba_mask=(ba['t']>=0.)&(ba['t']<=T_END)
bw_mask=(bw['t']>=0.)&(bw['t']<=T_END)
m24={}; m25={}
for bdict,bmask,mout in[(ba,ba_mask,m24),(bw,bw_mask,m25)]:
    for ax_name in['x','y','z']:
        v=bdict[ax_name][bmask]
        if len(v)==0: continue
        mout[ax_name]={'init':float(v[0]),'ss':float(v[-len(v)//4:].mean()),
                       'std':float(v[-len(v)//4:].std())}

print(f'  2.1 Pos RMSE — X={m21["RMSE_X"]*100:.1f}cm Y={m21["RMSE_Y"]*100:.1f}cm '
      f'Z={m21["RMSE_Z"]*100:.1f}cm 3D={m21["RMSE_3D"]*100:.1f}cm MAX={m21["MAX_3D"]*100:.1f}cm')
print(f'  2.2 Vel RMSE (12-17s) — vx={m22["RMSE_vx"]:.3f} vy={m22["RMSE_vy"]:.3f} vz={m22["RMSE_vz"]:.3f} m/s')
print(f'  2.3 Att RMSE — roll={m23["RMSE_roll_deg"]:.2f}° pitch={m23["RMSE_pitch_deg"]:.2f}° yaw={m23["RMSE_yaw_deg"]:.2f}°')
print(f'  2.4 ba ss — x={m24["x"]["ss"]:.5f} y={m24["y"]["ss"]:.5f} z={m24["z"]["ss"]:.5f} m/s²')
print(f'  2.5 bw ss — x={m25["x"]["ss"]:.6f} y={m25["y"]["ss"]:.6f} z={m25["z"]["ss"]:.6f} rad/s')

# Plot 2a: XY trajectory
fig,axes=plt.subplots(1,2,figsize=(14,5))
ax=axes[0]
ax.plot(vi.pos_m[vi.valid_hip&mask_win_vi,0],vi.pos_m[vi.valid_hip&mask_win_vi,1],
        lw=1.5,label='VICON',color='#1E88E5')
ax.plot(epx,epy,lw=1.5,label='EKF',color='#FF5722',alpha=0.8)
ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
ax.set_aspect('equal'); ax.legend(); ax.grid(True,alpha=0.3)
ax.set_title(f'{TRIAL} — 2.1 EKF vs VICON XY')
ax=axes[1]; ax.plot(et,epz*100,label='EKF Z',lw=1.,color='#FF5722')
ax.plot(vi_t_v,pos_vicon[vi_valid,2]*100,label='VICON Z',lw=1.,color='#1E88E5',alpha=0.7)
ax.set_xlabel('Time [s]'); ax.set_ylabel('Z [cm]'); ax.legend(); ax.grid(True,alpha=0.3)
ax.set_title('2.1 Z vs Time')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_ekf_xy.png'),dpi=150); plt.close()
print('Saved: fig_ekf_xy.png')

# Plot 2b: position time series with 3σ
fig,axes=plt.subplots(3,1,figsize=(14,8),sharex=True)
for ax,lbl,ev,vv,cov in[(axes[0],'X[m]',epx,vi_px_e,ecov_px),
                         (axes[1],'Y[m]',epy,vi_py_e,ecov_py),
                         (axes[2],'Z[m]',epz,vi_pz_e,ecov_pz)]:
    ax.plot(et,ev,lw=1.,label='EKF',color='#FF5722')
    ax.plot(et,vv,lw=1.,label='VICON',color='#1E88E5',alpha=0.8)
    sig=np.sqrt(np.abs(cov))
    ax.fill_between(et,ev-3*sig,ev+3*sig,alpha=0.15,color='#FF5722',label='3σ')
    ax.set_ylabel(lbl); ax.legend(loc='upper right',fontsize=7); ax.grid(True,alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.1 EKF Position vs VICON (3σ bands)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_ekf_pos_time.png'),dpi=150); plt.close()
print('Saved: fig_ekf_pos_time.png')

# Plot 2c: velocity
fig,axes=plt.subplots(3,1,figsize=(14,7),sharex=True)
for ax,lbl,ev,vv in[(axes[0],'vx[m/s]',evx,vi_vx_e),
                     (axes[1],'vy[m/s]',evy,vi_vy_e),
                     (axes[2],'vz[m/s]',evz,vi_vz_e)]:
    ax.plot(et,ev,lw=0.8,label='EKF',color='#FF5722')
    ax.plot(et,vv,lw=0.8,label='VICON SG',color='#1E88E5',alpha=0.8)
    ax.set_ylabel(lbl); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.2 EKF Velocity vs VICON')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_ekf_vel.png'),dpi=150); plt.close()
print('Saved: fig_ekf_vel.png')

# Plot 2d: RPY
fig,axes=plt.subplots(3,1,figsize=(14,7),sharex=True)
for ax,lbl,ev,vv in[(axes[0],'Roll[°]',rpy_ekf_deg[:,0],vi_roll_e),
                     (axes[1],'Pitch[°]',rpy_ekf_deg[:,1],vi_pitch_e),
                     (axes[2],'Yaw[°]',rpy_ekf_deg[:,2],vi_yaw_e)]:
    ax.plot(et,ev,lw=0.8,label='EKF',color='#FF5722')
    ax.plot(et,vv,lw=0.8,label='VICON',color='#1E88E5',alpha=0.8)
    ax.set_ylabel(lbl); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.3 EKF Attitude (RPY) vs VICON')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_ekf_rpy.png'),dpi=150); plt.close()
print('Saved: fig_ekf_rpy.png')

# Plot 2e: bias
fig,axes=plt.subplots(2,3,figsize=(14,6))
for row,(bd,bmask,bname,unit) in enumerate([(ba,ba_mask,'ba(accel)','m/s²'),
                                              (bw,bw_mask,'bw(gyro)','rad/s')]):
    for col,ax_name in enumerate(['x','y','z']):
        ax=axes[row][col]
        ax.plot(bd['t'][bmask],bd[ax_name][bmask],lw=0.7)
        ax.set_title(f'{bname} {ax_name}'); ax.set_ylabel(unit); ax.grid(True,alpha=0.3)
        if row==1: ax.set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 2.4/2.5 IMU Bias Convergence')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_ekf_ba_bw.png'),dpi=150); plt.close()
print('Saved: fig_ekf_ba_bw.png')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 3: Outer Fusion Node
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 3: Outer Fusion Node')

om_mask=(odom['t']>=0.)&(odom['t']<=T_END)
ot=odom['t'][om_mask]; opx=odom['px'][om_mask]; opy=odom['py'][om_mask]; opz=odom['pz'][om_mask]
oqw=odom['qw'][om_mask]; oqx=odom['qx'][om_mask]; oqy=odom['qy'][om_mask]; oqz=odom['qz'][om_mask]
rpy_odom_deg=quat_to_rpy_deg(oqw,oqx,oqy,oqz)

vi_px_o=interp_to(vi_t_v,pos_vicon[vi_valid,0],ot)
vi_py_o=interp_to(vi_t_v,pos_vicon[vi_valid,1],ot)
ek_px_o=interp_to(et,epx,ot); ek_py_o=interp_to(et,epy,ot)
err_ov=np.sqrt((opx-vi_px_o)**2+(opy-vi_py_o)**2)
err_oe=np.sqrt((opx-ek_px_o)**2+(opy-ek_py_o)**2)
vo=~np.isnan(err_ov)
m31={'RMSE_2D_vs_VICON':rmse(err_ov[vo]),'MAX_2D_vs_VICON':float(np.max(err_ov[vo])),
     'RMSE_2D_vs_EKF':rmse(err_oe[vo]),
     'final_odom':(float(opx[-1]),float(opy[-1]))}

vi_yaw_o=interp_to(rpy_vi_t_v,np.degrees(rpy_vicon[rpy_vi_valid,2]),ot)
ek_yaw_o=interp_to(et,rpy_ekf_deg[:,2],ot)
vy=~np.isnan(vi_yaw_o)
m32={'RMSE_yaw_vs_VICON_deg':rmse(rpy_odom_deg[vy,2]-vi_yaw_o[vy]),
     'RMSE_yaw_vs_EKF_deg':rmse(rpy_odom_deg[vy,2]-ek_yaw_o[vy]),
     'final_yaw_odom':float(rpy_odom_deg[-1,2]),
     'final_yaw_vicon':float(vi_yaw_o[vy][-1]) if vy.any() else float('nan')}

# fusion/bv = velocity BIAS CORRECTION signal from outer fusion node
# (NOT a body velocity — it's the estimated leg-odometry velocity offset)
fv_mask=(fv['t']>=0.)&(fv['t']<=T_END)
ft=fv['t'][fv_mask]; fx=fv['x'][fv_mask]; fy=fv['y'][fv_mask]; fz=fv['z'][fv_mask]
fv_mag=np.sqrt(fx**2+fy**2+fz**2)
# For velocity quality: use odom_mapping twist (the actual fused velocity output)
ov_vx=odom['vx'][om_mask]; ov_vy=odom['vy'][om_mask]
vi_vx_om=interp_to(vi_t_v,v_body_vi[vi_valid,0],ot)
vi_vy_om=interp_to(vi_t_v,v_body_vi[vi_valid,1],ot)
ek_vx_om=interp_to(et,evx,ot); ek_vy_om=interp_to(et,evy,ot)
# Also compute leg-odometry error to show fusion/bv corrects it
ek_vx_err=interp_to(et,evx-vi_vx_e,ft)  # leg-odom vx error at bv timestamps
ek_vy_err=interp_to(et,evy-vi_vy_e,ft)
ek_vz_err=interp_to(et,evz-vi_vz_e,ft)
# Velocity RMSE for odom_mapping restricted to t=12-17s
_ovw=(ot>=12.)&(ot<=17.)
m33={'bv_mean_x':float(np.mean(fx)),'bv_mean_y':float(np.mean(fy)),'bv_mean_z':float(np.mean(fz)),
     'bv_mean_mag':float(np.mean(fv_mag)),'bv_max_mag':float(np.max(fv_mag)),
     'RMSE_omvx_vs_VICON_12_17':rmse(ov_vx[_ovw]-vi_vx_om[_ovw]),
     'RMSE_omvy_vs_VICON_12_17':rmse(ov_vy[_ovw]-vi_vy_om[_ovw]),
     'RMSE_omvx_vs_EKF_12_17':rmse(ov_vx[_ovw]-ek_vx_om[_ovw]),
     'vel_window':'12-17s'}

print(f'  3.1 RMSE 2D vs VICON={m31["RMSE_2D_vs_VICON"]*100:.1f}cm '
      f'vs EKF={m31["RMSE_2D_vs_EKF"]*100:.1f}cm MAX={m31["MAX_2D_vs_VICON"]*100:.1f}cm')
print(f'  3.2 Yaw RMSE vs VICON={m32["RMSE_yaw_vs_VICON_deg"]:.2f}°')
print(f'  3.3 fusion/bv correction: mean_x={m33["bv_mean_x"]:.4f} mean_y={m33["bv_mean_y"]:.4f} '
      f'mean_mag={m33["bv_mean_mag"]:.4f} m/s')
print(f'  3.3 odom_mapping vel RMSE (12-17s): vx={m33["RMSE_omvx_vs_VICON_12_17"]:.3f} '
      f'vy={m33["RMSE_omvy_vs_VICON_12_17"]:.3f} m/s vs VICON')

# Plot 3a: XY
fig,axes=plt.subplots(1,2,figsize=(14,5))
ax=axes[0]
ax.plot(vi.pos_m[vi.valid_hip&mask_win_vi,0],vi.pos_m[vi.valid_hip&mask_win_vi,1],
        lw=2,label='VICON',color='#1E88E5')
ax.plot(epx,epy,lw=1.2,label='EKF',color='#FF5722',alpha=0.8)
ax.plot(opx,opy,lw=1.2,label='odom_mapping',color='#43A047',alpha=0.8)
ax.set_xlabel('X[m]'); ax.set_ylabel('Y[m]'); ax.set_aspect('equal')
ax.legend(); ax.grid(True,alpha=0.3); ax.set_title(f'{TRIAL} — 3.1 XY Trajectories')
ax=axes[1]
ax.plot(et,epx,lw=1.,label='EKF X',color='#FF5722')
ax.plot(ot,opx,lw=1.,label='odom X',color='#43A047')
ax.plot(vi_t_v,pos_vicon[vi_valid,0],lw=1.,label='VICON X',color='#1E88E5',alpha=0.8)
ax.set_xlabel('Time [s]'); ax.set_ylabel('X [m]'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title('X vs Time')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_fusion_xy.png'),dpi=150); plt.close()
print('Saved: fig_fusion_xy.png')

# Plot 3b: fusion/bv as velocity bias correction + odom_mapping velocity vs VICON
fig,axes=plt.subplots(4,1,figsize=(14,10),sharex=True)
ax=axes[0]
ax.plot(ft,fx,lw=0.9,label='fusion/bv x (correction)',color='#43A047')
ax.plot(ft,ek_vx_err,lw=0.9,label='leg-odom vx error (EKF−VICON)',color='#FF5722',alpha=0.8,ls='--')
ax.axhline(0,color='gray',ls=':',lw=0.7)
ax.set_ylabel('vx [m/s]'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title('fusion/bv vx correction vs leg-odometry vx error (should correlate)',fontsize=8)
ax=axes[1]
ax.plot(ft,fy,lw=0.9,label='fusion/bv y (correction)',color='#43A047')
ax.plot(ft,ek_vy_err,lw=0.9,label='leg-odom vy error (EKF−VICON)',color='#FF5722',alpha=0.8,ls='--')
ax.axhline(0,color='gray',ls=':',lw=0.7)
ax.set_ylabel('vy [m/s]'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title('fusion/bv vy correction vs leg-odometry vy error',fontsize=8)
ax=axes[2]
ax.plot(ft,fz,lw=0.9,label='fusion/bv z (correction)',color='#43A047')
ax.plot(ft,ek_vz_err,lw=0.9,label='leg-odom vz error (EKF−VICON)',color='#FF5722',alpha=0.8,ls='--')
ax.axhline(0,color='gray',ls=':',lw=0.7)
ax.set_ylabel('vz [m/s]'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title('fusion/bv vz correction vs leg-odometry vz error',fontsize=8)
ax=axes[3]
ax.plot(ft,fv_mag,lw=0.9,color='#9C27B0',label='|fusion/bv| magnitude')
ax.fill_between(ft,0,fv_mag,alpha=0.15,color='#9C27B0')
ax.set_ylabel('|bv| [m/s]'); ax.set_xlabel('Time [s]'); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title(f'velocity bias correction magnitude  mean={m33["bv_mean_mag"]:.4f} m/s  max={m33["bv_max_mag"]:.4f} m/s',fontsize=8)
fig.suptitle(f'{TRIAL} — 3.3 fusion/bv: Velocity Bias Correction Signal')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_fusion_bv.png'),dpi=150); plt.close()
print('Saved: fig_fusion_bv.png')
# Plot 3b-2: odom_mapping velocity vs VICON (actual fused velocity quality)
fig,axes=plt.subplots(2,1,figsize=(14,6),sharex=True)
for ax,lbl,omv,viv,ekv in[(axes[0],'vx[m/s]',ov_vx,vi_vx_om,ek_vx_om),
                            (axes[1],'vy[m/s]',ov_vy,vi_vy_om,ek_vy_om)]:
    ax.plot(ot,omv,lw=0.8,label='odom_mapping twist',color='#43A047')
    ax.plot(ot,viv,lw=0.8,label='VICON SG',color='#1E88E5',alpha=0.8)
    ax.plot(ot,ekv,lw=0.8,label='EKF',color='#FF5722',alpha=0.7)
    ax.axvspan(12,17,color='yellow',alpha=0.12,label='RMSE window 12-17s')
    ax.set_ylabel(lbl); ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
axes[-1].set_xlabel('Time [s]')
fig.suptitle(f'{TRIAL} — 3.3 odom_mapping Velocity vs VICON vs EKF (12-17s window)')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_fusion_omvel.png'),dpi=150); plt.close()
print('Saved: fig_fusion_omvel.png')

# ═══════════════════════════════════════════════════════════════════════════════
# STEP 4: LiDAR Input Quality
# ═══════════════════════════════════════════════════════════════════════════════
print('\n'+'─'*60+'\nSTEP 4: LiDAR Input Quality')

l_mask=(lidar['t']>=0.)&(lidar['t']<=T_END)
lt=lidar['t'][l_mask]; lpx=lidar['px_odom'][l_mask]
lpy=lidar['py_odom'][l_mask]; lpz=lidar['pz_odom'][l_mask]

dt_l=np.diff(lt); dt_all_l=np.diff(lidar['t'])
m4={'n_msgs_total':int(np.sum(l_mask)),
    'mean_dt_ms':float(np.mean(dt_l)*1000) if len(dt_l) else float('nan'),
    'gaps_500ms':int(np.sum(dt_all_l>0.5))}
dp_l=np.sqrt(np.diff(lpx)**2+np.diff(lpy)**2)
m4['jumps_5cm']=int(np.sum(dp_l>0.05))
vi_px_l=interp_to(vi_t_v,pos_vicon[vi_valid,0],lt)
vi_py_l=interp_to(vi_t_v,pos_vicon[vi_valid,1],lt)
err_l=np.sqrt((lpx-vi_px_l)**2+(lpy-vi_py_l)**2)
vl=~np.isnan(err_l)
m4['XY_RMSE_vs_VICON']=rmse(err_l[vl]) if vl.any() else float('nan')
m4['Z_drift']=float(np.nanmax(np.abs(lpz))) if len(lpz)>0 else float('nan')

print(f'  count={m4["n_msgs_total"]} mean_dt={m4["mean_dt_ms"]:.1f}ms '
      f'gaps>500ms={m4["gaps_500ms"]} jumps>5cm={m4["jumps_5cm"]}')
print(f'  XY RMSE vs VICON={m4["XY_RMSE_vs_VICON"]*100:.1f}cm max|Z|={m4["Z_drift"]*100:.1f}cm')

# Plot 4a: LiDAR XY + Z
fig,axes=plt.subplots(1,2,figsize=(14,5))
ax=axes[0]
ax.plot(vi.pos_m[vi.valid_hip&mask_win_vi,0],vi.pos_m[vi.valid_hip&mask_win_vi,1],
        lw=2,label='VICON',color='#1E88E5')
ax.plot(lpx,lpy,lw=1.2,label='lidar_odom(odom)',color='#9C27B0',alpha=0.8)
ax.set_xlabel('X[m]'); ax.set_ylabel('Y[m]'); ax.set_aspect('equal')
ax.legend(); ax.grid(True,alpha=0.3); ax.set_title(f'{TRIAL} — 4 LiDAR XY')
ax=axes[1]; ax.plot(lt,lpz*100,lw=1.,color='#9C27B0')
ax.axhline(0,color='gray',ls='--',lw=0.8)
ax.set_xlabel('Time[s]'); ax.set_ylabel('Z[cm]'); ax.set_title('LiDAR Z drift')
ax.grid(True,alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_lidar_xy.png'),dpi=150); plt.close()
print('Saved: fig_lidar_xy.png')

# Plot 4b: message intervals
fig,ax=plt.subplots(figsize=(12,3))
if len(dt_all_l)>0:
    ax.plot(lidar['t'][1:],dt_all_l*1000,lw=0.7,color='#9C27B0')
    ax.axhline(100,color='green',ls='--',lw=0.8,label='100ms(10Hz)')
    ax.axhline(500,color='red',ls='--',lw=0.8,label='500ms gap thr')
    ax.axvspan(0,T_END,color='#E3F2FD',alpha=0.3,label='walk phase')
    ax.set_xlabel('Time[s]'); ax.set_ylabel('Interval[ms]')
    ax.legend(fontsize=7); ax.grid(True,alpha=0.3)
ax.set_title(f'{TRIAL} — 4 LiDAR Message Intervals')
plt.tight_layout()
plt.savefig(os.path.join(RESULTS,'fig_lidar_interval.png'),dpi=150); plt.close()
print('Saved: fig_lidar_interval.png')

# ─── Save metrics JSON for report ─────────────────────────────────────────────
metrics={'trial':TRIAL,'date':DATE,'T_END':T_END,
         'contact_threshold_m':CONTACT_THRESHOLD_M,
         'contact':{leg:{k:v for k,v in contact_results[leg].items()
                         if not isinstance(v,np.ndarray)} for leg in['RF','RH']},
         'ekf_pos':m21,'ekf_vel':m22,'ekf_att':m23,
         'ekf_ba':{ax:m24[ax] for ax in['x','y','z']},
         'ekf_bw':{ax:m25[ax] for ax in['x','y','z']},
         'fusion_pos':m31,'fusion_yaw':m32,'fusion_bv':m33,'lidar':m4,
         'T_CO_t':t_CO.tolist(),'T_CO_RPY_deg':rpy_CO.tolist()}
with open(os.path.join(RESULTS,'metrics.json'),'w') as f:
    json.dump(metrics,f,indent=2)
print('Saved: metrics.json')

print('\n'+'='*60)
print(f'FULL SUMMARY — {TRIAL} ({DATE})')
print(f'  Window: [0,{T_END:.2f}]s  Threshold:{CONTACT_THRESHOLD_M*1000:.0f}mm')
print(f'  Contact RF: Acc={contact_results["RF"]["accuracy"]*100:.1f}% '
      f'Prec={contact_results["RF"]["precision"]*100:.1f}% '
      f'Rec={contact_results["RF"]["recall"]*100:.1f}% '
      f'F1={contact_results["RF"]["f1"]:.4f}')
print(f'  Contact RH: Acc={contact_results["RH"]["accuracy"]*100:.1f}% '
      f'Prec={contact_results["RH"]["precision"]*100:.1f}% '
      f'Rec={contact_results["RH"]["recall"]*100:.1f}% '
      f'F1={contact_results["RH"]["f1"]:.4f}')
print(f'  EKF Pos 3D RMSE={m21["RMSE_3D"]*100:.1f}cm  Vel vx RMSE(12-17s)={m22["RMSE_vx"]:.3f}m/s')
print(f'  Fusion 2D RMSE={m31["RMSE_2D_vs_VICON"]*100:.1f}cm vs VICON  '
      f'({m31["RMSE_2D_vs_EKF"]*100:.1f}cm vs EKF)')
print(f'  LiDAR {m4["n_msgs_total"]} msgs  XY RMSE={m4["XY_RMSE_vs_VICON"]*100:.1f}cm')
print('Done.')
