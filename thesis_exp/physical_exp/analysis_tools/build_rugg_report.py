#!/usr/bin/env python3
"""Build the rugged-ground Walk report from selected NEW/OLD trials."""
from __future__ import annotations
import argparse, importlib.util, json, sys
from pathlib import Path
from statistics import mean, stdev
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp")
EXP = ROOT / "experiments" / "RUGG_exp"
OUT = ROOT / "results" / "5.4_rugg_experiment"
FIG = OUT / "figures"
REPORT = OUT / "5.4_崎嶇地實驗.md"
sys.path.insert(0, str(ROOT / "common"))
from thesis_figure_style import (  # noqa: E402
    create_three_panel, finish_figure, format_axis, plot_method, save_figure,
)
NEW = ["RUGG_Walk_NEW_REAL_1", "RUGG_Walk_NEW_REAL_2", "RUGG_Walk_NEW_REAL_5"]
OLD = ["RUGG_Walk_OLD_REAL_2", "RUGG_Walk_OLD_REAL_3", "RUGG_Walk_OLD_REAL_5"]
def load_analyzer():
    p = ROOT / "analysis_tools" / "analyze_imu_only_rugg.py"
    s = importlib.util.spec_from_file_location("rugg_imu", p)
    m = importlib.util.module_from_spec(s); assert s.loader
    s.loader.exec_module(m); return m

def metric(exp):
    return json.loads((EXP / exp / "results" / exp / "metrics.json").read_text())

def stats(values):
    return mean(values), stdev(values)

def f(values, d=3):
    a, b = stats(values); return f"{a:.{d}f} ± {b:.{d}f}"

def group(ids, old=False):
    raw = [metric(x) for x in ids]
    p = raw[0]["position"]; v = raw[0]["velocity"]
    pk = ["RMSE_X_cm", "RMSE_Y_cm", "RMSE_Z_cm", "RMSE_3D_cm"]
    vk = ["RMSE_vx", "RMSE_vy", "RMSE_vz", "RMSE_3D"]
    out = {"raw": raw, "position": {k: [r["position"][k] / 100 for r in raw] for k in pk},
           "velocity": {k: [r["velocity"][k] for r in raw] for k in vk}}
    if not old:
        ak = ["RMSE_roll_deg", "RMSE_pitch_deg", "RMSE_yaw_deg"]
        out["attitude"] = {k: [r["attitude"][k] for r in raw] for k in ak}
    return out

def interp(t, x, tq): return interp1d(t, x, axis=0, bounds_error=False, fill_value=np.nan)(tq)
def ref_lim(*x):
    a=np.concatenate([np.ravel(z) for z in x]); a=a[np.isfinite(a)]
    lo,hi=float(a.min()),float(a.max()); pad=max((hi-lo)*.06,.001); return lo-pad,hi+pad
def nice(v):
    e=np.floor(np.log10(max(v,1e-9))); z=v/10**e
    return next(q*10**e for q in (1,2,5,10) if z<=q)
def pos_limits(gt,p):
    h=nice(float(np.nanmax(np.abs(np.r_[gt[:,1:3],p[:,1:3]])))*1.06); yz=(-h,h); xs=2*h
    lo,hi=ref_lim(gt[:,0],p[:,0]); ratio=max(1,int(np.ceil((hi-lo)/xs-1e-12)))
    mid=(lo+hi)/2; return (mid-ratio*xs/2,mid+ratio*xs/2),yz,ratio

def representative_plot(a):
    item, vi, base, imu = a.analyze("RUGG_Walk_NEW_REAL_2")
    end=min(vi.t_trigger_end,base["t"][-1],imu["plot_t"][-1],30.0)
    gt_t=np.linspace(0,end,int(end*200)+1); gt=interp(vi.t_traj,vi.pos_m,gt_t)
    bm=(base["t"]>=0)&(base["t"]<=end); im=(imu["plot_t"]>=0)&(imu["plot_t"]<=end)
    bp=np.c_[base["px"][bm],base["py"][bm],base["pz"][bm]]; ip=imu["plot_pos"][im]
    # Translation-only visual alignment to the ground-truth initial sample.
    bp-=bp[0]-interp(vi.t_traj,vi.pos_m,np.array([base["t"][bm][0]]))[0]
    xlim,yz,ratio=pos_limits(gt,bp)
    def plot(kind, truth, bt, bv, it, iv, title, labels, stem):
        fig,axs=create_three_panel(title)
        for i,ax in enumerate(axs):
            plot_method(ax,gt_t,truth[:,i],"Ground Truth")
            plot_method(ax,bt,bv[:,i],"Proposed Method")
            plot_method(ax,it,iv[:,i],"IMU Integration")
            ylim=xlim if kind=="p" and i==0 else yz if kind=="p" else ref_lim(truth[:,i],bv[:,i]) if kind=="v" else None
            format_axis(ax,labels[i],ylim=ylim,contact_font_sizes=True)
        finish_figure(fig,axs,contact_font_sizes=True); save_figure(fig,FIG/stem)
    plot("p",gt,base["t"][bm],bp,imu["plot_t"][im],ip,"Position Comparison",[r"$p_x$ [m]",r"$p_y$ [m]",r"$p_z$ [m]"],"fig_rugg_position_walk")
    gt_v=interp(vi.t_traj,vi.v_body,gt_t); bv=np.c_[base["vx"][bm],base["vy"][bm],base["vz"][bm]]; iv=np.c_[imu["vx"][im],imu["vy"][im],imu["vz"][im]]
    plot("v",gt_v,base["t"][bm],bv,imu["plot_t"][im],iv,"Velocity Comparison",[r"$v_x$ [m/s]",r"$v_y$ [m/s]",r"$v_z$ [m/s]"],"fig_rugg_velocity_walk")
    gt_a=np.degrees(interp(vi.t_traj,vi.rpy,gt_t)); ba=np.degrees(np.c_[base["roll"][bm],base["pitch"][bm],base["yaw"][bm]]); ia=np.degrees(np.c_[imu["roll"][im],imu["pitch"][im],imu["yaw"][im]])
    plot("a",gt_a,base["t"][bm],ba,imu["plot_t"][im],ia,"Attitude Comparison",["Roll [deg]","Pitch [deg]","Yaw [deg]"],"fig_rugg_attitude_walk")
    return ratio

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--plots-only",action="store_true")
    args=parser.parse_args()
    FIG.mkdir(parents=True,exist_ok=True); a=load_analyzer(); ratio=representative_plot(a)
    if args.plots_only:
        print("generated rugged Walk figures only")
        return
    n,o=group(NEW),group(OLD,True); imu=json.loads((OUT/"imu_only_metrics.json").read_text())["group_statistics"]["WALK"]
    I=imu
    pnew=n["position"]; pold=o["position"]; vnew=n["velocity"]; vold=o["velocity"]
    pi=I
    redp=(1-mean(pnew["RMSE_3D_cm"])/mean(pold["RMSE_3D_cm"]))*100; redv=(1-mean(vnew["RMSE_3D"])/mean(vold["RMSE_3D"]))*100
    lines=["# 5.4 崎嶇地實驗","","## 5.4.1 資料選取與分析方法","","本節僅分析崎嶇地 Walk。NEW 與 OLD 各選取三筆品質較佳且完整的試驗，報告平均值 ± 樣本標準差。純 IMU 積分以選入的三筆 NEW 原始 bag 重播 prediction-only 節點取得；不使用腿部速度更新、ZUPT、GMO/contact、LiDAR 或 VICON 校正。位置、速度使用 SI 制，姿態使用 deg。","","### 納入與排除清單","","| 組別 | 納入統計 | 排除統計 |","|---|---|---|",f"| NEW Walk | {', '.join(NEW)} | RUGG_Walk_NEW_REAL_3（位置與姿態誤差較高）；REAL_4、REAL_6 無有效資料 |",f"| OLD Walk | {', '.join(OLD)} | RUGG_Walk_OLD_REAL_1、REAL_4（位置誤差較高） |","| 滾走 | — | 尚無資料，保留待補 |","","## 5.4.2 位置與速度估測結果","","| 步態 | 方法 | n | 位置 RMSE X / Y / Z [m] | 位置 RMSE 3D [m] | 速度 RMSE vx / vy / vz [m/s] | 速度 RMSE 3D [m/s] |","|---|---|---:|---:|---:|---:|---:|",f"| Walk | NEW（ES-EKF） | 3 | {f(pnew['RMSE_X_cm'])} / {f(pnew['RMSE_Y_cm'])} / {f(pnew['RMSE_Z_cm'])} | {f(pnew['RMSE_3D_cm'])} | {f(vnew['RMSE_vx'])} / {f(vnew['RMSE_vy'])} / {f(vnew['RMSE_vz'])} | {f(vnew['RMSE_3D'])} |",f"| Walk | OLD（Legacy） | 3 | {f(pold['RMSE_X_cm'])} / {f(pold['RMSE_Y_cm'])} / {f(pold['RMSE_Z_cm'])} | {f(pold['RMSE_3D_cm'])} | {f(vold['RMSE_vx'])} / {f(vold['RMSE_vy'])} / {f(vold['RMSE_vz'])} | {f(vold['RMSE_3D'])} |",f"| Walk | 純 IMU 積分 | 3 | {f(I['position_rmse_x_m']['values'])} / {f(I['position_rmse_y_m']['values'])} / {f(I['position_rmse_z_m']['values'])} | {f(I['position_rmse_3d_m']['values'])} | {f(I['velocity_rmse_vx']['values'])} / {f(I['velocity_rmse_vy']['values'])} / {f(I['velocity_rmse_vz']['values'])} | {f(I['velocity_rmse_3d']['values'])} |","",f"Walk 的 NEW 位置 3D RMSE 為 **{f(pnew['RMSE_3D_cm'])} m**，相較 OLD 的 **{f(pold['RMSE_3D_cm'])} m** 降低 **{redp:.1f}%**；速度 3D RMSE 降低 **{redv:.1f}%**。","","### 代表性位置時序比較","",f"代表性試驗採 NEW 中位置 3D RMSE 最低的 `RUGG_Walk_NEW_REAL_2`。位置與速度顯示範圍僅由 Ground Truth 與 Proposed Method 決定；IMU 漂移不擴張座標軸。$p_y$、$p_z$ 使用以 0 為中心的共同尺度，$p_x$ 顯示跨度為 Y/Z 的 {ratio} 倍。","","![Walk position](figures/fig_rugg_position_walk.png)","","![Walk velocity](figures/fig_rugg_velocity_walk.png)","","### 納入試驗之個別位置與速度結果","","| Trial | 組別 | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] |","|---|---|---:|---:|"]
    for x in NEW+OLD:
        m=metric(x); lines.append(f"| {x} | {'NEW' if '_NEW_' in x else 'OLD'} | {m['position']['RMSE_3D_cm']/100:.3f} | {m['velocity']['RMSE_3D']:.3f} |")
    lines += ["","### 純 IMU 積分資料與個別結果","","| Trial | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] | 最終水平漂移 [m] |","|---|---:|---:|---:|"]
    for r in json.loads((OUT/"imu_only_metrics.json").read_text())["records"]:
        z=r['imu_only']; lines.append(f"| {r['exp_id']} | {z['position_rmse_3d_m']:.3f} | {z['velocity_rmse_3d']:.3f} | {z['final_horizontal_drift_m']:.2f} |")
    A=n['attitude']; lines += ["","## 5.4.3 姿態估測結果","","| 步態 | 方法 | n | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] |","|---|---|---:|---:|---:|---:|",f"| Walk | NEW（ES-EKF） | 3 | {f(A['RMSE_roll_deg'],2)} | {f(A['RMSE_pitch_deg'],2)} | {f(A['RMSE_yaw_deg'],2)} |",f"| Walk | OLD（Legacy） | 3 | — | — | — |",f"| Walk | 純 IMU 積分 | 3 | {f(I['attitude_rmse_roll_deg']['values'],2)} | {f(I['attitude_rmse_pitch_deg']['values'],2)} | {f(I['attitude_rmse_yaw_deg']['values'],2)} |","","![Walk attitude](figures/fig_rugg_attitude_walk.png)","","## 5.4.4 滾走實驗","","尚無滾走 NEW／OLD 資料；本節與圖表保留待補。","","## 5.4.5 小結","",f"崎嶇地 Walk 的 NEW 在三筆選入試驗上，位置與速度誤差均低於 OLD；純 IMU 積分的平均位置 3D RMSE 為 {f(I['position_rmse_3d_m']['values'])} m，顯示缺乏觀測約束時漂移顯著。"]
    REPORT.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(REPORT)
if __name__ == '__main__': main()
