#!/usr/bin/env python3
"""Analyse the clean fixed-seed RUGG Walk simulation comparison."""
from __future__ import annotations

import json
import sqlite3
import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time
from corgi_msgs.msg import TriggerStamped
from geometry_msgs.msg import Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

ROOT = Path('/home/hiho817/analysis_ws/thesis_exp')
SOURCE = ROOT / 'simulation/RUGG_WALK_SIM/RUGG_WALK_SIM_0.db3'
LEGACY = ROOT / 'simulation/RUGG_WALK_SIM_legacy_fixed42/RUGG_WALK_SIM_legacy_fixed42_0.db3'
OUT = ROOT / 'physical_exp/results/5.4_rugg_experiment'
FIG = OUT / 'figures'
sys.path.insert(0, str(ROOT / 'physical_exp/common'))
from thesis_figure_style import (  # noqa: E402
    create_three_panel, finish_figure, format_axis, plot_method, save_figure,
)
LABEL = 'walk'
LEGACY_Y_SIGN = 1.0

def rows(db, topic):
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    hit = con.execute('SELECT id FROM topics WHERE name=?', (topic,)).fetchone()
    if not hit: raise RuntimeError(f'missing {topic} in {db}')
    ans = con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (hit[0],)).fetchall()
    con.close(); return ans

def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def interp(t, x, q): return interp1d(t, x, axis=0, bounds_error=False, fill_value=np.nan)(q)
def rmse(x):
    x=np.asarray(x); return np.sqrt(np.nanmean(x*x, axis=0))

def triggers():
    v=[]
    for _, raw in rows(SOURCE, '/trigger'):
        m=deserialize_message(raw, TriggerStamped); v.append((bool(m.enable), stamp(m.header)))
    if len(v)!=2 or [a for a,_ in v] != [True,False]: raise RuntimeError(f'bad triggers: {v}')
    return v[0][1],v[1][1]

def clock_map():
    s=[]; c=[]
    for storage, raw in rows(SOURCE, '/clock'):
        m=deserialize_message(raw, Clock); s.append(storage*1e-9); c.append(m.clock.sec+m.clock.nanosec*1e-9)
    return np.asarray(s),np.asarray(c)

def raw_vec(topic, s_clock, c_clock, t0):
    t=[]; x=[]
    for storage, raw in rows(SOURCE, topic):
        m=deserialize_message(raw, Vector3); t.append(storage*1e-9); x.append([m.x,m.y,m.z])
    return interp(s_clock,c_clock,np.asarray(t))-t0,np.asarray(x)

def odom(db, topic, t0):
    t=[]; p=[]; v=[]; a=[]
    for _,raw in rows(db,topic):
        m=deserialize_message(raw,Odometry); q=m.pose.pose.orientation
        t.append(stamp(m.header)-t0); p.append([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])
        v.append([m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z])
        a.append(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz'))
    t,p,v,a=np.asarray(t),np.asarray(p),np.asarray(v),np.asarray(a)
    # Multiple callbacks can publish the same state timestamp.  Keep its last
    # value: it is the completed state for that simulation instant, and avoids
    # artificially weighting that instant several times in RMSE.
    keep=np.r_[np.diff(t) != 0.0, True]
    return t[keep],p[keep],v[keep],a[keep]

def legacy(t0):
    t=[]; p=[]; v=[]
    pr=rows(LEGACY,'/validation/legacy/position_stamped'); vr=rows(LEGACY,'/validation/legacy/velocity_stamped')
    if len(pr)!=len(vr): raise RuntimeError('legacy position/velocity count mismatch')
    for (_,rp),(_,rv) in zip(pr,vr):
        mp=deserialize_message(rp,Vector3Stamped); mv=deserialize_message(rv,Vector3Stamped)
        if abs(stamp(mp.header)-stamp(mv.header))>1e-6: raise RuntimeError('legacy timestamp mismatch')
        t.append(stamp(mp.header)-t0); p.append([mp.vector.x,LEGACY_Y_SIGN*mp.vector.y,mp.vector.z]); v.append([mv.vector.x,LEGACY_Y_SIGN*mv.vector.y,mv.vector.z])
    return np.asarray(t),np.asarray(p),np.asarray(v)

def gt_attitude(t0):
    t=[]; a=[]
    for _,raw in rows(SOURCE,'/tf'):
        m=deserialize_message(raw,TFMessage)
        for tr in m.transforms:
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                q=tr.transform.rotation; t.append(stamp(tr.header)-t0)
                a.append(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz'))
    if not t: raise RuntimeError('missing simulator odom->base_link TF')
    return np.asarray(t),np.asarray(a)

def assess(t,p,v,a,gt_t,gt_p,gt_v_t,gt_v,gt_a_t,gt_a, name):
    valid=(t>=0)&(t<=min(gt_t[-1],gt_v_t[-1],gt_a_t[-1]))
    t,p,v,a=t[valid],p[valid],v[valid],a[valid]
    gp,gv,ga=interp(gt_t,gt_p,t),interp(gt_v_t,gt_v,t),interp(gt_a_t,gt_a,t)
    n=min(500,len(t)); po=np.nanmean(p[:n]-gp[:n],axis=0); ao=np.nanmean(a[:n]-ga[:n],axis=0)
    pe=p-gp-po; ae=np.arctan2(np.sin(a-ga-ao),np.cos(a-ga-ao)); ve=v-gv
    pr=rmse(pe); vr=rmse(ve); ar=np.degrees(rmse(ae)); last=np.where(np.isfinite(pe).all(axis=1))[0][-1]
    return {'name':name,'t':t,'p':p-po,'v':v,'a':a-ao,'pe':pe,'ve':ve,
            'position_rmse_xyz_m':pr.tolist(),'position_rmse_3d_m':float(np.linalg.norm(pr)),
            'velocity_rmse_xyz_mps':vr.tolist(),'velocity_rmse_3d_mps':float(np.linalg.norm(vr)),
            'attitude_rmse_deg':ar.tolist(),'final_horizontal_error_m':float(np.linalg.norm(pe[last,:2])),
            'final_velocity_error_mps':ve[last].tolist(),'samples':int(len(t)),
            'dt_median_s':float(np.median(np.diff(t)))}

def plot(gt_t,gt_p,gt_v_t,gt_v,gt_a_t,gt_a, series, end, figure_kind='all'):
    def figure(kind, labels, truth, key, unit, filename, title):
        if figure_kind != 'all' and kind != figure_kind:
            return
        fig,ax=create_three_panel(title)
        truth_t = gt_t if key == 'p' else (gt_v_t if key == 'v' else gt_a_t)
        mask=(truth_t>=0)&(truth_t<=end)
        for i,(aa,label) in enumerate(zip(ax,labels)):
            plot_method(aa,truth_t[mask],truth[mask,i],'Ground Truth')
            # Range follows truth and proposed; drifting curves remain visible without controlling limits.
            proposed=next(s for s in series if s['name']=='Proposed Method')
            pm=(proposed['t']>=0)&(proposed['t']<=end); vals=np.r_[truth[mask,i],proposed[key][pm,i]]
            # Preserve the comparable scale while showing the legacy lateral-position error.
            if key == 'p' and i == 1:
                legacy=next(s for s in series if s['name']=='IF+KLD (Legacy)')
                lm=(legacy['t']>=0)&(legacy['t']<=end)
                vals=np.r_[vals,legacy['p'][lm,i]]
            pad=max(0.02,0.08*(np.nanmax(vals)-np.nanmin(vals))); ylim=(np.nanmin(vals)-pad,np.nanmax(vals)+pad)
            for s in series:
                if key not in s: continue
                sm=(s['t']>=0)&(s['t']<=end); value=s[key][sm,i]
                if key == 'a': value=np.degrees(value)
                method='IF+KLD' if s['name']=='IF+KLD (Legacy)' else s['name']
                plot_method(aa,s['t'][sm],value,method)
            format_axis(aa,label,xlim=(0,end),ylim=ylim)
        finish_figure(fig,ax); save_figure(fig,FIG/Path(filename).stem)
    figure('p',[r'$p_x$ [m]',r'$p_y$ [m]',r'$p_z$ [m]'],gt_p,'p','m',f'fig_rugg_sim_{LABEL}_position.png','Position Comparison')
    figure('v',[r'$v_x$ [m/s]',r'$v_y$ [m/s]',r'$v_z$ [m/s]'],gt_v,'v','m/s',f'fig_rugg_sim_{LABEL}_velocity.png','Velocity Comparison')
    figure('a',['Roll [deg]','Pitch [deg]','Yaw [deg]'],np.degrees(gt_a),'a','deg',f'fig_rugg_sim_{LABEL}_attitude.png','Attitude Comparison')

def main():
    global SOURCE, LEGACY, LABEL, LEGACY_Y_SIGN
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=SOURCE)
    parser.add_argument('--legacy',type=Path,default=LEGACY)
    parser.add_argument('--label',default=LABEL,choices=('walk','wlw'))
    parser.add_argument('--legacy-y-sign',type=float,default=LEGACY_Y_SIGN,choices=(-1.0,1.0))
    parser.add_argument('--plots-only',action='store_true')
    parser.add_argument('--figure', choices=('all', 'p'), default='all')
    args=parser.parse_args()
    SOURCE=args.source; LEGACY=args.legacy; LABEL=args.label; LEGACY_Y_SIGN=args.legacy_y_sign
    FIG.mkdir(parents=True,exist_ok=True); t0,t1=triggers(); sc,cc=clock_map()
    gpt,gp=raw_vec('/sim/position',sc,cc,t0); gvt,gv=raw_vec('/sim/body_velocity',sc,cc,t0); gat,ga=gt_attitude(t0)
    pt,pp,pv,pa=odom(SOURCE,'/ekf',t0); it,ip,iv,ia=odom(SOURCE,'/imu_only/ekf',t0); lt,lp,lv=legacy(t0)
    end=min(t1-t0,pt[-1],it[-1],lt[-1],gpt[-1],gat[-1])
    prop=assess(pt,pp,pv,pa,gpt,gp,gvt,gv,gat,ga,'Proposed Method')
    imu=assess(it,ip,iv,ia,gpt,gp,gvt,gv,gat,ga,'IMU Integration')
    # Legacy publishes no attitude; use position/velocity metrics only.
    lt,lp,lv=lt[(lt>=0)&(lt<=end)],lp[(lt>=0)&(lt<=end)],lv[(lt>=0)&(lt<=end)]
    lgtp,lgtv=interp(gpt,gp,lt),interp(gvt,gv,lt); n=min(500,len(lt)); lo=np.mean(lp[:n]-lgtp[:n],axis=0); le=lp-lgtp-lo; lve=lv-lgtv; pr=rmse(le);vr=rmse(lve)
    leg={'name':'IF+KLD (Legacy)','t':lt,'p':lp-lo,'v':lv,'position_rmse_xyz_m':pr.tolist(),'position_rmse_3d_m':float(np.linalg.norm(pr)),'velocity_rmse_xyz_mps':vr.tolist(),'velocity_rmse_3d_mps':float(np.linalg.norm(vr)),'samples':int(len(lt))}
    plot(gpt,gp,gvt,gv,gat,ga,[prop,leg,imu],end,args.figure)
    result={'source_bag':str(SOURCE),'legacy_bag':str(LEGACY),'legacy_y_sign':LEGACY_Y_SIGN,'analysis_window_s':[0.0,float(end)],'methods':{'proposed':{k:v for k,v in prop.items() if k not in ('t','p','v','a','pe','ve')},'legacy':{k:v for k,v in leg.items() if k not in ('t','p','v')},'imu_integration':{k:v for k,v in imu.items() if k not in ('t','p','v','a','pe','ve')}}}
    if not args.plots_only:
        (OUT/f'5.4_rugg_simulation_{LABEL}_metrics.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__': main()
