#!/usr/bin/env python3
"""
Ablation Comparison — 20260514 exp1-6

比較 plain_odometry vs obs_odometry 對 ESEKF 及 Legacy (Information Filter) 的效果。

Ablation 設計：
  ESEKF:  exp1,2 (plain) vs exp4,5 (obs)
  Legacy: exp3 (plain)   vs exp6 (obs)

輸出：
  fig_ablation_esekf_pos.png   — 位置 RMSE 比較
  fig_ablation_esekf_odom.png  — odom_mapping RMSE 比較
  fig_ablation_legacy_pos.png  — Legacy 位置 RMSE 比較
  fig_ablation_vel.png         — 速度 RMSE 比較
  ablation_metrics.json        — 彙整後的數字
  analysis_report.md           — 完整比較報告（繁體中文）
"""

import os, sys, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE    = os.path.dirname(os.path.abspath(__file__))
EXP_DIR = os.path.join(BASE, 'experiments', '20260514')
OUT_DIR = os.path.join(EXP_DIR, 'ablation_result')
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Load metrics ─────────────────────────────────────────────────────────────
def load(exp_id):
    p = os.path.join(EXP_DIR, exp_id, 'result', 'metrics.json')
    with open(p) as f:
        return json.load(f)

m = {i: load(f'exp{i}') for i in range(1, 7)}

# ─── Helpers ──────────────────────────────────────────────────────────────────
def bar_grouped(ax, labels, vals_A, vals_B, label_A, label_B, ylabel, title,
                color_A='steelblue', color_B='darkorange'):
    x = np.arange(len(labels)); w = 0.35
    bars_a = ax.bar(x - w/2, vals_A, w, label=label_A, color=color_A, alpha=0.85)
    bars_b = ax.bar(x + w/2, vals_B, w, label=label_B, color=color_B, alpha=0.85)
    for bar in list(bars_a) + list(bars_b):
        h = bar.get_height()
        if not np.isnan(h):
            ax.text(bar.get_x() + bar.get_width()/2., h + 0.02,
                    f'{h:.2f}', ha='center', va='bottom', fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel); ax.set_title(title)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3, axis='y')

# ═══════════════════════════════════════════════════════════════════════════════
# ESEKF Ablation — Position
# ═══════════════════════════════════════════════════════════════════════════════
# Average plain_odometry (exp1,exp2) vs obs_odometry (exp4,exp5)
plain_pos3d = np.mean([m[1]['position']['RMSE_3D_cm'], m[2]['position']['RMSE_3D_cm']])
plain_posX  = np.mean([m[1]['position']['RMSE_X_cm'],  m[2]['position']['RMSE_X_cm']])
plain_posY  = np.mean([m[1]['position']['RMSE_Y_cm'],  m[2]['position']['RMSE_Y_cm']])
obs_pos3d   = np.mean([m[4]['position']['RMSE_3D_cm'], m[5]['position']['RMSE_3D_cm']])
obs_posX    = np.mean([m[4]['position']['RMSE_X_cm'],  m[5]['position']['RMSE_X_cm']])
obs_posY    = np.mean([m[4]['position']['RMSE_Y_cm'],  m[5]['position']['RMSE_Y_cm']])

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].clear()  # skip the broken call, draw directly below
x = np.arange(4); w = 0.6
vals = [m[1]['position']['RMSE_3D_cm'], m[2]['position']['RMSE_3D_cm'],
        m[4]['position']['RMSE_3D_cm'], m[5]['position']['RMSE_3D_cm']]
colors = ['steelblue', 'steelblue', 'darkorange', 'darkorange']
bars = axes[0].bar(x, vals, w, color=colors, alpha=0.85)
for bar, v in zip(bars, vals):
    axes[0].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.05,
                 f'{v:.2f}', ha='center', va='bottom', fontsize=9)
axes[0].set_xticks(x)
axes[0].set_xticklabels(['exp1\n(plain)', 'exp2\n(plain)', 'exp4\n(obs)', 'exp5\n(obs)'])
axes[0].set_ylabel('RMSE 3D [cm]')
axes[0].set_title('Inner EKF 3D Position RMSE (ESEKF)')
axes[0].grid(True, alpha=0.3, axis='y')
from matplotlib.patches import Patch
axes[0].legend(handles=[Patch(color='steelblue', label='plain_odometry'),
                         Patch(color='darkorange', label='obs_odometry')], fontsize=9)

axes[1].clear()
bar_grouped(axes[1],
            ['X', 'Y', '3D'],
            [plain_posX, plain_posY, plain_pos3d],
            [obs_posX,   obs_posY,   obs_pos3d],
            'plain_odometry (avg exp1,2)',
            'obs_odometry (avg exp4,5)',
            'RMSE [cm]',
            'ESEKF Inner EKF Position RMSE (Averaged)')

fig.suptitle('Ablation: plain_odometry vs obs_odometry — ESEKF Inner EKF Position', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fig_ablation_esekf_pos.png'), dpi=150)
plt.close(fig)
print('Saved fig_ablation_esekf_pos.png')

# ═══════════════════════════════════════════════════════════════════════════════
# ESEKF Ablation — odom_mapping
# ═══════════════════════════════════════════════════════════════════════════════
plain_odom2d = np.mean([m[1]['odom_pos']['RMSE_2D_vs_VICON_cm'],
                        m[2]['odom_pos']['RMSE_2D_vs_VICON_cm']])
obs_odom2d   = np.mean([m[4]['odom_pos']['RMSE_2D_vs_VICON_cm'],
                        m[5]['odom_pos']['RMSE_2D_vs_VICON_cm']])
plain_yaw    = np.mean([m[1]['attitude']['RMSE_yaw_deg'],
                        m[2]['attitude']['RMSE_yaw_deg']])
obs_yaw      = np.mean([m[4]['attitude']['RMSE_yaw_deg'],
                        m[5]['attitude']['RMSE_yaw_deg']])

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
# Per-run odom
x = np.arange(4); w = 0.6
vals_odom = [m[1]['odom_pos']['RMSE_2D_vs_VICON_cm'], m[2]['odom_pos']['RMSE_2D_vs_VICON_cm'],
             m[4]['odom_pos']['RMSE_2D_vs_VICON_cm'], m[5]['odom_pos']['RMSE_2D_vs_VICON_cm']]
colors = ['steelblue', 'steelblue', 'darkorange', 'darkorange']
bars = axes[0].bar(x, vals_odom, w, color=colors, alpha=0.85)
for bar, v in zip(bars, vals_odom):
    axes[0].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.02,
                 f'{v:.2f}', ha='center', va='bottom', fontsize=9)
axes[0].set_xticks(x)
axes[0].set_xticklabels(['exp1\n(plain)', 'exp2\n(plain)', 'exp4\n(obs)', 'exp5\n(obs)'])
axes[0].set_ylabel('RMSE 2D [cm]')
axes[0].set_title('odom_mapping 2D RMSE vs VICON (per run)')
axes[0].legend(handles=[Patch(color='steelblue', label='plain_odometry'),
                         Patch(color='darkorange', label='obs_odometry')], fontsize=9)
axes[0].grid(True, alpha=0.3, axis='y')

bar_grouped(axes[1],
            ['odom 2D [cm]', 'EKF yaw [°]'],
            [plain_odom2d, plain_yaw],
            [obs_odom2d,   obs_yaw],
            'plain_odometry (avg exp1,2)',
            'obs_odometry (avg exp4,5)',
            'Error', 'Outer Fusion Metrics (Averaged)')

fig.suptitle('Ablation: plain_odometry vs obs_odometry — Outer Fusion Node', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fig_ablation_esekf_odom.png'), dpi=150)
plt.close(fig)
print('Saved fig_ablation_esekf_odom.png')

# ═══════════════════════════════════════════════════════════════════════════════
# Legacy Ablation — Position
# ═══════════════════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
vals_leg = [m[3]['position']['RMSE_3D_cm'], m[6]['position']['RMSE_3D_cm']]
x = np.arange(2); w = 0.4
colors_leg = ['steelblue', 'darkorange']
bars = axes[0].bar(x, vals_leg, w, color=colors_leg, alpha=0.85)
for bar, v in zip(bars, vals_leg):
    axes[0].text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.1,
                 f'{v:.2f}', ha='center', va='bottom', fontsize=10)
axes[0].set_xticks(x)
axes[0].set_xticklabels(['exp3\n(plain_legacy)', 'exp6\n(obs_legacy)'])
axes[0].set_ylabel('RMSE 3D [cm]')
axes[0].set_title('Legacy (Information Filter) 3D Position RMSE')
axes[0].grid(True, alpha=0.3, axis='y')

bar_grouped(axes[1],
            ['X', 'Y', '3D'],
            [m[3]['position']['RMSE_X_cm'], m[3]['position']['RMSE_Y_cm'],
             m[3]['position']['RMSE_3D_cm']],
            [m[6]['position']['RMSE_X_cm'], m[6]['position']['RMSE_Y_cm'],
             m[6]['position']['RMSE_3D_cm']],
            'plain_legacy (exp3)',
            'obs_legacy (exp6)',
            'RMSE [cm]', 'Legacy Position RMSE Components')

fig.suptitle('Ablation: plain vs obs — Information Filter (Legacy)', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fig_ablation_legacy_pos.png'), dpi=150)
plt.close(fig)
print('Saved fig_ablation_legacy_pos.png')

# ═══════════════════════════════════════════════════════════════════════════════
# Velocity comparison (all 6 experiments)
# ═══════════════════════════════════════════════════════════════════════════════
fig, ax = plt.subplots(figsize=(10, 5))
labels = [f'exp{i}' for i in range(1, 7)]
vel_rmse = [m[i]['velocity']['RMSE_vx'] for i in range(1, 7)]
colors_all = ['steelblue', 'steelblue', '#3a7ebf',
              'darkorange', 'darkorange', '#e07010']
bars = ax.bar(np.arange(6), vel_rmse, 0.5, color=colors_all, alpha=0.85)
for bar, v in zip(bars, vel_rmse):
    ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.001,
            f'{v:.3f}', ha='center', va='bottom', fontsize=9)
ax.set_xticks(np.arange(6))
ax.set_xticklabels(['exp1\nplain\nESEKF', 'exp2\nplain\nESEKF', 'exp3\nplain\nLegacy',
                    'exp4\nobs\nESEKF', 'exp5\nobs\nESEKF', 'exp6\nobs\nLegacy'])
ax.set_ylabel('RMSE vx [m/s]')
ax.set_title('Velocity vx RMSE vs VICON — All 6 Experiments')
ax.legend(handles=[Patch(color='steelblue', label='plain_odometry'),
                   Patch(color='darkorange', label='obs_odometry')], fontsize=9)
ax.grid(True, alpha=0.3, axis='y')
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, 'fig_ablation_vel.png'), dpi=150)
plt.close(fig)
print('Saved fig_ablation_vel.png')

# ═══════════════════════════════════════════════════════════════════════════════
# Save JSON summary
# ═══════════════════════════════════════════════════════════════════════════════
ablation = {
    'date': '20260514',
    'esekf': {
        'plain': {
            'exps': ['exp1', 'exp2'],
            'pos_3d_rmse_cm': [m[1]['position']['RMSE_3D_cm'], m[2]['position']['RMSE_3D_cm']],
            'pos_3d_avg_cm': plain_pos3d,
            'odom_2d_rmse_cm': [m[1]['odom_pos']['RMSE_2D_vs_VICON_cm'],
                                 m[2]['odom_pos']['RMSE_2D_vs_VICON_cm']],
            'odom_2d_avg_cm': plain_odom2d,
            'yaw_rmse_deg':   [m[1]['attitude']['RMSE_yaw_deg'], m[2]['attitude']['RMSE_yaw_deg']],
            'yaw_avg_deg':    plain_yaw,
            'vel_vx_rmse':    [m[1]['velocity']['RMSE_vx'], m[2]['velocity']['RMSE_vx']],
        },
        'obs': {
            'exps': ['exp4', 'exp5'],
            'pos_3d_rmse_cm': [m[4]['position']['RMSE_3D_cm'], m[5]['position']['RMSE_3D_cm']],
            'pos_3d_avg_cm': obs_pos3d,
            'odom_2d_rmse_cm': [m[4]['odom_pos']['RMSE_2D_vs_VICON_cm'],
                                 m[5]['odom_pos']['RMSE_2D_vs_VICON_cm']],
            'odom_2d_avg_cm': obs_odom2d,
            'yaw_rmse_deg':   [m[4]['attitude']['RMSE_yaw_deg'], m[5]['attitude']['RMSE_yaw_deg']],
            'yaw_avg_deg':    obs_yaw,
            'vel_vx_rmse':    [m[4]['velocity']['RMSE_vx'], m[5]['velocity']['RMSE_vx']],
        },
        'improvement': {
            'pos_3d_cm':    plain_pos3d - obs_pos3d,
            'odom_2d_cm':   plain_odom2d - obs_odom2d,
            'yaw_deg':      plain_yaw - obs_yaw,
        }
    },
    'legacy': {
        'plain': {
            'exp': 'exp3',
            'pos_3d_rmse_cm': m[3]['position']['RMSE_3D_cm'],
            'vel_vx_rmse': m[3]['velocity']['RMSE_vx'],
        },
        'obs': {
            'exp': 'exp6',
            'pos_3d_rmse_cm': m[6]['position']['RMSE_3D_cm'],
            'vel_vx_rmse': m[6]['velocity']['RMSE_vx'],
        },
        'improvement': {
            'pos_3d_cm': m[3]['position']['RMSE_3D_cm'] - m[6]['position']['RMSE_3D_cm'],
        }
    }
}

with open(os.path.join(OUT_DIR, 'ablation_metrics.json'), 'w') as f:
    json.dump(ablation, f, indent=2)
print('Saved ablation_metrics.json')

# ─── Print summary ────────────────────────────────────────────────────────────
print('\n' + '='*60)
print('ABLATION SUMMARY')
print('='*60)
print('\n[ESEKF Inner EKF Position 3D RMSE]')
print(f'  plain (exp1,2): {plain_pos3d:.2f} cm  (exp1={m[1]["position"]["RMSE_3D_cm"]:.2f}, exp2={m[2]["position"]["RMSE_3D_cm"]:.2f})')
print(f'  obs   (exp4,5): {obs_pos3d:.2f} cm  (exp4={m[4]["position"]["RMSE_3D_cm"]:.2f}, exp5={m[5]["position"]["RMSE_3D_cm"]:.2f})')
print(f'  Δ = {plain_pos3d - obs_pos3d:+.2f} cm')
print('\n[ESEKF odom_mapping 2D RMSE vs VICON]')
print(f'  plain (exp1,2): {plain_odom2d:.2f} cm')
print(f'  obs   (exp4,5): {obs_odom2d:.2f} cm')
print(f'  Δ = {plain_odom2d - obs_odom2d:+.2f} cm')
print('\n[ESEKF EKF Yaw RMSE]')
print(f'  plain (exp1,2): {plain_yaw:.2f}°')
print(f'  obs   (exp4,5): {obs_yaw:.2f}°')
print(f'  Δ = {plain_yaw - obs_yaw:+.2f}°')
print('\n[Legacy Position 3D RMSE]')
print(f'  plain (exp3): {m[3]["position"]["RMSE_3D_cm"]:.2f} cm')
print(f'  obs   (exp6): {m[6]["position"]["RMSE_3D_cm"]:.2f} cm')
print(f'  Δ = {m[3]["position"]["RMSE_3D_cm"] - m[6]["position"]["RMSE_3D_cm"]:+.2f} cm')
print('\n[Velocity vx RMSE - all runs]')
for i in range(1, 7):
    print(f'  exp{i}: {m[i]["velocity"]["RMSE_vx"]:.3f} m/s')
