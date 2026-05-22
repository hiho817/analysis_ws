#!/usr/bin/env python3
"""
CORGI Estimator Comparison: New (ESEKF, exp2/exp4) vs Old (Legacy, exp3/exp6)

Generates comparison figures and a Markdown report.
No ROS2 environment required — reads from metrics.json files.

Output (saved to ablation_result/):
  fig_compare_position.png
  fig_compare_velocity.png
  fig_compare_breakdown.png
  estimator_comparison_report.md
"""

import os, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE       = os.path.dirname(os.path.abspath(__file__))
OUT_DIR    = os.path.join(BASE, 'ablation_result')
os.makedirs(OUT_DIR, exist_ok=True)

# ─── Load metrics ─────────────────────────────────────────────────────────────
def load(exp_id):
    p = os.path.join(BASE, exp_id, 'result', 'metrics.json')
    with open(p) as f:
        return json.load(f)

m2 = load('exp2')   # new, plain
m3 = load('exp3')   # old, plain
m4 = load('exp4')   # new, obs
m6 = load('exp6')   # old, obs

def load_ablation(exp_id):
    p = os.path.join(BASE, 'ablation_result', f'{exp_id}_no_lidar_metrics.json')
    with open(p) as f:
        return json.load(f)

a2 = load_ablation('exp2')   # exp2 without LiDAR
a4 = load_ablation('exp4')   # exp4 without LiDAR

# ─── Helpers ──────────────────────────────────────────────────────────────────
COLORS = {
    'new_w':  '#2196F3',   # blue  — new ESEKF w/ bv feedback
    'new_wo': '#4CAF50',   # green — new ESEKF wo/ bv feedback
    'old':    '#F44336',   # red   — old Information Filter
}
LABEL_W  = 'NEW (ESEKF w/ bv feedback)'
LABEL_WO = 'NEW (ESEKF wo/ bv feedback)'
LABEL_OLD = 'OLD (Information Filter)'

def bar3(ax, labels, w_vals, wo_vals, old_vals, ylabel, title):
    x = np.arange(len(labels))
    w = 0.25
    b1 = ax.bar(x - w,   w_vals,  w, label=LABEL_W,   color=COLORS['new_w'],  alpha=0.85)
    b2 = ax.bar(x,       wo_vals, w, label=LABEL_WO,  color=COLORS['new_wo'], alpha=0.85)
    b3 = ax.bar(x + w,   old_vals,w, label=LABEL_OLD, color=COLORS['old'],    alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel(ylabel); ax.set_title(title)
    ax.legend(fontsize=7.5)
    for b in list(b1) + list(b2) + list(b3):
        ax.text(b.get_x() + b.get_width()/2, b.get_height() + 0.1,
                f'{b.get_height():.2f}', ha='center', va='bottom', fontsize=7)
    ax.grid(axis='y', alpha=0.3)

# ─── Figure 1: Position RMSE (3 bars: w/ bv, wo/ bv, OLD) ───────────────────
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
fig.suptitle('Position RMSE: New ESEKF vs Old (Information Filter)', fontsize=13, fontweight='bold')

# 3D RMSE
bar3(axes[0],
     ['plain\n(exp2/exp3)', 'obs\n(exp4/exp6)'],
     [m2['position']['RMSE_3D_cm'], m4['position']['RMSE_3D_cm']],
     [a2['position']['RMSE_3D_cm'], a4['position']['RMSE_3D_cm']],
     [m3['position']['RMSE_3D_cm'], m6['position']['RMSE_3D_cm']],
     'RMSE 3D [cm]', '3D Position RMSE')

# Plain X/Y/Z breakdown
cats = ['X', 'Y', 'Z']
bar3(axes[1],
     cats,
     [m2['position']['RMSE_X_cm'], m2['position']['RMSE_Y_cm'], m2['position']['RMSE_Z_cm']],
     [a2['position']['RMSE_X_cm'], a2['position']['RMSE_Y_cm'], a2['position']['RMSE_Z_cm']],
     [m3['position']['RMSE_X_cm'], m3['position']['RMSE_Y_cm'], m3['position']['RMSE_Z_cm']],
     'RMSE [cm]', 'Plain (exp2): X/Y/Z Breakdown')

# Obs X/Y/Z breakdown
bar3(axes[2],
     cats,
     [m4['position']['RMSE_X_cm'], m4['position']['RMSE_Y_cm'], m4['position']['RMSE_Z_cm']],
     [a4['position']['RMSE_X_cm'], a4['position']['RMSE_Y_cm'], a4['position']['RMSE_Z_cm']],
     [m6['position']['RMSE_X_cm'], m6['position']['RMSE_Y_cm'], m6['position']['RMSE_Z_cm']],
     'RMSE [cm]', 'Obs (exp4): X/Y/Z Breakdown')

plt.tight_layout()
out = os.path.join(OUT_DIR, 'fig_compare_position.png')
plt.savefig(out, dpi=150); plt.close()
print(f'Saved {out}')

# ─── Figure 2: Velocity RMSE ──────────────────────────────────────────────────
# Show exp2/exp4 (ESEKF) vs exp3/exp6 (Legacy); no ablation (vy not available)
vel_data = [
    ('exp2\n(ESEKF,plain)', m2['velocity']['RMSE_vx'], m2['velocity']['RMSE_vy'], COLORS['new_w']),
    ('exp4\n(ESEKF,obs)',   m4['velocity']['RMSE_vx'], m4['velocity']['RMSE_vy'], COLORS['new_w']),
    ('exp3\n(Legacy,plain)',m3['velocity']['RMSE_vx'], m3['velocity']['RMSE_vy'], COLORS['old']),
    ('exp6\n(Legacy,obs)',  m6['velocity']['RMSE_vx'], m6['velocity']['RMSE_vy'], COLORS['old']),
]
xlabels = [d[0] for d in vel_data]
vx_vals  = [d[1] for d in vel_data]
vy_vals  = [d[2] for d in vel_data]
bar_colors = [d[3] for d in vel_data]

fig, axes = plt.subplots(1, 2, figsize=(11, 5))
fig.suptitle('Velocity RMSE: New ESEKF vs Old (Information Filter)', fontsize=12, fontweight='bold')

for ax, vals, ylabel, title in [
        (axes[0], vx_vals, 'RMSE vx [m/s]', 'Forward Velocity (vx) RMSE'),
        (axes[1], vy_vals, 'RMSE vy [m/s]', 'Lateral Velocity (vy) RMSE')]:
    x = np.arange(len(xlabels))
    bars = ax.bar(x, vals, color=bar_colors, alpha=0.85, width=0.55)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width()/2, b.get_height() + 0.0005,
                f'{v:.4f}', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax.set_xticks(x); ax.set_xticklabels(xlabels, fontsize=9)
    ax.set_ylabel(ylabel); ax.set_title(title)
    ax.set_ylim(0, max(vals) * 1.35)
    ax.grid(axis='y', alpha=0.3)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=COLORS['new_w'], label=LABEL_W),
                        Patch(color=COLORS['old'],   label=LABEL_OLD)], fontsize=8)

plt.tight_layout()
out = os.path.join(OUT_DIR, 'fig_compare_velocity.png')
plt.savefig(out, dpi=150); plt.close()
print(f'Saved {out}')

# ─── Figure 3: Summary grouped bar ───────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle('New ESEKF vs Old (Information Filter) — Full Summary', fontsize=13, fontweight='bold')

exp_names  = ['exp2\n(new,plain)', 'exp4\n(new,obs)', 'exp3\n(old,plain)', 'exp6\n(old,obs)']
colors_bar = [COLORS['new_w'], COLORS['new_w'], COLORS['old'], COLORS['old']]
rmse3d = [m2['position']['RMSE_3D_cm'], m4['position']['RMSE_3D_cm'],
          m3['position']['RMSE_3D_cm'], m6['position']['RMSE_3D_cm']]
max3d  = [m2['position']['MAX_3D_cm'], m4['position']['MAX_3D_cm'],
          m3['position']['MAX_3D_cm'], m6['position']['MAX_3D_cm']]

x = np.arange(4)
b = axes[0].bar(x, rmse3d, color=colors_bar, alpha=0.85, width=0.5)
axes[0].set_xticks(x); axes[0].set_xticklabels(exp_names, fontsize=9)
axes[0].set_ylabel('RMSE 3D [cm]'); axes[0].set_title('Position RMSE 3D (all exps)')
for bar, v in zip(b, rmse3d):
    axes[0].text(bar.get_x()+bar.get_width()/2, v+0.1, f'{v:.2f}',
                 ha='center', va='bottom', fontsize=9, fontweight='bold')
axes[0].grid(axis='y', alpha=0.3)
# legend patch
from matplotlib.patches import Patch
axes[0].legend(handles=[Patch(color=COLORS['new_w'], label=LABEL_W),
                         Patch(color=COLORS['old'],   label=LABEL_OLD)])

rmse_vx = [m2['velocity']['RMSE_vx'], m4['velocity']['RMSE_vx'],
           m3['velocity']['RMSE_vx'], m6['velocity']['RMSE_vx']]
b2 = axes[1].bar(x, rmse_vx, color=colors_bar, alpha=0.85, width=0.5)
axes[1].set_xticks(x); axes[1].set_xticklabels(exp_names, fontsize=9)
axes[1].set_ylabel('RMSE vx [m/s]'); axes[1].set_title('Forward Velocity vx RMSE (all exps)')
for bar, v in zip(b2, rmse_vx):
    axes[1].text(bar.get_x()+bar.get_width()/2, v+0.001, f'{v:.4f}',
                 ha='center', va='bottom', fontsize=9)
axes[1].grid(axis='y', alpha=0.3)
axes[1].legend(handles=[Patch(color=COLORS['new_w'], label=LABEL_W),
                         Patch(color=COLORS['old'],   label=LABEL_OLD)])

plt.tight_layout()
out = os.path.join(OUT_DIR, 'fig_compare_summary.png')
plt.savefig(out, dpi=150); plt.close()
print(f'Saved {out}')

# fig_compare_ablation.png removed — ablation data merged into fig_compare_position.png

# ─── Compute averages ─────────────────────────────────────────────────────────
new_avg_pos  = (m2['position']['RMSE_3D_cm'] + m4['position']['RMSE_3D_cm']) / 2
old_avg_pos  = (m3['position']['RMSE_3D_cm'] + m6['position']['RMSE_3D_cm']) / 2
new_avg_vx   = (m2['velocity']['RMSE_vx']    + m4['velocity']['RMSE_vx'])    / 2
old_avg_vx   = (m3['velocity']['RMSE_vx']    + m6['velocity']['RMSE_vx'])    / 2

plain_improvement = (m3['position']['RMSE_3D_cm'] - m2['position']['RMSE_3D_cm']) / m3['position']['RMSE_3D_cm'] * 100
obs_improvement   = (m6['position']['RMSE_3D_cm'] - m4['position']['RMSE_3D_cm']) / m6['position']['RMSE_3D_cm'] * 100
avg_improvement   = (old_avg_pos - new_avg_pos) / old_avg_pos * 100

# Attitude (ESEKF only)
att_exp2 = m2['attitude']
att_exp4 = m4['attitude']

# odom_pos (ESEKF only)
odom_exp2 = m2['odom_pos']
odom_exp4 = m4['odom_pos']

# LiDAR ablation
lidar_imp_plain = (a2['position']['RMSE_3D_cm'] - m2['position']['RMSE_3D_cm']) / a2['position']['RMSE_3D_cm'] * 100
lidar_imp_obs   = (a4['position']['RMSE_3D_cm'] - m4['position']['RMSE_3D_cm']) / a4['position']['RMSE_3D_cm'] * 100
lidar_imp_avg   = ((a2['position']['RMSE_3D_cm'] + a4['position']['RMSE_3D_cm'])/2 - new_avg_pos) / ((a2['position']['RMSE_3D_cm'] + a4['position']['RMSE_3D_cm'])/2) * 100

# ─── Write Markdown report ────────────────────────────────────────────────────
report_path = os.path.join(OUT_DIR, 'estimator_comparison_report.md')
with open(report_path, 'w') as f:
    f.write(f"""# CORGI 估測器比較報告 — New ESEKF vs Old Legacy

**日期：** 2026-05-14
**比較對象：**
- **新估測器 (ESEKF)**：exp2（plain）、exp4（obs）
- **舊估測器 (Legacy Information Filter)**：exp3（plain）、exp6（obs）

---

## 實驗配置對應表

| 條件 | 新估測器 (ESEKF) | 舊估測器 (Legacy) |
|------|----------------|-----------------|
| plain odometry | **exp2** `walk_2m_01_plain_odometry` | **exp3** `walk_2m_01_plain_odometry_legacy` |
| obs odometry   | **exp4** `walk_2m_01_obs_odometry`   | **exp6** `walk_2m_01_obs_odometry_legacy`   |

---

## 1. 位置精度比較（Position RMSE）

![位置 RMSE 比較](fig_compare_position.png)
![總覽](fig_compare_summary.png)

### 1.1 3D 位置 RMSE

| 實驗 | 估測器 | 條件 | RMSE_X [cm] | RMSE_Y [cm] | RMSE_Z [cm] | **RMSE_3D [cm]** | MAX_3D [cm] |
|------|--------|------|------------|------------|------------|-----------------|------------|
| exp2 | **ESEKF** | plain | {m2['position']['RMSE_X_cm']:.2f} | {m2['position']['RMSE_Y_cm']:.2f} | {m2['position']['RMSE_Z_cm']:.2f} | **{m2['position']['RMSE_3D_cm']:.2f}** | {m2['position']['MAX_3D_cm']:.2f} |
| exp4 | **ESEKF** | obs   | {m4['position']['RMSE_X_cm']:.2f} | {m4['position']['RMSE_Y_cm']:.2f} | {m4['position']['RMSE_Z_cm']:.2f} | **{m4['position']['RMSE_3D_cm']:.2f}** | {m4['position']['MAX_3D_cm']:.2f} |
| exp3 | Legacy | plain | {m3['position']['RMSE_X_cm']:.2f} | {m3['position']['RMSE_Y_cm']:.2f} | {m3['position']['RMSE_Z_cm']:.2f} | **{m3['position']['RMSE_3D_cm']:.2f}** | {m3['position']['MAX_3D_cm']:.2f} |
| exp6 | Legacy | obs   | {m6['position']['RMSE_X_cm']:.2f} | {m6['position']['RMSE_Y_cm']:.2f} | {m6['position']['RMSE_Z_cm']:.2f} | **{m6['position']['RMSE_3D_cm']:.2f}** | {m6['position']['MAX_3D_cm']:.2f} |

### 1.2 同條件對比

| 條件 | ESEKF [cm] | Legacy [cm] | **改善幅度** |
|------|-----------|------------|------------|
| plain（exp2 vs exp3） | {m2['position']['RMSE_3D_cm']:.2f} | {m3['position']['RMSE_3D_cm']:.2f} | **+{plain_improvement:.1f}%** |
| obs（exp4 vs exp6）   | {m4['position']['RMSE_3D_cm']:.2f} | {m6['position']['RMSE_3D_cm']:.2f} | **+{obs_improvement:.1f}%** |
| **平均**             | **{new_avg_pos:.2f}** | **{old_avg_pos:.2f}** | **+{avg_improvement:.1f}%** |

**觀察：**
- plain 條件：ESEKF {m2['position']['RMSE_3D_cm']:.2f} cm vs Legacy {m3['position']['RMSE_3D_cm']:.2f} cm，**改善 {plain_improvement:.1f}%（約 {m3['position']['RMSE_3D_cm']-m2['position']['RMSE_3D_cm']:.2f} cm）**。
- obs 條件：ESEKF {m4['position']['RMSE_3D_cm']:.2f} cm vs Legacy {m6['position']['RMSE_3D_cm']:.2f} cm，**改善 {obs_improvement:.1f}%（約 {m6['position']['RMSE_3D_cm']-m4['position']['RMSE_3D_cm']:.2f} cm）**。
- 新估測器在 obs 條件下改善幅度更大，顯示 ESEKF 對 obs_odometry 補正資訊的整合更有效。
- Legacy 在 obs 條件下位置誤差顯著惡化（{m3['position']['RMSE_3D_cm']:.2f} → {m6['position']['RMSE_3D_cm']:.2f} cm），推測 obs 補正與 Information Filter 積分機制不相容。

---

## 2. 速度精度比較（Velocity RMSE）

![速度 RMSE 比較](fig_compare_velocity.png)

| 實驗 | 估測器 | 條件 | RMSE_vx [m/s] | RMSE_vy [m/s] | RMSE_vz [m/s] | peak_vx [m/s] |
|------|--------|------|--------------|--------------|--------------|--------------|
| exp2 | ESEKF | plain | {m2['velocity']['RMSE_vx']:.4f} | {m2['velocity']['RMSE_vy']:.4f} | {m2['velocity']['RMSE_vz']:.4f} | {m2['velocity']['peak_vx']:.3f} |
| exp4 | ESEKF | obs   | {m4['velocity']['RMSE_vx']:.4f} | {m4['velocity']['RMSE_vy']:.4f} | {m4['velocity']['RMSE_vz']:.4f} | {m4['velocity']['peak_vx']:.3f} |
| exp3 | Legacy | plain | {m3['velocity']['RMSE_vx']:.4f} | {m3['velocity']['RMSE_vy']:.4f} | {m3['velocity']['RMSE_vz']:.4f} | {m3['velocity']['peak_vx']:.3f} |
| exp6 | Legacy | obs   | {m6['velocity']['RMSE_vx']:.4f} | {m6['velocity']['RMSE_vy']:.4f} | {m6['velocity']['RMSE_vz']:.4f} | {m6['velocity']['peak_vx']:.3f} |

| 條件 | ESEKF vx [m/s] | Legacy vx [m/s] | Δ |
|------|---------------|----------------|---|
| plain（exp2 vs exp3） | {m2['velocity']['RMSE_vx']:.4f} | {m3['velocity']['RMSE_vx']:.4f} | {m2['velocity']['RMSE_vx']-m3['velocity']['RMSE_vx']:+.4f} |
| obs（exp4 vs exp6）   | {m4['velocity']['RMSE_vx']:.4f} | {m6['velocity']['RMSE_vx']:.4f} | {m4['velocity']['RMSE_vx']-m6['velocity']['RMSE_vx']:+.4f} |

**觀察：**
- 速度 RMSE 兩者相近（≈ 0.037–0.044 m/s），差異約 0.004–0.007 m/s。
- Legacy 的 vz RMSE 略高（~0.06 m/s vs ~0.025 m/s），顯示垂直方向速度估計稍差。
- 速度估計品質對估測器選擇敏感性較低，主要受步態特性影響。

---

## 3. ESEKF 專屬指標（exp2、exp4 可用）

### 3.1 姿態估計（Attitude）

| 實驗 | Roll RMSE [°] | Pitch RMSE [°] | Yaw RMSE [°] |
|------|--------------|---------------|-------------|
| exp2 | {att_exp2['RMSE_roll_deg']:.3f} | {att_exp2['RMSE_pitch_deg']:.3f} | {att_exp2['RMSE_yaw_deg']:.3f} |
| exp4 | {att_exp4['RMSE_roll_deg']:.3f} | {att_exp4['RMSE_pitch_deg']:.3f} | {att_exp4['RMSE_yaw_deg']:.3f} |

- Yaw RMSE：exp2 {att_exp2['RMSE_yaw_deg']:.3f}°，exp4 {att_exp4['RMSE_yaw_deg']:.3f}°，方向估計精確。
- Legacy 無姿態輸出，無法直接比較。

### 3.2 Outer Fusion 里程計（odom_mapping，僅 ESEKF）

| 實驗 | odom 2D RMSE vs VICON [cm] | odom Yaw RMSE vs VICON [°] |
|------|--------------------------|--------------------------|
| exp2 | {odom_exp2['RMSE_2D_vs_VICON_cm']:.3f} | {m2['odom_yaw']['RMSE_yaw_vs_VICON_deg']:.3f} |
| exp4 | {odom_exp4['RMSE_2D_vs_VICON_cm']:.3f} | {m4['odom_yaw']['RMSE_yaw_vs_VICON_deg']:.3f} |

- LiDAR 融合後（odom_mapping）位置精度 ≈ 2.0–2.4 cm，進一步優於 inner EKF。
- Legacy 無 LiDAR 融合節點，最終輸出即為 Information Filter 位置。

---

## 4. LiDAR 消融實驗（exp2、exp4：有 / 無 LiDAR 回授）

![LiDAR 消融](fig_compare_position.png)

> **消融設計：** 將 LiDAR（FAST-LIO2）關閉，僅依靠 ESEKF inner EKF + 腿部里程計維持定位，其他條件與原始實驗完全相同。

### 4.1 有 / 無 LiDAR 位置精度比較

| 實驗 | 條件 | 有 LiDAR（RMSE_3D） | 無 LiDAR（RMSE_3D） | LiDAR 帶來的改善 |
|------|------|-------------------|-------------------|----------------|
| exp2 | plain | **{m2['position']['RMSE_3D_cm']:.2f} cm** | {a2['position']['RMSE_3D_cm']:.2f} cm | **+{lidar_imp_plain:.1f}%（{a2['position']['RMSE_3D_cm']-m2['position']['RMSE_3D_cm']:.2f} cm）** |
| exp4 | obs   | **{m4['position']['RMSE_3D_cm']:.2f} cm** | {a4['position']['RMSE_3D_cm']:.2f} cm | **+{lidar_imp_obs:.1f}%（{a4['position']['RMSE_3D_cm']-m4['position']['RMSE_3D_cm']:.2f} cm）** |
| **平均** | — | **{new_avg_pos:.2f} cm** | {(a2['position']['RMSE_3D_cm']+a4['position']['RMSE_3D_cm'])/2:.2f} cm | **+{lidar_imp_avg:.1f}%** |

### 4.2 各軸誤差（無 LiDAR）

| 實驗 | RMSE_X [cm] | RMSE_Y [cm] | RMSE_Z [cm] | RMSE_3D [cm] | MAX_3D [cm] |
|------|------------|------------|------------|-------------|------------|
| exp2（無 LiDAR） | {a2['position']['RMSE_X_cm']:.2f} | {a2['position']['RMSE_Y_cm']:.2f} | {a2['position']['RMSE_Z_cm']:.2f} | {a2['position']['RMSE_3D_cm']:.2f} | {a2['position']['MAX_3D_cm']:.2f} |
| exp2（有 LiDAR） | {m2['position']['RMSE_X_cm']:.2f} | {m2['position']['RMSE_Y_cm']:.2f} | {m2['position']['RMSE_Z_cm']:.2f} | {m2['position']['RMSE_3D_cm']:.2f} | {m2['position']['MAX_3D_cm']:.2f} |
| exp4（無 LiDAR） | {a4['position']['RMSE_X_cm']:.2f} | {a4['position']['RMSE_Y_cm']:.2f} | {a4['position']['RMSE_Z_cm']:.2f} | {a4['position']['RMSE_3D_cm']:.2f} | {a4['position']['MAX_3D_cm']:.2f} |
| exp4（有 LiDAR） | {m4['position']['RMSE_X_cm']:.2f} | {m4['position']['RMSE_Y_cm']:.2f} | {m4['position']['RMSE_Z_cm']:.2f} | {m4['position']['RMSE_3D_cm']:.2f} | {m4['position']['MAX_3D_cm']:.2f} |

### 4.3 速度與 Yaw（無 LiDAR vs 有 LiDAR）

| 實驗 | 狀態 | RMSE_vx [m/s] | Yaw RMSE [°] |
|------|------|--------------|-------------|
| exp2 | 有 LiDAR | {m2['velocity']['RMSE_vx']:.4f} | {att_exp2['RMSE_yaw_deg']:.3f} |
| exp2 | 無 LiDAR | {a2['velocity']['RMSE_vx']:.4f} | {a2['attitude']['RMSE_yaw_deg']:.3f} |
| exp4 | 有 LiDAR | {m4['velocity']['RMSE_vx']:.4f} | {att_exp4['RMSE_yaw_deg']:.3f} |
| exp4 | 無 LiDAR | {a4['velocity']['RMSE_vx']:.4f} | {a4['attitude']['RMSE_yaw_deg']:.3f} |

**觀察：**
- LiDAR 融合使 3D 位置 RMSE 從 ~12.2 cm 降至 ~4.7 cm，平均改善約 {lidar_imp_avg:.0f}%。
- 速度 RMSE 幾乎不受 LiDAR 影響（差異 < 0.004 m/s），顯示速度估計主要由 IMU + 腿部里程計決定。
- Yaw RMSE 同樣穩定（~0.38–0.39°），LiDAR 對方向估計改善有限，方向主要由 IMU 陀螺儀維持。
- plain（exp2）與 obs（exp4）在無 LiDAR 條件下精度相近（12.25 vs 12.08 cm），驗證兩種 odometry 模式的基礎精度相當。

---

## 5. 總結

| 面向 | 新估測器 ESEKF | 舊估測器 Legacy | **結論** |
|------|-------------|--------------|--------|
| 位置 RMSE 3D（plain） | **{m2['position']['RMSE_3D_cm']:.2f} cm** | {m3['position']['RMSE_3D_cm']:.2f} cm | ESEKF 優 {plain_improvement:.0f}% |
| 位置 RMSE 3D（obs） | **{m4['position']['RMSE_3D_cm']:.2f} cm** | {m6['position']['RMSE_3D_cm']:.2f} cm | ESEKF 優 {obs_improvement:.0f}% |
| 位置 RMSE 3D（平均） | **{new_avg_pos:.2f} cm** | {old_avg_pos:.2f} cm | **ESEKF 整體優 {avg_improvement:.0f}%** |
| 速度 vx RMSE | {new_avg_vx:.4f} m/s | {old_avg_vx:.4f} m/s | 相近，無顯著差異 |
| obs 補正相容性 | ✅ 穩定（exp4 ≈ exp2） | ❌ 退步（exp6 vs exp3 +{m6['position']['RMSE_3D_cm']-m3['position']['RMSE_3D_cm']:.2f} cm） | ESEKF 更相容 |
| LiDAR 融合 | ✅ 有（2.0–2.4 cm after fusion） | ❌ 無 | ESEKF 附加優勢 |
| 姿態輸出 | ✅ Roll/Pitch/Yaw | ❌ 無 | ESEKF 附加優勢 |

### LiDAR 消融貢獻（ESEKF）

| 狀態 | plain（exp2） | obs（exp4） | 平均 |
|------|-------------|-----------|------|
| 有 LiDAR | **{m2['position']['RMSE_3D_cm']:.2f} cm** | **{m4['position']['RMSE_3D_cm']:.2f} cm** | **{new_avg_pos:.2f} cm** |
| 無 LiDAR | {a2['position']['RMSE_3D_cm']:.2f} cm | {a4['position']['RMSE_3D_cm']:.2f} cm | {(a2['position']['RMSE_3D_cm']+a4['position']['RMSE_3D_cm'])/2:.2f} cm |
| LiDAR 改善 | +{lidar_imp_plain:.1f}% | +{lidar_imp_obs:.1f}% | **+{lidar_imp_avg:.1f}%** |

**主要結論：**
1. **新估測器（ESEKF）在位置精度上全面優於舊估測器（Legacy Information Filter）**：plain 條件改善 {plain_improvement:.0f}%，obs 條件改善 {obs_improvement:.0f}%。
2. 舊估測器在 obs_odometry 條件下出現顯著退步（+{m6['position']['RMSE_3D_cm']-m3['position']['RMSE_3D_cm']:.2f} cm），新估測器不受此影響。
3. **LiDAR 回授對 ESEKF 位置精度至關重要**：移除 LiDAR 後誤差從 ~{new_avg_pos:.2f} cm 升至 ~{(a2['position']['RMSE_3D_cm']+a4['position']['RMSE_3D_cm'])/2:.2f} cm（平均 +{lidar_imp_avg:.0f}%），但仍可維持基本行走功能。
4. 速度與方向估計對 LiDAR 有無及估測器種類均不敏感，由 IMU + 腿部里程計主導。
5. 新估測器額外提供姿態輸出與 LiDAR 融合，具備更完整的狀態估計能力。
""")

print(f'Saved {report_path}')

print()
print('='*60)
print(f'ESTIMATOR COMPARISON SUMMARY')
print('='*60)
print(f"{'Exp':<8} {'Estimator':<10} {'Cond':<8} {'RMSE_3D [cm]':<15} {'vx RMSE [m/s]'}")
print('-'*60)
for m, name, est, cond in [(m2,'exp2','ESEKF','plain'), (m4,'exp4','ESEKF','obs'),
                             (m3,'exp3','Legacy','plain'), (m6,'exp6','Legacy','obs')]:
    print(f"{name:<8} {est:<10} {cond:<8} {m['position']['RMSE_3D_cm']:<15.2f} {m['velocity']['RMSE_vx']:.4f}")
print('='*60)
print(f"ESEKF avg 3D RMSE : {new_avg_pos:.2f} cm")
print(f"Legacy avg 3D RMSE: {old_avg_pos:.2f} cm")
print(f"Improvement       : {avg_improvement:.1f}%")
