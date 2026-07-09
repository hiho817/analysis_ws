# CORGI 實驗分析報告 — 20260709

**日期：** 2026-07-09
**實驗地點/模式：** Rugged ground walk、Obstacle MPC
**有效實驗數：** 18 / 23
**分析腳本：** `analysis_scripts/run_20260709_analysis.py`
**備註：** 本次依照要求未分析 `/gmo/contact_state` 觸地狀態。
**VICON 座標：** 20260709 CSV 未包含 ground markers，因此使用 VICON 原始 Z 軸作為垂直軸並以初始 O1-O4 建立 robot-centric frame。

---

## 1. 實驗分組

| 分組代碼 | 模式 | 系統 | 試驗數 |
|----------|------|------|--------|
| NEW_RUGG_WALK | Rugged walk | ESEKF + fusion | 4 |
| OLD_RUGG_WALK | Rugged walk | Legacy | 5 |
| NEW_OBS_MPC_GAIT | Obstacle MPC | ESEKF, gait feedback | 0 |
| NEW_OBS_MPC_GMO | Obstacle MPC | ESEKF, GMO feedback | 4 |
| OLD_OBS_MPC | Obstacle MPC | Legacy | 5 |

### 1.1 排除資料

以下資料保留原始 `metrics.json` 與圖表，但不納入本報告主要表格、分組統計、NEW/OLD 比較與終點統計。

| 實驗編號 | 分組 | 排除原因 |
|----------|------|----------|
| RUGG_Walk_NEW_REAL_4 | NEW_RUGG_WALK | 依使用者指定排除；T_CO residual max 偏高 |
| RUGG_Walk_NEW_REAL_6 | NEW_RUGG_WALK | 依使用者指定排除；位置與速度 RMSE 離群 |
| OBS_MPC_NEW_REAL_1 | NEW_OBS_MPC_GAIT | 依使用者指定排除；位置 RMSE 離群 |
| OBS_MPC_NEW_REAL_2 | NEW_OBS_MPC_GAIT | 依使用者指定排除；bag 未涵蓋停止點 |
| OBS_MPC_NEW_REAL_7 | NEW_OBS_MPC_GMO | 依使用者指定排除；位置 RMSE 離群 |

---

## 2. 每次試驗結果

位置誤差單位為 cm；速度誤差單位為 m/s。位置與速度沿用 20260528 的有效重疊區間與 `35%–75% T_END` 穩態速度窗。

| 實驗編號 | 分組 | 有效資料 (s) | 位置 X | 位置 Y | 位置 Z | 位置 3D | 速度 X | 速度 Y | 速度 Z | 速度 3D |
|----------|------|--------------|--------|--------|--------|---------|--------|--------|--------|---------|
| RUGG_Walk_NEW_REAL_1 | NEW_RUGG_WALK | 0.0-32.2 | 14.76 | 9.86 | 3.06 | 18.02 | 0.099 | 0.088 | 0.113 | 0.174 |
| RUGG_Walk_NEW_REAL_2 | NEW_RUGG_WALK | 0.0-32.2 | 3.68 | 4.81 | 3.72 | 7.11 | 0.069 | 0.061 | 0.028 | 0.096 |
| RUGG_Walk_NEW_REAL_3 | NEW_RUGG_WALK | 0.0-33.5 | 13.00 | 12.02 | 4.81 | 18.34 | 0.095 | 0.084 | 0.103 | 0.163 |
| RUGG_Walk_NEW_REAL_5 | NEW_RUGG_WALK | 0.0-34.4 | 2.08 | 6.69 | 2.86 | 7.57 | 0.074 | 0.058 | 0.034 | 0.100 |
| RUGG_Walk_OLD_REAL_1 | OLD_RUGG_WALK | 0.0-34.3 | 28.36 | 35.85 | 1.63 | 45.74 | 0.079 | 0.069 | 0.066 | 0.124 |
| RUGG_Walk_OLD_REAL_2 | OLD_RUGG_WALK | 0.0-32.1 | 17.07 | 28.07 | 1.88 | 32.90 | 0.070 | 0.062 | 0.063 | 0.113 |
| RUGG_Walk_OLD_REAL_3 | OLD_RUGG_WALK | 0.0-32.7 | 21.95 | 33.58 | 1.75 | 40.16 | 0.072 | 0.076 | 0.069 | 0.125 |
| RUGG_Walk_OLD_REAL_4 | OLD_RUGG_WALK | 0.0-35.4 | 30.15 | 30.49 | 1.94 | 42.93 | 0.073 | 0.075 | 0.068 | 0.125 |
| RUGG_Walk_OLD_REAL_5 | OLD_RUGG_WALK | 0.0-34.3 | 28.45 | 29.36 | 1.63 | 40.91 | 0.072 | 0.071 | 0.064 | 0.120 |
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 0.0-46.2 | 2.43 | 6.06 | 5.10 | 8.28 | 0.039 | 0.050 | 0.030 | 0.070 |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 0.0-45.4 | 2.83 | 4.89 | 1.61 | 5.87 | 0.089 | 0.089 | 0.074 | 0.146 |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 0.0-47.3 | 2.99 | 12.76 | 5.65 | 14.27 | 0.038 | 0.046 | 0.035 | 0.069 |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 0.0-44.8 | 1.60 | 9.77 | 2.25 | 10.15 | 0.033 | 0.045 | 0.023 | 0.060 |
| OBS_MPC_OLD_REAL_1 | OLD_OBS_MPC | 0.0-38.8 | 26.73 | 16.46 | 8.67 | 32.57 | 0.060 | 0.055 | 0.047 | 0.094 |
| OBS_MPC_OLD_REAL_2 | OLD_OBS_MPC | 0.0-38.4 | 17.90 | 7.98 | 2.95 | 19.82 | 0.070 | 0.087 | 0.079 | 0.137 |
| OBS_MPC_OLD_REAL_3 | OLD_OBS_MPC | 0.0-37.2 | 13.24 | 7.40 | 1.53 | 15.24 | 0.061 | 0.080 | 0.065 | 0.119 |
| OBS_MPC_OLD_REAL_4 | OLD_OBS_MPC | 0.0-35.9 | 16.56 | 8.82 | 1.54 | 18.82 | 0.064 | 0.079 | 0.072 | 0.124 |
| OBS_MPC_OLD_REAL_5 | OLD_OBS_MPC | 0.0-40.1 | 21.76 | 6.41 | 1.76 | 22.75 | 0.038 | 0.043 | 0.052 | 0.078 |

---

## 3. 分組統計（平均 ± 樣本標準差）

| 模式 | 系統 | n | Pos X (cm) | Pos Y (cm) | Pos Z (cm) | Pos 3D (cm) | Vel X (m/s) | Vel Y (m/s) | Vel Z (m/s) | Vel 3D (m/s) |
|------|------|---|------------|------------|------------|-------------|-------------|-------------|-------------|--------------|
| Rugged walk | ESEKF + fusion | 4 | 8.382 ± 6.425 | 8.346 ± 3.215 | 3.613 ± 0.877 | 12.760 ± 6.263 | 0.084 ± 0.015 | 0.073 ± 0.015 | 0.069 ± 0.045 | 0.133 ± 0.041 |
| Rugged walk | Legacy | 5 | 25.195 ± 5.518 | 31.472 ± 3.188 | 1.765 ± 0.143 | 40.529 ± 4.781 | 0.073 ± 0.004 | 0.071 ± 0.006 | 0.066 ± 0.003 | 0.121 ± 0.005 |
| Obstacle MPC | ESEKF, gait feedback | 0 | N/A ± N/A | N/A ± N/A | N/A ± N/A | N/A ± N/A | N/A ± N/A | N/A ± N/A | N/A ± N/A | N/A ± N/A |
| Obstacle MPC | ESEKF, GMO feedback | 4 | 2.461 ± 0.622 | 8.371 ± 3.593 | 3.652 ± 2.015 | 9.646 ± 3.548 | 0.050 ± 0.026 | 0.057 ± 0.021 | 0.041 ± 0.023 | 0.086 ± 0.040 |
| Obstacle MPC | Legacy | 5 | 19.238 ± 5.185 | 9.415 ± 4.035 | 3.288 ± 3.063 | 21.842 ± 6.570 | 0.058 ± 0.012 | 0.069 ± 0.019 | 0.063 ± 0.013 | 0.110 ± 0.024 |

---

## 4. NEW vs OLD 比較

### 4.1 位置 3D RMSE

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|------------|-------------|----------|
| RUGG Walk | 12.76 ± 6.26 | 40.53 ± 4.78 | +68.5% |
| OBS MPC | 9.65 ± 3.55 | 21.84 ± 6.57 | +55.8% |

### 4.2 速度 3D RMSE

| 模式 | ESEKF (m/s) | Legacy (m/s) | 改善幅度 |
|------|-------------|--------------|----------|
| RUGG Walk | 0.13 ± 0.04 | 0.12 ± 0.01 | -9.8% |
| OBS MPC | 0.086 ± 0.040 | 0.110 ± 0.024 | +21.9% |

---

## 5. ESEKF 詳細指標

### 5.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | 分組 | Roll (deg) | Pitch (deg) | Yaw (deg) |
|----------|------|------------|-------------|-----------|
| RUGG_Walk_NEW_REAL_1 | NEW_RUGG_WALK | 3.406 | 3.094 | 2.922 |
| RUGG_Walk_NEW_REAL_2 | NEW_RUGG_WALK | 1.609 | 0.731 | 0.315 |
| RUGG_Walk_NEW_REAL_3 | NEW_RUGG_WALK | 3.153 | 2.947 | 3.076 |
| RUGG_Walk_NEW_REAL_5 | NEW_RUGG_WALK | 0.945 | 0.379 | 0.670 |
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 1.686 | 1.240 | 0.501 |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 1.840 | 2.599 | 1.258 |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 3.663 | 1.834 | 2.708 |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 2.958 | 1.054 | 1.434 |

### 5.2 odom_mapping 位置 RMSE

| 實驗編號 | 分組 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|------|-------------|-------------|--------------|
| RUGG_Walk_NEW_REAL_1 | NEW_RUGG_WALK | 15.47 | 9.47 | 18.14 |
| RUGG_Walk_NEW_REAL_2 | NEW_RUGG_WALK | 1.62 | 5.25 | 5.49 |
| RUGG_Walk_NEW_REAL_3 | NEW_RUGG_WALK | 14.69 | 8.98 | 17.22 |
| RUGG_Walk_NEW_REAL_5 | NEW_RUGG_WALK | 1.33 | 3.71 | 3.95 |
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 0.94 | 4.32 | 4.42 |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 1.82 | 4.51 | 4.86 |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 2.42 | 4.07 | 4.74 |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 0.73 | 3.15 | 3.23 |

### 5.3 LiDAR/Fusion 品質摘要

| 實驗編號 | LiDAR rate (Hz) | T_CO residual mean (cm) | T_CO residual max (cm) | Fusion bv 3D RMSE (m/s) |
|----------|-----------------|-------------------------|------------------------|--------------------------|
| RUGG_Walk_NEW_REAL_1 | 10.00 | 0.83 | 3.13 | 0.153 |
| RUGG_Walk_NEW_REAL_2 | 9.99 | 1.15 | 9.79 | 0.144 |
| RUGG_Walk_NEW_REAL_3 | 9.99 | 0.88 | 3.45 | 0.145 |
| RUGG_Walk_NEW_REAL_5 | 10.00 | 0.90 | 4.60 | 0.141 |
| OBS_MPC_NEW_REAL_3 | 10.02 | 0.79 | 3.18 | 0.114 |
| OBS_MPC_NEW_REAL_4 | 9.99 | 0.82 | 5.22 | 0.116 |
| OBS_MPC_NEW_REAL_5 | 9.98 | 1.41 | 7.21 | 0.117 |
| OBS_MPC_NEW_REAL_6 | 10.01 | 0.70 | 2.46 | 0.113 |

---

## 6. OBS MPC 終點 X 位置

目標 X = 3.000 m。停止誤差以 VICON final X 相對目標計算。

| 實驗編號 | 分組 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | VICON 停止誤差 (cm) | 備註 |
|----------|------|-------------------|-------------------|--------------|--------------------|------|
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 2.998 | 2.997 | -0.2 | -0.3 |  |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 2.985 | 2.966 | -1.5 | -3.4 |  |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 3.001 | 2.951 | 0.1 | -4.9 |  |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 2.987 | 2.987 | -1.3 | -1.3 |  |
| OBS_MPC_OLD_REAL_1 | OLD_OBS_MPC | 3.010 | 2.548 | 1.0 | -45.2 |  |
| OBS_MPC_OLD_REAL_2 | OLD_OBS_MPC | 3.003 | 2.682 | 0.3 | -31.8 |  |
| OBS_MPC_OLD_REAL_3 | OLD_OBS_MPC | 3.009 | 2.667 | 0.9 | -33.3 |  |
| OBS_MPC_OLD_REAL_4 | OLD_OBS_MPC | 3.002 | 2.623 | 0.2 | -37.7 |  |
| OBS_MPC_OLD_REAL_5 | OLD_OBS_MPC | 3.003 | 2.651 | 0.3 | -34.9 |  |

### 6.1 統計摘要

| 系統 | n | 估測器 final X (m) | VICON final X (m) |
|------|---|-------------------|-------------------|
| ESEKF | 4 | 2.993 ± 0.008 | 2.975 ± 0.021 |
| Legacy | 5 | 3.005 ± 0.003 | 2.634 ± 0.053 |

---

## 7. 觀察與結論

- RUGG Walk：ESEKF 位置 3D RMSE 為 12.76 cm，Legacy 為 40.53 cm，ESEKF 在崎嶇地面仍明顯降低位置誤差。
- RUGG Walk 速度：ESEKF 速度 3D RMSE 為 0.133 m/s，Legacy 為 0.121 m/s。
- OBS MPC：ESEKF 位置 3D RMSE 平均 9.65 cm，Legacy 平均 21.84 cm；這反映障礙物任務中 LiDAR 融合仍能抑制腿式里程計累積誤差。
- OBS MPC 速度：ESEKF 速度 3D RMSE 平均 0.086 m/s，Legacy 平均 0.110 m/s。
- 本報告保留 gait feedback 與 GMO feedback 兩組 OBS_MPC_NEW 標籤，但不分析觸地狀態本身。

---

*報告由 `analysis_scripts/run_20260709_analysis.py` 產生；未執行觸地狀態分析。*
