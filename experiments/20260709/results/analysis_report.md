# CORGI 實驗分析報告 — 20260709

**日期：** 2026-07-09
**實驗地形：** Rugged / obstacle terrain（崎嶇與障礙地形）
**有效實驗數：** 19
**分析腳本：** `analyze.py`

---

## 實驗架構

```
/imu_raw, /motor/state ──► corgi_leg_odom ──► Inner EKF (/ekf)
                                                      │
/lidar_odom (FAST-LIO2) ──────────────► corgi_fusion_node ──► /odom_mapping
                                                              /fusion/bv

Legacy system: /odometry/legacy/position, /odometry/legacy/velocity
```

**實驗分組：**

| 分組代碼 | 模式 | 里程計系統 | 試驗數 |
|----------|------|-----------|--------|
| NEW_RUGG_WALK | RUGG Walk（崎嶇地面步行） | ESEKF + fusion | 4 |
| OLD_RUGG_WALK | RUGG Walk（崎嶇地面步行） | Legacy | 5 |
| NEW_OBS_MPC_GMO | Obstacle MPC（障礙地形） | ESEKF + fusion | 5 |
| OLD_OBS_MPC | Obstacle MPC（障礙地形） | Legacy | 5 |

---

## 1. 每次試驗結果

位置誤差單位為 cm；速度誤差單位為 m/s。位置使用 VICON 與估測器的有效重疊區間；速度使用 `35%–75% T_END` 穩態窗。

| 實驗編號 | 分組 | 有效資料 (s) | 位置 X | 位置 Y | 位置 Z | 位置 3D | 速度 X | 速度 Y | 速度 Z | 速度 3D |
|----------|------|--------------|--------|--------|--------|---------|--------|--------|--------|---------|
| RUGG_Walk_NEW_REAL_1 | NEW_RUGG_WALK | 0.0–32.2 | 2.36 | 5.10 | 3.04 | 6.38 | 0.055 | 0.060 | 0.023 | 0.085 |
| RUGG_Walk_NEW_REAL_2 | NEW_RUGG_WALK | 0.0–32.2 | 3.67 | 4.81 | 3.56 | 7.02 | 0.069 | 0.060 | 0.029 | 0.096 |
| RUGG_Walk_NEW_REAL_3 | NEW_RUGG_WALK | 0.0–33.5 | 4.02 | 8.32 | 5.43 | 10.72 | 0.052 | 0.061 | 0.023 | 0.083 |
| RUGG_Walk_NEW_REAL_5 | NEW_RUGG_WALK | 0.0–34.4 | 2.10 | 6.67 | 2.64 | 7.47 | 0.076 | 0.062 | 0.027 | 0.101 |
| RUGG_Walk_OLD_REAL_1 | OLD_RUGG_WALK | 0.0–34.3 | 28.57 | 35.84 | 1.38 | 45.86 | 0.079 | 0.069 | 0.067 | 0.124 |
| RUGG_Walk_OLD_REAL_2 | OLD_RUGG_WALK | 0.0–32.1 | 30.31 | 28.01 | 1.75 | 41.30 | 0.071 | 0.062 | 0.064 | 0.114 |
| RUGG_Walk_OLD_REAL_3 | OLD_RUGG_WALK | 0.0–32.7 | 34.56 | 33.55 | 1.55 | 48.19 | 0.073 | 0.076 | 0.069 | 0.126 |
| RUGG_Walk_OLD_REAL_4 | OLD_RUGG_WALK | 0.0–35.4 | 35.39 | 30.48 | 1.58 | 46.73 | 0.074 | 0.075 | 0.068 | 0.126 |
| RUGG_Walk_OLD_REAL_5 | OLD_RUGG_WALK | 0.0–34.3 | 29.75 | 29.34 | 1.59 | 41.81 | 0.074 | 0.071 | 0.064 | 0.121 |
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 0.0–46.2 | 2.38 | 6.02 | 5.75 | 8.66 | 0.041 | 0.055 | 0.024 | 0.073 |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 0.0–45.4 | 1.50 | 4.45 | 1.95 | 5.08 | 0.038 | 0.052 | 0.023 | 0.068 |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 0.0–47.3 | 2.96 | 12.73 | 5.70 | 14.26 | 0.038 | 0.053 | 0.029 | 0.071 |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 0.0–44.8 | 1.58 | 9.76 | 2.11 | 10.11 | 0.033 | 0.046 | 0.022 | 0.061 |
| OBS_MPC_NEW_REAL_7 | NEW_OBS_MPC_GMO | 0.0–48.2 | 2.95 | 5.63 | 8.76 | 10.82 | 0.047 | 0.047 | 0.024 | 0.071 |
| OBS_MPC_OLD_REAL_1 | OLD_OBS_MPC | 0.0–38.8 | 27.89 | 16.43 | 9.40 | 33.70 | 0.056 | 0.055 | 0.045 | 0.090 |
| OBS_MPC_OLD_REAL_2 | OLD_OBS_MPC | 0.0–38.4 | 19.38 | 6.15 | 3.20 | 20.58 | 0.040 | 0.052 | 0.056 | 0.086 |
| OBS_MPC_OLD_REAL_3 | OLD_OBS_MPC | 0.0–37.2 | 18.21 | 6.94 | 1.46 | 19.54 | 0.040 | 0.047 | 0.046 | 0.077 |
| OBS_MPC_OLD_REAL_4 | OLD_OBS_MPC | 0.0–35.9 | 23.07 | 5.70 | 1.20 | 23.79 | 0.038 | 0.045 | 0.045 | 0.074 |
| OBS_MPC_OLD_REAL_5 | OLD_OBS_MPC | 0.0–40.1 | 21.80 | 6.40 | 1.58 | 22.77 | 0.036 | 0.044 | 0.048 | 0.074 |

---

## 2. 分組統計（平均 ± 樣本標準差）

| 模式 | 系統 | n | Pos X (cm) | Pos Y (cm) | Pos Z (cm) | Pos 3D (cm) | Vel X (m/s) | Vel Y (m/s) | Vel Z (m/s) | Vel 3D (m/s) |
|------|------|---|------------|------------|------------|-------------|-------------|-------------|-------------|--------------|
| RUGG Walk（崎嶇地面步行） | ESEKF + fusion | 4 | 3.035 ± 0.951 | 6.223 ± 1.620 | 3.669 ± 1.235 | 7.899 ± 1.932 | 0.063 ± 0.011 | 0.061 ± 0.001 | 0.025 ± 0.003 | 0.091 ± 0.009 |
| RUGG Walk（崎嶇地面步行） | Legacy | 5 | 31.715 ± 3.057 | 31.446 ± 3.199 | 1.570 ± 0.129 | 44.781 ± 3.062 | 0.074 ± 0.003 | 0.071 ± 0.006 | 0.066 ± 0.002 | 0.122 ± 0.005 |
| Obstacle MPC（障礙地形） | ESEKF + fusion | 5 | 2.277 ± 0.712 | 7.717 ± 3.436 | 4.852 ± 2.861 | 9.787 ± 3.337 | 0.040 ± 0.005 | 0.050 ± 0.004 | 0.024 ± 0.003 | 0.069 ± 0.005 |
| Obstacle MPC（障礙地形） | Legacy | 5 | 22.066 ± 3.777 | 8.325 ± 4.555 | 3.368 ± 3.460 | 24.077 ± 5.641 | 0.042 ± 0.008 | 0.049 ± 0.005 | 0.048 ± 0.005 | 0.080 ± 0.007 |

---

## 3. NEW vs OLD 比較（位置與速度 3D RMSE）

### 3.1 位置 3D RMSE

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|------------|-------------|----------|
| RUGG Walk | 7.90 ± 1.93 | 44.78 ± 3.06 | +82.4% |
| Obstacle MPC | 9.79 ± 3.34 | 24.08 ± 5.64 | +59.4% |

### 3.2 速度 3D RMSE

| 模式 | ESEKF (m/s) | Legacy (m/s) | 改善幅度 |
|------|-------------|--------------|----------|
| RUGG Walk | 0.091 ± 0.009 | 0.122 ± 0.005 | +25.2% |
| Obstacle MPC | 0.069 ± 0.005 | 0.080 ± 0.007 | +14.6% |

> 改善幅度為 `(Legacy − ESEKF) / Legacy × 100%`。

---

## 4. ESEKF 系統詳細指標

### 4.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |
|----------|----------|-----------|---------|
| RUGG_Walk_NEW_REAL_1 | 0.746 | 0.568 | 0.259 |
| RUGG_Walk_NEW_REAL_2 | 1.611 | 0.735 | 0.317 |
| RUGG_Walk_NEW_REAL_3 | 1.053 | 1.329 | 0.620 |
| RUGG_Walk_NEW_REAL_5 | 0.933 | 0.348 | 0.665 |
| OBS_MPC_NEW_REAL_3 | 1.679 | 1.226 | 0.490 |
| OBS_MPC_NEW_REAL_4 | 0.743 | 0.720 | 0.483 |
| OBS_MPC_NEW_REAL_5 | 3.659 | 1.826 | 2.705 |
| OBS_MPC_NEW_REAL_6 | 2.959 | 1.051 | 1.433 |
| OBS_MPC_NEW_REAL_7 | 0.958 | 2.161 | 0.700 |

### 4.2 odom_mapping 位置 RMSE

| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|-------------|-------------|--------------|
| RUGG_Walk_NEW_REAL_1 | 1.75 | 4.78 | 5.09 |
| RUGG_Walk_NEW_REAL_2 | 1.61 | 5.24 | 5.48 |
| RUGG_Walk_NEW_REAL_3 | 2.01 | 4.69 | 5.10 |
| RUGG_Walk_NEW_REAL_5 | 1.34 | 3.69 | 3.93 |
| OBS_MPC_NEW_REAL_3 | 0.88 | 4.27 | 4.36 |
| OBS_MPC_NEW_REAL_4 | 1.22 | 4.04 | 4.22 |
| OBS_MPC_NEW_REAL_5 | 2.45 | 4.03 | 4.72 |
| OBS_MPC_NEW_REAL_6 | 0.73 | 3.14 | 3.22 |
| OBS_MPC_NEW_REAL_7 | 1.77 | 3.75 | 4.15 |

### 4.3 LiDAR 輸入品質

| 實驗編號 | Rate (Hz) | 配準 residual mean (cm) | residual max (cm) |
|----------|-----------|-------------------------|-------------------|
| RUGG_Walk_NEW_REAL_1 | 10.00 | 0.83 | 3.13 |
| RUGG_Walk_NEW_REAL_2 | 9.99 | 1.15 | 9.79 |
| RUGG_Walk_NEW_REAL_3 | 9.99 | 0.88 | 3.45 |
| RUGG_Walk_NEW_REAL_5 | 10.00 | 0.90 | 4.60 |
| OBS_MPC_NEW_REAL_3 | 10.02 | 0.79 | 3.18 |
| OBS_MPC_NEW_REAL_4 | 9.99 | 0.82 | 5.22 |
| OBS_MPC_NEW_REAL_5 | 10.01 | 1.41 | 7.21 |
| OBS_MPC_NEW_REAL_6 | 10.01 | 0.70 | 2.46 |
| OBS_MPC_NEW_REAL_7 | 10.00 | 0.73 | 3.74 |

---

## 5. MPC 終點 X 位置分析（目標：3.0 m）

MPC 控制器以 X = 3 m 為目標停止。估測器 final X 是控制器的停止依據，VICON final X 是實際停止位置。

| 實驗編號 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | 停止誤差 VICON (cm) |
|----------|-------------------|-------------------|--------------|--------------------|
| OBS_MPC_NEW_REAL_3 | 2.998 | 2.997 | -0.2 | -0.3 |
| OBS_MPC_NEW_REAL_4 | 2.986 | 2.967 | -1.4 | -3.3 |
| OBS_MPC_NEW_REAL_5 | 3.002 | 2.951 | +0.2 | -4.9 |
| OBS_MPC_NEW_REAL_6 | 2.987 | 2.987 | -1.3 | -1.3 |
| OBS_MPC_NEW_REAL_7 | 2.977 | 2.926 | -2.3 | -7.4 |
| OBS_MPC_OLD_REAL_1 | 3.010 | 2.548 | +1.0 | -45.2 |
| OBS_MPC_OLD_REAL_2 | 3.003 | 2.682 | +0.3 | -31.8 |
| OBS_MPC_OLD_REAL_3 | 3.009 | 2.667 | +0.9 | -33.3 |
| OBS_MPC_OLD_REAL_4 | 3.005 | 2.623 | +0.5 | -37.7 |
| OBS_MPC_OLD_REAL_5 | 3.003 | 2.651 | +0.3 | -34.9 |

**統計摘要（目標 X = 3.0 m）**

| 系統 | n | 估測器 final X | 實際 VICON final X | VICON 停止誤差 (abs mean) |
|------|---|------------------|---------------------|-----------------------------|
| ESEKF (NEW) | 5 | 2.990 ± 0.010 m | 2.966 ± 0.028 m | 3.4 cm |
| Legacy (OLD) | 5 | 3.006 ± 0.003 m | 2.634 ± 0.053 m | 36.6 cm |

---

## 6. 時間對齊與資料品質

ROS bag 與 VICON 使用共同 trigger OFF 校正；LiDAR XYZ 完整轉換至 odom frame，且只使用 trigger 有效時間窗。

| 實驗編號 | 時間偏移 (s) |
|----------|--------------:|
| RUGG_Walk_NEW_REAL_1 | -2.055 |
| RUGG_Walk_NEW_REAL_2 | +0.002 |
| RUGG_Walk_NEW_REAL_3 | -2.039 |
| RUGG_Walk_NEW_REAL_5 | -0.006 |
| RUGG_Walk_OLD_REAL_1 | -0.031 |
| RUGG_Walk_OLD_REAL_2 | -2.064 |
| RUGG_Walk_OLD_REAL_3 | -2.068 |
| RUGG_Walk_OLD_REAL_4 | -0.849 |
| RUGG_Walk_OLD_REAL_5 | -0.179 |
| OBS_MPC_NEW_REAL_3 | -0.009 |
| OBS_MPC_NEW_REAL_4 | -0.276 |
| OBS_MPC_NEW_REAL_5 | -0.011 |
| OBS_MPC_NEW_REAL_6 | -0.003 |
| OBS_MPC_NEW_REAL_7 | -3.310 |
| OBS_MPC_OLD_REAL_1 | -0.188 |
| OBS_MPC_OLD_REAL_2 | -3.160 |
| OBS_MPC_OLD_REAL_3 | -0.915 |
| OBS_MPC_OLD_REAL_4 | -3.318 |
| OBS_MPC_OLD_REAL_5 | -0.007 |

---

## 7. Closed-Loop（MPC）vs Open-Loop（Walk）比較

本節比較 ESEKF 的 Closed-Loop Obstacle MPC 與 Open-Loop RUGG Walk。姿態數值是 EKF 相對 VICON 的估測 RMSE，不是機體相對水平面的實際震盪量。

| 指標 | Closed-Loop（MPC） | Open-Loop（RUGG Walk） |
|------|-------------------|------------------------|
| n（試驗數） | 5 | 4 |
| Position 3D RMSE (cm) | 9.79 ± 3.34 | 7.90 ± 1.93 |
| Velocity 3D RMSE (m/s) | 0.069 ± 0.005 | 0.091 ± 0.009 |
| Roll estimation RMSE (°) | 2.00 ± 1.27 | 1.09 ± 0.37 |
| Pitch estimation RMSE (°) | 1.40 ± 0.59 | 0.74 ± 0.42 |
| Yaw estimation RMSE (°) | 1.16 ± 0.95 | 0.47 ± 0.21 |
| peak vx EKF（35–75% T_END，m/s） | 0.395 ± 0.083 | 0.566 ± 0.036 |

### 7.1 分析

- Closed-Loop MPC 的位置 3D RMSE 為 9.79 cm；Open-Loop RUGG Walk 為 7.90 cm。
- 兩組地形與任務條件不同，因此本比較用於描述系統行為，不應解讀為單一控制器因素的因果效果。
- peak vx 可反映步態中的瞬時速度振盪；姿態 RMSE 則反映估測器追蹤 VICON 的一致性。

---

## 8. 觀察與結論

### 崎嶇地面步行（RUGG Walk）

- ESEKF 位置 3D RMSE 為 7.90 cm；Legacy 為 44.78 cm。
- 修正後 LiDAR XZ 高度與 VICON 地形起伏一致，先前的大幅 Z 漂移來自未轉換的 LiDAR Z 軸與錯誤時間窗。

### 障礙地形 MPC

- ESEKF 位置 3D RMSE 為 9.79 cm；Legacy 為 24.08 cm。
- MPC 終點表直接呈現估測器停止依據與 VICON 實際停止位置，可用來判斷里程計累積誤差是否導致提前停止。

### 整體結論

- 20260709 有效資料中，ESEKF + LiDAR fusion 的位置誤差低於 Legacy。
- 所有統計均來自 position、velocity、attitude、outer fusion 與 LiDAR 品質。

---

*報告由 `analyze.py --report-only` 從各試驗 metrics 重建。更新日期：2026-07-12*
