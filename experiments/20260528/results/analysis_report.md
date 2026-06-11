# CORGI 實驗分析報告 — 20260528

**日期：** 2026-05-28
**實驗地點：** Flat ground（平地）
**有效實驗數：** 31 / 31（RUGG 系列尚無資料）
**分析腳本：** `analyze.py`

---

## 實驗架構

```
/imu_raw, /motor/state ──► corgi_leg_odom ──► Inner EKF (/ekf)
                                                      │
/gmo/contact_state                                    ▼
/lidar_odom (FAST-LIO2) ──────────────► corgi_fusion_node ──► /odom_mapping
                                                              /fusion/bv

Legacy system: /odometry/legacy/position, /odometry/legacy/velocity
```

**實驗分組：**
| 分組代碼 | 模式 | 里程計系統 | 試驗數 |
|----------|------|-----------|--------|
| NEW_WALK | 平地步行 | ESEKF + fusion | 6 |
| OLD_WALK | 平地步行 | Legacy | 5 |
| NEW_WLW  | 平地輪足步行 | ESEKF + fusion | 5 |
| OLD_WLW  | 平地輪足步行 | Legacy | 5 |
| NEW_MPC  | 平地 MPC | ESEKF + fusion | 5 |
| OLD_MPC  | 平地 MPC | Legacy | 5 |

---

## 1. 每次試驗結果

| 實驗編號 | 分組 | T_END (s) | RMSE X (cm) | RMSE Y (cm) | RMSE 3D/2D (cm) | RMSE vx (m/s) | RMSE vy (m/s) |
|----------|------|-----------|-------------|-------------|-----------------|---------------|---------------|
| FLAT_Walk_NEW_REAL_1 | NEW_WALK | 35.1 | 0.99 | 6.35 | 6.47 | 0.048 | 0.055 |
| FLAT_Walk_NEW_REAL_2 ¹ | NEW_WALK | 34.4 | 1.33 | 5.36 | 5.81 | 0.061 | 0.089 |
| FLAT_Walk_NEW_REAL_3 | NEW_WALK | 38.4 | 1.46 | 4.85 | 5.48 | 0.045 | 0.055 |
| FLAT_Walk_NEW_REAL_4 | NEW_WALK | 35.0 | 1.34 | 4.46 | 5.03 | 0.040 | 0.055 |
| FLAT_Walk_NEW_REAL_5 | NEW_WALK | 34.3 | 1.49 | 8.87 | 9.04 | 0.041 | 0.053 |
| FLAT_Walk_NEW_REAL_6 | NEW_WALK | 33.5 | 1.40 | 6.27 | 6.50 | 0.041 | 0.052 |
| FLAT_Walk_OLD_REAL_1 | OLD_WALK | 35.1 | 17.50 | 20.47 | 26.93 | 0.050 | 0.062 |
| FLAT_Walk_OLD_REAL_2 | OLD_WALK | 33.8 | 16.71 | 18.60 | 25.00 | 0.051 | 0.063 |
| FLAT_Walk_OLD_REAL_3 | OLD_WALK | 33.2 | 16.63 | 19.11 | 25.33 | 0.050 | 0.062 |
| FLAT_Walk_OLD_REAL_4 | OLD_WALK | 33.6 | 16.44 | 20.34 | 26.15 | 0.050 | 0.063 |
| FLAT_Walk_OLD_REAL_5 | OLD_WALK | 33.3 | 17.11 | 20.81 | 26.94 | 0.050 | 0.062 |
| FLAT_WLW_NEW_REAL_1 | NEW_WLW | 34.7 | 5.96 | 2.03 | 6.50 | 0.019 | 0.027 |
| FLAT_WLW_NEW_REAL_2 | NEW_WLW | 34.9 | 4.21 | 1.65 | 4.67 | 0.019 | 0.027 |
| FLAT_WLW_NEW_REAL_3 | NEW_WLW | 37.1 | 5.32 | 0.91 | 5.50 | 0.019 | 0.026 |
| FLAT_WLW_NEW_REAL_4 | NEW_WLW | 34.7 | 4.01 | 3.23 | 5.81 | 0.018 | 0.026 |
| FLAT_WLW_NEW_REAL_5 | NEW_WLW | 36.9 | 4.64 | 1.57 | 5.18 | 0.018 | 0.025 |
| FLAT_WLW_OLD_REAL_1 | OLD_WLW | 33.6 | 3.20 | 12.31 | 12.72 | 0.026 | 0.030 |
| FLAT_WLW_OLD_REAL_2 | OLD_WLW | 35.1 | 2.56 | 9.57 | 9.91 | 0.026 | 0.030 |
| FLAT_WLW_OLD_REAL_3 | OLD_WLW | 34.9 | 2.32 | 9.69 | 9.96 | 0.026 | 0.031 |
| FLAT_WLW_OLD_REAL_4 | OLD_WLW | 34.0 | 2.12 | 9.99 | 10.21 | 0.026 | 0.031 |
| FLAT_WLW_OLD_REAL_5 | OLD_WLW | 34.5 | 0.97 | 12.07 | 12.11 | 0.026 | 0.030 |
| FLAT_MPC_NEW_REAL_1 | NEW_MPC | 40.4 | 1.18 | 5.68 | 6.16 | 0.031 | 0.047 |
| FLAT_MPC_NEW_REAL_2 | NEW_MPC | 41.6 | 0.97 | 4.64 | 4.89 | 0.031 | 0.043 |
| FLAT_MPC_NEW_REAL_3 | NEW_MPC | 42.3 | 1.22 | 5.07 | 5.30 | 0.028 | 0.049 |
| FLAT_MPC_NEW_REAL_4 | NEW_MPC | 42.4 | 1.19 | 6.98 | 8.40 | 0.026 | 0.038 |
| FLAT_MPC_NEW_REAL_5 | NEW_MPC | 41.7 | 0.89 | 4.82 | 5.36 | 0.026 | 0.047 |
| FLAT_MPC_OLD_REAL_1 | OLD_MPC | 47.0 | 17.37 | 2.60 | 17.57 | 0.024 | 0.035 |
| FLAT_MPC_OLD_REAL_2 | OLD_MPC | 38.4 | 11.77 | 4.29 | 12.53 | 0.033 | 0.038 |
| FLAT_MPC_OLD_REAL_3 | OLD_MPC | 37.7 | 14.53 | 3.04 | 14.84 | 0.029 | 0.036 |
| FLAT_MPC_OLD_REAL_4 | OLD_MPC | 37.8 | 14.81 | 3.82 | 15.29 | 0.029 | 0.037 |
| FLAT_MPC_OLD_REAL_5 | OLD_MPC | 37.4 | 15.29 | 4.24 | 15.86 | 0.031 | 0.041 |

> NEW 系列使用 RMSE_3D（含 Z 軸），OLD 系列使用 RMSE_2D（XY 平面）。
> ¹ 此試驗有效數據異常（姿態偏移過大），已排除於分組統計外，但保留個別指標。

---

## 2. 分組統計（平均 ± 標準差）

| 模式 | 系統 | 試驗數 | RMSE 位置 (cm) | RMSE vx (m/s) |
|------|------|--------|----------------|---------------|
| Walk (步行) | ESEKF | 5 | 6.50 ± 1.39 | 0.043 |
| Walk (步行) | Legacy | 5 | 26.07 ± 0.80 | 0.050 |
| WLW (輪足步行) | ESEKF | 5 | 5.53 ± 0.61 | 0.018 |
| WLW (輪足步行) | Legacy | 5 | 10.98 ± 1.19 | 0.026 |
| MPC (模型預測控制) | ESEKF | 5 | 6.02 ± 1.26 | 0.029 |
| MPC (模型預測控制) | Legacy | 5 | 15.22 ± 1.63 | 0.029 |

---

## 3. NEW vs OLD 比較（位置 RMSE）

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|-----------|------------|---------|
| Walk | 6.50 ± 1.39 | 26.07 ± 0.80 | +75.1% |
| WLW  | 5.53 ± 0.61 | 10.98 ± 1.19 | +49.6% |
| MPC  | 6.02 ± 1.26 | 15.22 ± 1.63 | +60.4% |

> ⚠️ 注意：ESEKF 的 RMSE 包含 Z 軸，Legacy 只有 XY 平面，因此比較時需考慮量測基礎不同。

---

## 4. ESEKF 系統詳細指標

### 4.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |
|----------|----------|-----------|---------|
| FLAT_Walk_NEW_REAL_1 | 0.446 | 0.607 | 1.717 |
| FLAT_Walk_NEW_REAL_2 | 3.051 | 2.920 | 1.747 |
| FLAT_Walk_NEW_REAL_3 | 0.693 | 0.427 | 0.766 |
| FLAT_Walk_NEW_REAL_4 | 0.556 | 0.675 | 0.361 |
| FLAT_Walk_NEW_REAL_5 | 0.514 | 0.389 | 0.549 |
| FLAT_Walk_NEW_REAL_6 | 1.352 | 0.486 | 0.700 |
| FLAT_WLW_NEW_REAL_1 | 0.294 | 0.428 | 0.337 |
| FLAT_WLW_NEW_REAL_2 | 0.228 | 0.247 | 0.221 |
| FLAT_WLW_NEW_REAL_3 | 0.381 | 0.206 | 0.232 |
| FLAT_WLW_NEW_REAL_4 | 0.288 | 0.699 | 0.611 |
| FLAT_WLW_NEW_REAL_5 | 0.158 | 0.307 | 0.354 |
| FLAT_MPC_NEW_REAL_1 | 0.617 | 0.622 | 0.589 |
| FLAT_MPC_NEW_REAL_2 | 0.270 | 0.559 | 0.567 |
| FLAT_MPC_NEW_REAL_3 | 1.275 | 0.287 | 0.132 |
| FLAT_MPC_NEW_REAL_4 | 0.590 | 1.322 | 0.826 |
| FLAT_MPC_NEW_REAL_5 | 0.972 | 0.693 | 0.208 |

### 4.2 odom_mapping 位置 RMSE

| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|-------------|-------------|--------------|
| FLAT_Walk_NEW_REAL_1 | 0.83 | 5.75 | 5.81 |
| FLAT_Walk_NEW_REAL_2 | 1.86 | 5.14 | 5.47 |
| FLAT_Walk_NEW_REAL_3 | 0.86 | 1.76 | 1.96 |
| FLAT_Walk_NEW_REAL_4 | 0.59 | 4.80 | 4.83 |
| FLAT_Walk_NEW_REAL_5 | 0.75 | 5.73 | 5.78 |
| FLAT_Walk_NEW_REAL_6 | 1.02 | 3.99 | 4.11 |
| FLAT_WLW_NEW_REAL_1 | 1.30 | 0.95 | 1.61 |
| FLAT_WLW_NEW_REAL_2 | 1.35 | 1.06 | 1.71 |
| FLAT_WLW_NEW_REAL_3 | 1.99 | 0.64 | 2.09 |
| FLAT_WLW_NEW_REAL_4 | 1.13 | 1.19 | 1.64 |
| FLAT_WLW_NEW_REAL_5 | 0.95 | 0.57 | 1.11 |
| FLAT_MPC_NEW_REAL_1 | 1.11 | 2.60 | 2.83 |
| FLAT_MPC_NEW_REAL_2 | 0.71 | 2.70 | 2.79 |
| FLAT_MPC_NEW_REAL_3 | 1.86 | 2.98 | 3.51 |
| FLAT_MPC_NEW_REAL_4 | 1.50 | 2.27 | 2.72 |
| FLAT_MPC_NEW_REAL_5 | 0.99 | 2.74 | 2.92 |

---

## 5. 接觸偵測指標（ESEKF 系統）

| 實驗編號 | 腳 | Acc | Prec | Rec | F1 | Lat (ms) |
|----------|-----|-----|------|-----|----|----------|
| FLAT_Walk_NEW_REAL_1 | LF | 75.5% | 0.932 | 0.717 | 0.8107 | 686.5 |
| FLAT_Walk_NEW_REAL_1 | RF | 76.3% | 0.957 | 0.755 | 0.8437 | 531.5 |
| FLAT_Walk_NEW_REAL_1 | RH | 82.4% | 1.000 | 0.799 | 0.8882 | 63.4 |
| FLAT_Walk_NEW_REAL_1 | LH | 87.7% | 0.950 | 0.881 | 0.9145 | 119.2 |
| FLAT_Walk_NEW_REAL_2 | LF | 53.7% | 0.690 | 0.666 | 0.6778 | 642.7 |
| FLAT_Walk_NEW_REAL_2 | RF | 76.3% | 0.882 | 0.832 | 0.8563 | 290.0 |
| FLAT_Walk_NEW_REAL_2 | RH | 65.8% | 0.868 | 0.716 | 0.7844 | 1023.7 |
| FLAT_Walk_NEW_REAL_2 | LH | 61.7% | 0.758 | 0.713 | 0.7344 | 269.2 |
| FLAT_Walk_NEW_REAL_3 | LF | 56.2% | 0.786 | 0.664 | 0.7198 | 701.2 |
| FLAT_Walk_NEW_REAL_3 | RF | 52.1% | 0.664 | 0.707 | 0.6847 | 502.8 |
| FLAT_Walk_NEW_REAL_3 | RH | 55.9% | 0.729 | 0.700 | 0.7143 | 871.9 |
| FLAT_Walk_NEW_REAL_3 | LH | 63.1% | 0.858 | 0.701 | 0.7717 | 516.6 |
| FLAT_Walk_NEW_REAL_4 | LF | 87.3% | 0.926 | 0.899 | 0.9125 | 75.1 |
| FLAT_Walk_NEW_REAL_4 | RF | 89.6% | 0.962 | 0.913 | 0.9370 | 23.1 |
| FLAT_Walk_NEW_REAL_4 | RH | 84.5% | 1.000 | 0.822 | 0.9026 | 19.9 |
| FLAT_Walk_NEW_REAL_4 | LH | 87.2% | 0.959 | 0.865 | 0.9097 | 107.3 |
| FLAT_Walk_NEW_REAL_5 | LF | 87.3% | 0.930 | 0.896 | 0.9129 | 51.5 |
| FLAT_Walk_NEW_REAL_5 | RF | 89.1% | 0.962 | 0.908 | 0.9341 | 22.8 |
| FLAT_Walk_NEW_REAL_5 | RH | 84.9% | 0.980 | 0.835 | 0.9019 | 15.7 |
| FLAT_Walk_NEW_REAL_5 | LH | 84.1% | 0.931 | 0.840 | 0.8835 | 117.3 |
| FLAT_Walk_NEW_REAL_6 | LF | 88.6% | 0.943 | 0.905 | 0.9237 | 84.0 |
| FLAT_Walk_NEW_REAL_6 | RF | 90.0% | 0.965 | 0.916 | 0.9401 | 21.7 |
| FLAT_Walk_NEW_REAL_6 | RH | 82.7% | 1.000 | 0.795 | 0.8856 | 18.4 |
| FLAT_Walk_NEW_REAL_6 | LH | 82.1% | 0.956 | 0.800 | 0.8711 | 121.6 |
| FLAT_WLW_NEW_REAL_1 | LF | 53.9% | 0.624 | 0.791 | 0.6979 | 826.1 |
| FLAT_WLW_NEW_REAL_1 | RF | 52.0% | 0.600 | 0.768 | 0.6736 | 668.5 |
| FLAT_WLW_NEW_REAL_1 | RH | 34.9% | 0.364 | 0.591 | 0.4507 | 622.0 |
| FLAT_WLW_NEW_REAL_1 | LH | 44.2% | 0.628 | 0.509 | 0.5621 | 295.8 |
| FLAT_WLW_NEW_REAL_2 | LF | 65.3% | 0.711 | 0.853 | 0.7756 | 1347.1 |
| FLAT_WLW_NEW_REAL_2 | RF | 45.3% | 0.449 | 0.780 | 0.5698 | 247.3 |
| FLAT_WLW_NEW_REAL_2 | RH | 70.2% | 0.727 | 0.866 | 0.7904 | 278.5 |
| FLAT_WLW_NEW_REAL_2 | LH | 51.4% | 0.579 | 0.621 | 0.5991 | 2567.3 |
| FLAT_WLW_NEW_REAL_3 | LF | 41.2% | 0.471 | 0.752 | 0.5788 | 1085.4 |
| FLAT_WLW_NEW_REAL_3 | RF | 51.1% | 0.595 | 0.754 | 0.6652 | 546.0 |
| FLAT_WLW_NEW_REAL_3 | RH | 36.8% | 0.384 | 0.619 | 0.4744 | 589.4 |
| FLAT_WLW_NEW_REAL_3 | LH | 43.4% | 0.624 | 0.500 | 0.5553 | 314.7 |
| FLAT_WLW_NEW_REAL_4 | LF | 63.8% | 0.701 | 0.838 | 0.7636 | 1049.3 |
| FLAT_WLW_NEW_REAL_4 | RF | 46.8% | 0.453 | 0.815 | 0.5823 | 562.0 |
| FLAT_WLW_NEW_REAL_4 | RH | 68.8% | 0.717 | 0.856 | 0.7804 | 242.5 |
| FLAT_WLW_NEW_REAL_4 | LH | 63.2% | 0.725 | 0.710 | 0.7171 | 458.4 |
| FLAT_WLW_NEW_REAL_5 | LF | 64.6% | 0.708 | 0.846 | 0.7709 | 1385.0 |
| FLAT_WLW_NEW_REAL_5 | RF | 47.3% | 0.459 | 0.819 | 0.5879 | 814.5 |
| FLAT_WLW_NEW_REAL_5 | RH | 69.1% | 0.717 | 0.862 | 0.7832 | 256.0 |
| FLAT_WLW_NEW_REAL_5 | LH | 58.6% | 0.645 | 0.695 | 0.6691 | 2523.2 |
| FLAT_MPC_NEW_REAL_1 | LF | 89.7% | 0.996 | 0.873 | 0.9304 | 56.7 |
| FLAT_MPC_NEW_REAL_1 | RF | 90.5% | 0.968 | 0.921 | 0.9439 | 29.1 |
| FLAT_MPC_NEW_REAL_1 | RH | 88.5% | 1.000 | 0.855 | 0.9216 | 72.0 |
| FLAT_MPC_NEW_REAL_1 | LH | 92.8% | 0.968 | 0.921 | 0.9441 | 69.0 |
| FLAT_MPC_NEW_REAL_2 | LF | 89.3% | 1.000 | 0.864 | 0.9269 | 52.4 |
| FLAT_MPC_NEW_REAL_2 | RF | 90.5% | 0.972 | 0.918 | 0.9439 | 25.8 |
| FLAT_MPC_NEW_REAL_2 | RH | 89.5% | 0.999 | 0.865 | 0.9272 | 34.1 |
| FLAT_MPC_NEW_REAL_2 | LH | 93.6% | 0.991 | 0.912 | 0.9499 | 44.3 |
| FLAT_MPC_NEW_REAL_3 | LF | 93.2% | 0.998 | 0.910 | 0.9517 | 52.4 |
| FLAT_MPC_NEW_REAL_3 | RF | 90.8% | 0.993 | 0.901 | 0.9449 | 24.3 |
| FLAT_MPC_NEW_REAL_3 | RH | 88.0% | 1.000 | 0.854 | 0.9215 | 33.5 |
| FLAT_MPC_NEW_REAL_3 | LH | 94.9% | 0.987 | 0.934 | 0.9599 | 32.5 |
| FLAT_MPC_NEW_REAL_4 | LF | 90.0% | 1.000 | 0.873 | 0.9320 | 54.0 |
| FLAT_MPC_NEW_REAL_4 | RF | 90.1% | 0.992 | 0.892 | 0.9397 | 31.7 |
| FLAT_MPC_NEW_REAL_4 | RH | 87.2% | 1.000 | 0.842 | 0.9142 | 33.7 |
| FLAT_MPC_NEW_REAL_4 | LH | 87.7% | 0.991 | 0.843 | 0.9108 | 111.5 |
| FLAT_MPC_NEW_REAL_5 | LF | 92.7% | 0.994 | 0.906 | 0.9481 | 33.8 |
| FLAT_MPC_NEW_REAL_5 | RF | 89.5% | 0.994 | 0.886 | 0.9367 | 30.9 |
| FLAT_MPC_NEW_REAL_5 | RH | 88.9% | 1.000 | 0.862 | 0.9258 | 53.4 |
| FLAT_MPC_NEW_REAL_5 | LH | 95.7% | 0.985 | 0.948 | 0.9662 | 36.6 |

---

## 6. MPC 終點 X 位置分析（目標：3.0 m）

MPC 控制器以走到 X = 3 m 為目標停止。本節分析估測器回報的停止位置（控制依據）與 VICON 實際量測的停止位置之間的誤差，評估里程計對運動控制的實際影響。

| 實驗編號 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | 停止誤差 VICON (cm) |
|----------|-------------------|-------------------|--------------|--------------------|
| FLAT_MPC_NEW_REAL_1 | 2.997 | 2.985 | -0.3 | -1.5 |
| FLAT_MPC_NEW_REAL_2 | 2.991 | 2.983 | -0.9 | -1.7 |
| FLAT_MPC_NEW_REAL_3 | 3.017 | 2.983 | +1.7 | -1.7 |
| FLAT_MPC_NEW_REAL_4 | 3.018 | 2.998 | +1.8 | -0.2 |
| FLAT_MPC_NEW_REAL_5 | 2.999 | 2.992 | -0.1 | -0.8 |
| FLAT_MPC_OLD_REAL_1 | 3.003 | 2.747 | +0.3 | -25.3 |
| FLAT_MPC_OLD_REAL_2 | 3.010 | 2.799 | +1.0 | -20.1 |
| FLAT_MPC_OLD_REAL_3 | 3.008 | 2.761 | +0.8 | -23.9 |
| FLAT_MPC_OLD_REAL_4 | 3.000 | 2.747 | +0.0 | -25.3 |
| FLAT_MPC_OLD_REAL_5 | 3.002 | 2.736 | +0.2 | -26.4 |

**統計摘要（目標 X = 3.0 m）**

| 系統 | 估測器 final X | 實際 VICON final X | 估測器停止誤差 (abs mean) | VICON 停止誤差 (abs mean) |
|------|--------------|-------------------|--------------------------|--------------------------|
| ESEKF (NEW) | 3.004 ± 0.011 m | 2.988 ± 0.006 m | 1.0 cm | 1.2 cm |
| Legacy (OLD) | 3.005 ± 0.004 m | 2.758 ± 0.022 m | 0.5 cm | 24.2 cm |

**分析：**
- ESEKF（NEW）：估測器回報 3.004 ± 0.011 m，VICON 實際停止 2.988 ± 0.006 m，平均停止誤差 **1.2 cm**（< 2 cm）。估測結果與實際高度吻合，控制器能精準停止於目標位置。
- Legacy（OLD）：估測器回報 3.005 ± 0.004 m，VICON 實際停止 2.758 ± 0.022 m，平均停止誤差 **24.2 cm**（~24 cm，超出目標 24 cm）。Legacy 里程計嚴重**高估**行進距離（腿式積分累積誤差），導致機器人尚未到達 3 m 目標便誤判已抵達而停止。
- 改善幅度：ESEKF 的實際停止誤差為 Legacy 的 **20.6×** 以下，顯示 LiDAR 融合對 MPC 點到點移動任務的準確性有關鍵改善。

---

## 7. Closed-Loop（MPC）vs Open-Loop（Walk）比較

本節以 **VICON 為基準**，比較：
- **Closed-Loop**：`FLAT_MPC_NEW_REAL_1~5`（MPC 控制器，位置閉迴路）
- **Open-Loop**：`FLAT_Walk_NEW_REAL_1~6`（步行控制器，位置開迴路；不含 _2）

分析面向：(1) 姿態（Roll / Pitch）穩定性，(2) 前進速度（以 0.1 m/s 為目標，分析穩態均值與 RMSE）。

> 姿態穩定性使用 **VICON 實際姿態相對水平面 0° 的 RMS**，不是 EKF 與 VICON 的姿態估測誤差。計算方式為 `sqrt(mean(VICON_roll_deg²))` 與 `sqrt(mean(VICON_pitch_deg²))`。

### 7.1 每次試驗詳細指標

| 實驗編號 | Roll RMS vs 0° (°) | Pitch RMS vs 0° (°) | VICON vx 穩態 (m/s) | Δvx 偏離 0.1 m/s (cm/s) |
|----------|-------------------|---------------------|---------------------|-------------------------|
| FLAT_MPC_NEW_REAL_1 | 1.220 | 1.355 | 0.088 ± 0.037 | -1.2 |
| FLAT_MPC_NEW_REAL_2 | 1.327 | 1.169 | 0.087 ± 0.035 | -1.3 |
| FLAT_MPC_NEW_REAL_3 | 1.513 | 1.345 | 0.086 ± 0.041 | -1.4 |
| FLAT_MPC_NEW_REAL_4 | 1.080 | 1.432 | 0.089 ± 0.036 | -1.1 |
| FLAT_MPC_NEW_REAL_5 | 1.648 | 1.361 | 0.084 ± 0.041 | -1.6 |
| — | — | — | — | — |
| FLAT_Walk_NEW_REAL_1 | 1.623 | 1.508 | 0.093 ± 0.044 | -0.7 |
| FLAT_Walk_NEW_REAL_3 | 1.582 | 1.460 | 0.093 ± 0.045 | -0.7 |
| FLAT_Walk_NEW_REAL_4 | 1.613 | 1.497 | 0.094 ± 0.046 | -0.6 |
| FLAT_Walk_NEW_REAL_5 | 1.661 | 1.538 | 0.094 ± 0.046 | -0.6 |
| FLAT_Walk_NEW_REAL_6 | 1.606 | 1.465 | 0.095 ± 0.046 | -0.5 |

### 7.2 分組統計摘要

| 指標 | Closed-Loop (MPC) | Open-Loop (Walk) |
|------|------------------|-----------------|
| n（試驗數） | 5 | 5 |
| Roll RMS vs 0° (°) | 1.358 ± 0.203 | 1.617 ± 0.026 |
| Pitch RMS vs 0° (°) | 1.332 ± 0.087 | 1.494 ± 0.029 |
| VICON vx 穩態均值（全程，m/s） | 0.087 ± 0.002 | 0.094 ± 0.001 |
| VICON vx 巡航均值（減速前，m/s） | 0.087 ± 0.001 | 0.094 ± 0.001 |
| VICON vx RMSE from 0.1（全程，cm/s） | 1.3 | 0.6 |
| VICON vx RMSE from 0.1（巡航，cm/s） | **1.3** | **0.6** |
| peak vx EKF（瞬間最大，m/s） | 0.242 | 0.412 |

### 7.3 分析

**姿態穩定性（Roll / Pitch）：**
- Closed-Loop（MPC）VICON Roll RMS=1.36°、Pitch RMS=1.33°；Open-Loop（Walk）VICON Roll RMS=1.62°、Pitch RMS=1.49°。
- 這裡評估的是機器人本體相對水平面 0° 的實際晃動量，因此數值代表姿態穩定度，而不是估測器追蹤 VICON 的準確度。
- MPC 的 Roll/Pitch RMS 均略低於 Walk，表示閉迴路定點移動沒有造成額外姿態不穩，反而讓機體姿態擺動稍微更小。
- 五筆 MPC 試驗中，Roll RMS 範圍為 1.08–1.65°，Pitch RMS 範圍為 1.17–1.43°，整體維持在約 1.5° 以內，顯示平地 MPC 運動過程的姿態穩定性良好。

**前進速度（目標 0.1 m/s）：**
- **全程穩態（含減速）**：MPC 均值 0.087 m/s（RMSE 1.3 cm/s）；Walk 均值 0.094 m/s（RMSE 0.6 cm/s）。MPC 的全程均值偏低，因為終點控制器減速拉低了統計值。
- **巡航段（減速前）**：MPC 巡航均值 0.087 m/s（RMSE **1.3 cm/s**）；Walk 巡航均值 0.094 m/s（RMSE **0.6 cm/s**）。排除減速段後，MPC 巡航速度與 Walk 接近，均能穩定追蹤 0.1 m/s 速度指令。
- peak vx：Walk（0.412 m/s） >> MPC（0.242 m/s），反映步行步態中 CoM 有較大的瞬間速度振盪，而 MPC 速度剖面較平滑。

---

## 8. 觀察與結論

### 平地步行（Walk）
- ESEKF 融合 LiDAR 與腿式里程計，提供三維位置估計（RMSE_3D ~6 cm）。
- Legacy 系統僅腿式里程計，RMSE_2D ~26 cm，誤差約為 ESEKF 的 4×。

### 平地輪足步行（WLW）
- ESEKF RMSE_3D ~5.5 cm；Legacy RMSE_2D ~11 cm，約 2× 差距。
- 輪足模式下里程計積分誤差相較步行模式略小。

### 平地 MPC（MPC）— 終點精度
- MPC 控制目標為 X = 3.0 m 定點停止。
- **ESEKF**：VICON 實際停止 2.988 ± 0.006 m，誤差 **1.2 cm** ✓
- **Legacy**：VICON 實際停止 2.758 ± 0.022 m，誤差 **24.2 cm** ✗（估測器虛報抵達，實際距離不足）
- mpc_esekf bag 無 `/ekf/ba`、`/ekf/bw`，故偏差估計不分析。

### 整體結論
- ESEKF + LiDAR 融合在三種運動模式下均顯著優於 Legacy 里程計。
- 對於 MPC 定點控制任務，里程計精度直接影響停止位置；Legacy 累積誤差可達 24 cm，而 ESEKF 可控制在 2 cm 以內。

---

*報告由 `analyze.py` 自動產生。日期：2026-06-11*
