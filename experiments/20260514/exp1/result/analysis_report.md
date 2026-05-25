# CORGI 實驗分析報告

**日期：** 2026-05-14
**實驗編號：** `exp1`
**實驗名稱：** `walk_2m_01_plain_odometry` （標準 plain_odometry）
**Bag 檔案：** `odom_fusion20260514_215405_0.db3`
**VICON CSV：** `EXP_01.csv`
**步行分析區間：** t = 0 – 23.50 s
**分析腳本：** `analyze.py`

---

## 系統架構

```
/imu_raw ─┐
/motor/state ──┤──► corgi_leg_odom ──► /ekf ────────────────────► (odom→base_link)
/trigger ──────┘                              │
/gmo/contact_state                            ▼
                          /lidar_odom ──► corgi_fusion_node ──► /odom_mapping
                         (FAST-LIO2,                           /fusion/bv
                          camera_init frame)
```

**T_{odom ← camera\_init}：**
translation = [0.101, -0.034, 0.166] m
RPY = [20.3°, 0.0°, 90.1°]
配準殘差：平均 0.6 cm，最大 1.7 cm

---

## 1. 接觸偵測（Contact Detection）

### 1.1 接觸偵測指標

接觸閾值：**15 mm**（相對於地面平面）
地面平面：ground1–ground4 單一凸包

![接觸時序](fig_contact.png)

| 腳 | N | TP | TN | FP | FN | 準確率 | 精確率 | 召回率 | F1 | 延遲 (ms) |
|----|---|----|----|----|----|--------|--------|--------|-----|-----------|
| LF | 11752 | 8384 | 2098 | 455 | 815 | 89.2% | 0.949 | 0.911 | 0.9296 | 26.8 |
| RF | 10567 | 8073 | 1269 | 68 | 1157 | 88.4% | 0.992 | 0.875 | 0.9295 | 23.4 |
| RH | 11752 | 8546 | 1358 | 0 | 1848 | 84.3% | 1.000 | 0.822 | 0.9024 | 16.9 |
| LH | 11752 | 7892 | 2060 | 160 | 1640 | 84.7% | 0.980 | 0.828 | 0.8976 | 92.8 |

**觀察：**
- RF（右前腳）精確率 0.992、召回率 0.875。
- RH（右後腳）精確率 1.000、召回率 0.822。
- 誤報率（FP / (TP+FP)）低表示 GMO 觸發保守；FN 偏高表示輕觸或抬腳初期有漏偵測。

---

## 2. 內部 EKF 分析（Inner EKF）

### 2.1 位置

![EKF XY 軌跡](fig_ekf_xy.png)
![EKF 位置時序](fig_ekf_pos.png)

| 指標 | 數值 |
|------|------|
| RMSE X（vs VICON） | 1.04 cm |
| RMSE Y（vs VICON） | 4.72 cm |
| RMSE Z（vs VICON） | 1.23 cm |
| RMSE 3D（vs VICON） | **4.99 cm** |
| 最大 3D 誤差 | 8.72 cm |
| 最終位置（EKF） | (1.867, -0.199, -0.004) m |
| 最終位置（VICON） | (1.861, -0.275, 0.002) m |

**觀察：**
- Y 方向誤差（4.72 cm）為主要誤差來源，X 方向追蹤良好（1.04 cm）。
- Z RMSE = 1.23 cm，Z 高度估計穩定。

### 2.2 速度

![EKF 速度](fig_ekf_vel.png)

| 指標 | 數值 |
|------|------|
| RMSE vx（vs VICON） | 0.042 m/s |
| RMSE vy（vs VICON） | 0.050 m/s |
| RMSE vz（vs VICON） | 0.021 m/s |
| 最大前進速度 | 0.358 m/s |

**觀察：**
- vx RMSE 0.042 m/s，前進速度追蹤品質良好。

### 2.3 姿態（RPY）

![EKF 姿態](fig_ekf_rpy.png)

| 指標 | 數值 |
|------|------|
| RMSE roll（vs VICON） | 0.26° |
| RMSE pitch（vs VICON） | 0.47° |
| RMSE yaw（vs VICON） | 0.30° |
| 最終 yaw（EKF） | -9.13° |
| 最終 yaw（VICON） | -9.61° |

**觀察：**
- Roll/Pitch 估計穩定（0.26°/0.47°）。
- Yaw 最終偏差 0.48°，偏航估計精確。

### 2.4 加速度計偏差（ba）

![偏差](fig_ekf_bias.png)

| 軸 | 初始值 [m/s²] | 穩態值 [m/s²] | 穩態標準差 [m/s²] |
|----|--------------|--------------|------------------|
| x  | 0.00011 | 0.00066 | 0.00011 |
| y  | -0.00009 | -0.00378 | 0.00084 |
| z  | -0.01703 | -0.01032 | 0.00092 |

### 2.5 陀螺儀偏差（bw）

| 軸 | 初始值 [rad/s] | 穩態值 [rad/s] | 穩態標準差 [rad/s] |
|----|---------------|---------------|------------------|
| x  | 0.000063 | 0.000063 | 0.0000001 |
| y  | 0.000640 | 0.000640 | 0.0000000 |
| z  | 0.000286 | 0.000287 | 0.0000002 |

**觀察：**
- ba_y 從 -0.00009 漂移至 -0.00378 m/s²，收斂不完全，為 Y 方向位置漂移的主因。
- bw 三軸穩態標準差極小，陀螺儀偏差收斂良好。

---

## 3. 外部融合節點（Outer Fusion Node）

### 3.1 odom_mapping 位置

![融合 XY 軌跡](fig_fusion_xy.png)
![融合位置對比](fig_fusion_pos.png)

| 指標 | 數值 |
|------|------|
| RMSE 2D vs VICON | **3.11 cm** |
| 最大 2D 誤差 vs VICON | 5.24 cm |
| RMSE 2D vs EKF | 1.93 cm |
| 最終位置（odom_mapping） | (1.867, -0.230) m |

**觀察：**
- odom_mapping RMSE 2D（3.11 cm）優於 inner EKF 3D RMSE（4.99 cm），LiDAR 融合提升橫向定位精度。

### 3.2 odom_mapping 偏航角

| 指標 | 數值 |
|------|------|
| RMSE yaw vs VICON | **0.23°** |
| RMSE yaw vs EKF | 0.50° |
| 最終 yaw（odom_mapping） | -10.05° |
| 最終 yaw（VICON） | -9.62° |

**觀察：**
- 融合節點 yaw RMSE 0.23° 接近 inner EKF（0.30°），LiDAR 修正偏航效果有限。

### 3.3 fusion/bv 速度偏差修正量

![fusion/bv](fig_fusion_bv.png)

> **說明：** `/fusion/bv` 為外部融合節點估計的腿部里程計速度偏差修正量，用以持續補正 leg odometry 速度估測誤差，修正過程不造成位置跳變。

| 指標 | 數值 |
|------|------|
| 平均修正量 vx | -0.0020 m/s |
| 平均修正量 vy | 0.0018 m/s |
| 平均修正幅度 | **0.0033 m/s** |
| 最大修正幅度 | 0.0058 m/s |

#### odom_mapping 速度 vs VICON

| 指標 | 數值 |
|------|------|
| RMSE vx（odom_mapping vs VICON） | 0.097 m/s |
| RMSE vy（odom_mapping vs VICON） | 0.060 m/s |

---

## 4. LiDAR 輸入品質

![LiDAR XY](fig_lidar_xy.png)

| 指標 | 數值 |
|------|------|
| 訊息總數 | 235 |
| 更新頻率 | 10.0 Hz |
| 跳變數（>10cm） | 0 |

**T_{odom←camera\_init} 估計（Procrustes 配準）：**
- translation = ['0.101', '-0.034', '0.166'] m
- RPY = ['20.3', '0.0', '90.1']°
- 配準殘差 mean=0.56 cm，max=1.74 cm

**觀察：**
- LiDAR 更新率 10.0 Hz，符合 FAST-LIO2 標準頻率（10 Hz）。
- 無明顯跳變事件，LiDAR 輸入穩定。

---

## 摘要

| 指標 | 數值 |
|------|------|
| 步行時間 | 23.50 s |
| Inner EKF 位置 RMSE 3D | 4.99 cm |
| odom_mapping RMSE 2D vs VICON | 3.11 cm |
| EKF Yaw RMSE | 0.30° |
| odom_mapping Yaw RMSE | 0.23° |
| EKF 速度 vx RMSE | 0.042 m/s |
