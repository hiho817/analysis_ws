# CORGI 實驗分析報告（Information Filter）

**日期：** 2026-05-14
**實驗編號：** `exp3`
**實驗名稱：** `walk_2m_01_plain_odometry_legacy` （標準 plain_odometry）
**Bag 檔案：** `legacy_odom20260514_222433_0.db3`
**VICON CSV：** `EXP_03.csv`
**步行分析區間：** t = 0 – 23.53 s
**分析腳本：** `analyze.py`

---

## 系統架構

```
/imu ──────────────────────────────────────────────► /odometry/legacy/position
/motor/state ──► corgi_odometry_legacy ───────────► /odometry/legacy/velocity
/trigger                                           /odometry/legacy/contact
```

> Information Filter（Legacy）不使用 LiDAR 融合，僅依賴 IMU 與腿部運動學估計位置與速度。

---

## 1. 位置分析（vs VICON）

![位置 XY 軌跡](fig_traj_xy.png)
![位置時序](fig_pos_time.png)

| 指標 | 數值 |
|------|------|
| RMSE X（vs VICON） | 6.03 cm |
| RMSE Y（vs VICON） | 7.26 cm |
| RMSE Z（vs VICON） | 1.78 cm |
| RMSE 3D（vs VICON） | **9.61 cm** |
| 最大 3D 誤差 | 17.12 cm |
| 最終位置（Legacy） | (1.955, 0.233) m |
| 最終位置（VICON） | (1.852, 0.341) m |

**觀察：**
- X 與 Y 方向誤差相近（6.03 / 7.26 cm），無明顯主導誤差方向。
- Z 方向誤差 1.78 cm，高度估計良好。
- Legacy（Information Filter）無 LiDAR 修正，位置誤差約為 ESEKF 的 2 倍。

---

## 2. 速度分析（vs VICON）

![速度時序](fig_vel_time.png)

> 速度 RMSE 計算窗口：t = 9.4 – 16.5 s（穩態步行段）

| 指標 | 數值 |
|------|------|
| RMSE vx（vs VICON） | 0.037 m/s |
| RMSE vy（vs VICON） | 0.043 m/s |
| RMSE vz（vs VICON） | 0.058 m/s |
| 最大前進速度 | 0.309 m/s |

**觀察：**
- vx RMSE 0.037 m/s，速度估計品質良好，與 ESEKF 相當。
- 速度估計不依賴位置積分，因此與位置誤差相互獨立，品質相對穩定。

---

## 摘要

| 指標 | 數值 |
|------|------|
| 步行時間 | 23.53 s |
| 位置 RMSE 3D | 9.61 cm |
| 最大位置誤差 3D | 17.12 cm |
| 速度 vx RMSE | 0.037 m/s |
| 速度 vy RMSE | 0.043 m/s |

> **結論：** Information Filter（Legacy）在前進速度估計上表現與 ESEKF 相當（vx RMSE 0.037 m/s），但側向速度誤差（vy 0.113 m/s）較大。位置 RMSE 9.61 cm，約為 ESEKF（~4.8 cm）的 2 倍，反映無 LiDAR 修正下的積分漂移。
