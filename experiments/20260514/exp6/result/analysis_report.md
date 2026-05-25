# CORGI 實驗分析報告（Information Filter）

**日期：** 2026-05-14
**實驗編號：** `exp6`
**實驗名稱：** `walk_2m_01_obs_odometry_legacy` （啟用觀測速度補正 obs_odometry）
**Bag 檔案：** `legacy_odom20260514_232823_0.db3`
**VICON CSV：** `EXP_06_z_corrected.csv`
**步行分析區間：** t = 0 – 24.51 s
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
| RMSE X（vs VICON） | 11.94 cm |
| RMSE Y（vs VICON） | 7.74 cm |
| RMSE Z（vs VICON） | 8.54 cm |
| RMSE 3D（vs VICON） | **16.60 cm** |
| 最大 3D 誤差 | 25.53 cm |
| 最終位置（Legacy） | (1.874, 0.554, -0.029) m |
| 最終位置（VICON） | (1.705, 0.663, 0.113) m |

**觀察：**
- Y 方向誤差（7.74 cm）為主要誤差來源，顯示側向漂移嚴重。
- X 方向 RMSE 11.94 cm，前進方向估計尚可。
- Legacy（Information Filter）無 LiDAR 修正，位置誤差遠大於 ESEKF。

---

## 2. 速度分析（vs VICON）

![速度時序](fig_vel_time.png)

| 指標 | 數值 |
|------|------|
| RMSE vx（vs VICON） | 0.039 m/s |
| RMSE vy（vs VICON） | 0.042 m/s |
| RMSE vz（vs VICON） | 0.060 m/s |
| 最大前進速度 | 0.325 m/s |

**觀察：**
- vx RMSE 0.037 m/s，速度估計品質良好，與 ESEKF 相當。
- 速度估計不依賴位置積分，因此與位置誤差相互獨立，品質相對穩定。

---

## 摘要

| 指標 | 數值 |
|------|------|
| 步行時間 | 24.51 s |
| 位置 RMSE 3D | 16.60 cm |
| 最大位置誤差 3D | 25.53 cm |
| 速度 vx RMSE | 0.037 m/s |
| 速度 vy RMSE | 0.044 m/s |

> **結論：** Information Filter（Legacy）在速度估計上表現與 ESEKF 相當，但位置估計因缺乏 LiDAR 修正而累積漂移顯著（16.60 cm vs ESEKF ~4.8 cm）。
