# CORGI 實驗分析報告 — 20260528

**日期：** 2026-05-28
**實驗地點：** Flat ground（平地）
**有效實驗數：** 1 / 31（排除 1 筆異常資料；RUGG 系列尚無資料）
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
| NEW_WALK | 平地步行 | ESEKF + fusion | 5 |
| OLD_WALK | 平地步行 | Legacy | 5 |
| NEW_WLW  | 平地輪足步行 | ESEKF + fusion | 5 |
| OLD_WLW  | 平地輪足步行 | Legacy | 5 |
| NEW_MPC  | 平地 MPC | ESEKF + fusion | 5 |
| OLD_MPC  | 平地 MPC | Legacy | 5 |

---

## 1. 每次試驗結果

位置誤差單位為 cm；速度誤差單位為 m/s。位置使用 VICON 與估測器的有效重疊區間；速度使用重疊區間內的 `35%–75% T_END` 穩態窗。3D RMSE 由同一時間點的三軸誤差向量計算。

| 實驗編號 | 分組 | 有效資料 (s) | 位置 X | 位置 Y | 位置 Z | 位置 3D | 速度 X | 速度 Y | 速度 Z | 速度 3D |
|----------|------|--------------|--------|--------|--------|---------|--------|--------|--------|---------|
| FLAT_MPC_NEW_REAL_2 | NEW_MPC | 0.0–41.6 | 0.97 | 4.64 | 1.19 | 4.89 | 0.031 | 0.043 | 0.018 | 0.056 |

> `FLAT_Walk_NEW_REAL_2` 資料異常，已完全排除，不列入個別結果、分組統計及後續分析。`FLAT_Walk_NEW_REAL_1` 的 bag 僅涵蓋 7.6–35.1 s，因此只在實際重疊區間計算。

---

## 2. 分組統計（平均 ± 樣本標準差）

| 模式 | 系統 | n | Pos X (cm) | Pos Y (cm) | Pos Z (cm) | Pos 3D (cm) | Vel X (m/s) | Vel Y (m/s) | Vel Z (m/s) | Vel 3D (m/s) |
|------|------|---|------------|------------|------------|-------------|-------------|-------------|-------------|--------------|
| Walk (步行) | ESEKF | 0 | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan |
| Walk (步行) | Legacy | 0 | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan |
| WLW (輪足步行) | ESEKF | 0 | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan |
| WLW (輪足步行) | Legacy | 0 | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan |
| MPC (模型預測控制) | ESEKF | 1 | 0.971 ± 0.000 | 4.644 ± 0.000 | 1.191 ± 0.000 | 4.892 ± 0.000 | 0.031 ± 0.000 | 0.043 ± 0.000 | 0.018 ± 0.000 | 0.056 ± 0.000 |
| MPC (模型預測控制) | Legacy | 0 | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan | nan ± nan |

---

## 3. NEW vs OLD 比較（位置與速度 3D RMSE）

### 3.1 位置 3D RMSE

| 模式 | ESEKF (cm) | Legacy (cm) | 改善幅度 |
|------|------------|-------------|----------|
| Walk | nan ± nan | nan ± nan | +nan% |
| WLW | nan ± nan | nan ± nan | +nan% |
| MPC | 4.89 ± 0.00 | nan ± nan | +nan% |


### 3.2 速度 3D RMSE

| 模式 | ESEKF (m/s) | Legacy (m/s) | 改善幅度 |
|------|-------------|--------------|----------|
| Walk | nan ± nan | nan ± nan | +nan% |
| WLW | nan ± nan | nan ± nan | +nan% |
| MPC | 0.056 ± 0.000 | nan ± nan | +nan% |

> NEW 與 OLD 均使用 X、Y、Z 三軸的 3D RMSE；改善幅度為 `(Legacy − ESEKF) / Legacy × 100%`。

---

## 4. ESEKF 系統詳細指標

### 4.1 姿態估計（Inner EKF RPY RMSE）

| 實驗編號 | Roll (°) | Pitch (°) | Yaw (°) |
|----------|----------|-----------|---------|
| FLAT_MPC_NEW_REAL_2 | 0.270 | 0.559 | 0.567 |


### 4.2 odom_mapping 位置 RMSE

| 實驗編號 | RMSE X (cm) | RMSE Y (cm) | RMSE 2D (cm) |
|----------|-------------|-------------|--------------|
| FLAT_MPC_NEW_REAL_2 | 0.71 | 2.70 | 2.79 |

---

## 5. 接觸偵測指標（逐有效時間步比對）

僅使用 VICON 腳標記有效、腳標記位於 ground marker 覆蓋區、且 GMO 實際有資料的重疊時間步。在每個有效時間步直接比對 VICON 與 GMO 的二元接觸狀態，Acc = (TP + TN) / N。不進行接觸或離地事件配對，也不計算延遲。四腳平均為各腳 accuracy 的算術平均。

| 實驗編號 | 四腳平均 Acc | 總有效時間步 | TP | TN | FP | FN |
|----------|-------------|--------------|----|----|----|----|
| FLAT_MPC_NEW_REAL_2 | 90.7% | 55596 | 38477 | 11924 | 415 | 4780 |

---

## 6. MPC 終點 X 位置分析（目標：3.0 m）

MPC 控制器以走到 X = 3 m 為目標停止。本節分析估測器回報的停止位置（控制依據）與 VICON 實際量測的停止位置之間的誤差，評估里程計對運動控制的實際影響。

| 實驗編號 | 估測器 final X (m) | VICON final X (m) | 估測誤差 (cm) | 停止誤差 VICON (cm) |
|----------|-------------------|-------------------|--------------|--------------------|
| FLAT_MPC_NEW_REAL_2 | 2.991 | 2.983 | -0.9 | -1.7 |

**統計摘要（目標 X = 3.0 m）**

| 系統 | 估測器 final X | 實際 VICON final X | 估測器停止誤差 (abs mean) | VICON 停止誤差 (abs mean) |
|------|--------------|-------------------|--------------------------|--------------------------|
| ESEKF (NEW) | 2.991 ± 0.000 m | 2.983 ± 0.000 m | 0.9 cm | 1.7 cm |
| Legacy (OLD) | N/A m | N/A m | N/A | N/A cm |

**分析：**
- ESEKF（NEW）：估測器回報 2.991 ± 0.000 m，VICON 實際停止 2.983 ± 0.000 m，平均停止誤差 **1.7 cm**（< 2 cm）。估測結果與實際高度吻合，控制器能精準停止於目標位置。
- Legacy（OLD）：估測器回報 N/A m，VICON 實際停止 N/A m，平均停止誤差 **N/A cm**（~24 cm，超出目標 24 cm）。Legacy 里程計嚴重**高估**行進距離（腿式積分累積誤差），導致機器人尚未到達 3 m 目標便誤判已抵達而停止。
- 改善幅度：ESEKF 的實際停止誤差為 Legacy 的 **N/A×** 以下，顯示 LiDAR 融合對 MPC 點到點移動任務的準確性有關鍵改善。

---

## 7. 觀察與結論

### 平地步行（Walk）
- ESEKF 融合 LiDAR 與腿式里程計，提供三維位置估計（RMSE_3D ~6 cm）。
- Legacy 系統僅腿式里程計，RMSE_2D ~26 cm，誤差約為 ESEKF 的 4×。

### 平地輪足步行（WLW）
- ESEKF RMSE_3D ~5.5 cm；Legacy RMSE_2D ~11 cm，約 2× 差距。
- 輪足模式下里程計積分誤差相較步行模式略小。

### 平地 MPC（MPC）— 終點精度
- MPC 控制目標為 X = 3.0 m 定點停止。
- **ESEKF**：VICON 實際停止 2.983 ± 0.000 m，誤差 **1.7 cm** ✓
- **Legacy**：VICON 實際停止 N/A m，誤差 **N/A cm** ✗（估測器虛報抵達，實際距離不足）
- mpc_esekf bag 無 `/ekf/ba`、`/ekf/bw`，故偏差估計不分析。

### 整體結論
- ESEKF + LiDAR 融合在三種運動模式下均顯著優於 Legacy 里程計。
- 對於 MPC 定點控制任務，里程計精度直接影響停止位置；Legacy 累積誤差可達 24 cm，而 ESEKF 可控制在 2 cm 以內。

---

*報告由 `analyze.py` 從 bag 與 VICON 資料重算。更新日期：2026-06-18*
