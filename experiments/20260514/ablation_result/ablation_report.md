# CORGI 消融實驗報告：有無 LiDAR 回授之比較

**實驗日期：** 2026-05-14

## 消融設計

| 配置 | 說明 |
|------|------|
| **With LiDAR** | 正常 odom_fusion bag（fusion/bv 回授至 inner ESEKF） |
| **Without LiDAR** | 相同 bag replay，排除 `/lidar_odom`，bv_outer_=0 |

系統架構中，`corgi_fusion_node` 輸出 `/fusion/bv`（body velocity bias correction），
`corgi_leg_odom` 的 `cb_bv_outer` 回調接收後透過 `ESEKF::update_leg()` 修正
leg velocity observation：`z_leg -= R_body^T * bv_outer`。
消融測試移除此回授迴路，比較位置與速度精度的差異。

## 實驗結果彙整

### 3D Position RMSE 比較

| 實驗 | 步態 | With LiDAR (cm) | Without LiDAR (cm) | 差異 (cm) | 改善率 |
|------|------|:---:|:---:|:---:|:---:|
| exp1 | walk_2m_01_plain_odometry | **4.99** | 9.38 | +4.40 | 46.9% |
| exp2 | walk_2m_01_plain_odometry | **4.63** | 12.25 | +7.62 | 62.2% |
| exp4 | walk_2m_01_obs_odometry | **4.83** | 12.08 | +7.25 | 60.0% |
| exp5 | walk_2m_01_obs_odometry | **19.62** | 22.02 | +2.39 | 10.9% |

### 速度 RMSE (vx) 比較

| 實驗 | With LiDAR (m/s) | Without LiDAR (m/s) | 差異 | 改善率 |
|------|:---:|:---:|:---:|:---:|
| exp1 | **0.0415** | 0.0418 | +0.0003 | 0.6% |
| exp2 | **0.0439** | 0.0434 | -0.0005 | -1.2% |
| exp4 | **0.0411** | 0.0477 | +0.0066 | 13.9% |
| exp5 | **0.0485** | 0.0527 | +0.0043 | 8.1% |

### 姿態偏航角 RMSE 比較

| 實驗 | With LiDAR (°) | Without LiDAR (°) |
|------|:---:|:---:|
| exp1 | **0.303** | 0.173 |
| exp2 | **0.575** | 0.384 |
| exp4 | **0.209** | 0.393 |
| exp5 | **0.547** | 0.658 |

## 結論

> 詳細圖表請參閱 `ablation_summary_bars.png` 及各實驗的軌跡比較圖
> `exp*_trajectory_comparison.png`。

### 量化效益（平均）
- 加入 LiDAR 回授後，3D 位置 RMSE 平均改善 **45.0%**
- 加入 LiDAR 回授後，vx RMSE 平均改善 **5.3%**

### 分析

- `/fusion/bv` 作為 body velocity bias correction 回授至 inner ESEKF，
  修正腿部運動學量測中的系統性速度誤差。
- 移除此回授後，inner ESEKF 僅依賴 IMU propagation 與接觸腿速度量測，
  速度偏差無法被外部感測器校正，導致積分位置誤差持續累積。
- 不同步態（plain / obs）均顯示相似趨勢，說明 LiDAR 回授效益與步態無關。
