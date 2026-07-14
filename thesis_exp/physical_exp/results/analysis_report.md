# 實機實驗分析總報告

本報告彙整 `experiments/*/results/*/metrics.json`。請先個別執行各實驗的 `analyze.py`，再執行本程式更新本報告。

- 已找到試驗：49 筆
- 納入統計：49 筆
- 排除統計：0 筆

## 分組統計

| 分組 | n | Position RMSE (cm) | Velocity RMSE (m/s) |
|---|---:|---:|---:|
| NEW_MPC | 5 | 6.02 ± 1.41 | 0.06 ± 0.00 |
| NEW_OBS_MPC_GMO | 5 | 13.00 ± 7.85 | 0.10 ± 0.04 |
| NEW_RUGG_WALK | 4 | 12.73 ± 6.33 | 0.13 ± 0.04 |
| NEW_WALK | 5 | 7.10 ± 1.73 | 0.07 ± 0.00 |
| NEW_WLW | 5 | 5.55 ± 0.51 | 0.03 ± 0.00 |
| OLD_MPC | 5 | 15.37 ± 1.90 | 0.06 ± 0.00 |
| OLD_OBS_MPC | 5 | 21.87 ± 6.64 | 0.11 ± 0.02 |
| OLD_RUGG_WALK | 5 | 40.51 ± 4.78 | 0.12 ± 0.01 |
| OLD_WALK | 5 | 26.16 ± 0.88 | 0.09 ± 0.00 |
| OLD_WLW | 5 | 11.02 ± 1.36 | 0.05 ± 0.00 |

## WLW NEW 接觸狀態（校正後）

VICON 真值採腳標記高度 < 20 mm；GMO OR-Schmitt 門檻採 `F_rm` 進／離地 50/25、`tau_beta` 進／離地 3.00/1.50。

排除 `FLAT_WLW_NEW_REAL_1` 與 `FLAT_WLW_NEW_REAL_3`：兩筆的 `F_rm` 與 VICON swing 同相，不能作為此門檻校正的驗證樣本。

| 實驗 | 四腳平均 Acc | 總有效時間步 | TP | TN | FP | FN |
|---|---:|---:|---:|---:|---:|---:|
| FLAT_WLW_NEW_REAL_2 | 75.3% | 69892 | 42155 | 10450 | 1754 | 15533 |
| FLAT_WLW_NEW_REAL_4 | 76.3% | 69420 | 42737 | 10201 | 1822 | 14660 |
| FLAT_WLW_NEW_REAL_5 | 75.8% | 73782 | 44670 | 11271 | 1820 | 16021 |

納入三筆試驗的加權整體 Acc：**75.8%**（N = 213094）。

## 個別試驗

| 實驗 | 分組 | Position RMSE (cm) | 排除統計 | metrics |
|---|---|---:|---|---|
| FLAT_MPC_NEW_REAL_1 | NEW_MPC | 6.16 | 否 | `experiments/FLAT_MPC_NEW_REAL_1/results/FLAT_MPC_NEW_REAL_1/metrics.json` |
| FLAT_MPC_NEW_REAL_2 | NEW_MPC | 4.89 | 否 | `experiments/FLAT_MPC_NEW_REAL_2/results/FLAT_MPC_NEW_REAL_2/metrics.json` |
| FLAT_MPC_NEW_REAL_3 | NEW_MPC | 5.30 | 否 | `experiments/FLAT_MPC_NEW_REAL_3/results/FLAT_MPC_NEW_REAL_3/metrics.json` |
| FLAT_MPC_NEW_REAL_4 | NEW_MPC | 8.40 | 否 | `experiments/FLAT_MPC_NEW_REAL_4/results/FLAT_MPC_NEW_REAL_4/metrics.json` |
| FLAT_MPC_NEW_REAL_5 | NEW_MPC | 5.36 | 否 | `experiments/FLAT_MPC_NEW_REAL_5/results/FLAT_MPC_NEW_REAL_5/metrics.json` |
| FLAT_MPC_OLD_REAL_1 | OLD_MPC | 17.87 | 否 | `experiments/FLAT_MPC_OLD_REAL_1/results/FLAT_MPC_OLD_REAL_1/metrics.json` |
| FLAT_MPC_OLD_REAL_2 | OLD_MPC | 12.60 | 否 | `experiments/FLAT_MPC_OLD_REAL_2/results/FLAT_MPC_OLD_REAL_2/metrics.json` |
| FLAT_MPC_OLD_REAL_3 | OLD_MPC | 15.03 | 否 | `experiments/FLAT_MPC_OLD_REAL_3/results/FLAT_MPC_OLD_REAL_3/metrics.json` |
| FLAT_MPC_OLD_REAL_4 | OLD_MPC | 15.39 | 否 | `experiments/FLAT_MPC_OLD_REAL_4/results/FLAT_MPC_OLD_REAL_4/metrics.json` |
| FLAT_MPC_OLD_REAL_5 | OLD_MPC | 15.97 | 否 | `experiments/FLAT_MPC_OLD_REAL_5/results/FLAT_MPC_OLD_REAL_5/metrics.json` |
| FLAT_WLW_NEW_REAL_1 | NEW_WLW | 6.38 | 否 | `experiments/FLAT_WLW_NEW_REAL_1/results/FLAT_WLW_NEW_REAL_1/metrics.json` |
| FLAT_WLW_NEW_REAL_2 | NEW_WLW | 5.05 | 否 | `experiments/FLAT_WLW_NEW_REAL_2/results/FLAT_WLW_NEW_REAL_2/metrics.json` |
| FLAT_WLW_NEW_REAL_3 | NEW_WLW | 5.59 | 否 | `experiments/FLAT_WLW_NEW_REAL_3/results/FLAT_WLW_NEW_REAL_3/metrics.json` |
| FLAT_WLW_NEW_REAL_4 | NEW_WLW | 5.52 | 否 | `experiments/FLAT_WLW_NEW_REAL_4/results/FLAT_WLW_NEW_REAL_4/metrics.json` |
| FLAT_WLW_NEW_REAL_5 | NEW_WLW | 5.21 | 否 | `experiments/FLAT_WLW_NEW_REAL_5/results/FLAT_WLW_NEW_REAL_5/metrics.json` |
| FLAT_WLW_OLD_REAL_1 | OLD_WLW | 12.72 | 否 | `experiments/FLAT_WLW_OLD_REAL_1/results/FLAT_WLW_OLD_REAL_1/metrics.json` |
| FLAT_WLW_OLD_REAL_2 | OLD_WLW | 9.91 | 否 | `experiments/FLAT_WLW_OLD_REAL_2/results/FLAT_WLW_OLD_REAL_2/metrics.json` |
| FLAT_WLW_OLD_REAL_3 | OLD_WLW | 9.98 | 否 | `experiments/FLAT_WLW_OLD_REAL_3/results/FLAT_WLW_OLD_REAL_3/metrics.json` |
| FLAT_WLW_OLD_REAL_4 | OLD_WLW | 10.22 | 否 | `experiments/FLAT_WLW_OLD_REAL_4/results/FLAT_WLW_OLD_REAL_4/metrics.json` |
| FLAT_WLW_OLD_REAL_5 | OLD_WLW | 12.26 | 否 | `experiments/FLAT_WLW_OLD_REAL_5/results/FLAT_WLW_OLD_REAL_5/metrics.json` |
| FLAT_Walk_NEW_REAL_1 | NEW_WALK | 6.47 | 否 | `experiments/FLAT_Walk_NEW_REAL_1/results/FLAT_Walk_NEW_REAL_1/metrics.json` |
| FLAT_Walk_NEW_REAL_3 | NEW_WALK | 5.92 | 否 | `experiments/FLAT_Walk_NEW_REAL_3/results/FLAT_Walk_NEW_REAL_3/metrics.json` |
| FLAT_Walk_NEW_REAL_4 | NEW_WALK | 6.29 | 否 | `experiments/FLAT_Walk_NEW_REAL_4/results/FLAT_Walk_NEW_REAL_4/metrics.json` |
| FLAT_Walk_NEW_REAL_5 | NEW_WALK | 10.15 | 否 | `experiments/FLAT_Walk_NEW_REAL_5/results/FLAT_Walk_NEW_REAL_5/metrics.json` |
| FLAT_Walk_NEW_REAL_6 | NEW_WALK | 6.68 | 否 | `experiments/FLAT_Walk_NEW_REAL_6/results/FLAT_Walk_NEW_REAL_6/metrics.json` |
| FLAT_Walk_OLD_REAL_1 | OLD_WALK | 27.00 | 否 | `experiments/FLAT_Walk_OLD_REAL_1/results/FLAT_Walk_OLD_REAL_1/metrics.json` |
| FLAT_Walk_OLD_REAL_2 | OLD_WALK | 25.14 | 否 | `experiments/FLAT_Walk_OLD_REAL_2/results/FLAT_Walk_OLD_REAL_2/metrics.json` |
| FLAT_Walk_OLD_REAL_3 | OLD_WALK | 25.40 | 否 | `experiments/FLAT_Walk_OLD_REAL_3/results/FLAT_Walk_OLD_REAL_3/metrics.json` |
| FLAT_Walk_OLD_REAL_4 | OLD_WALK | 26.22 | 否 | `experiments/FLAT_Walk_OLD_REAL_4/results/FLAT_Walk_OLD_REAL_4/metrics.json` |
| FLAT_Walk_OLD_REAL_5 | OLD_WALK | 27.04 | 否 | `experiments/FLAT_Walk_OLD_REAL_5/results/FLAT_Walk_OLD_REAL_5/metrics.json` |
| OBS_MPC_NEW_REAL_3 | NEW_OBS_MPC_GMO | 8.68 | 否 | `experiments/OBS_MPC_NEW_REAL_3/results/OBS_MPC_NEW_REAL_3/metrics.json` |
| OBS_MPC_NEW_REAL_4 | NEW_OBS_MPC_GMO | 5.95 | 否 | `experiments/OBS_MPC_NEW_REAL_4/results/OBS_MPC_NEW_REAL_4/metrics.json` |
| OBS_MPC_NEW_REAL_5 | NEW_OBS_MPC_GMO | 14.28 | 否 | `experiments/OBS_MPC_NEW_REAL_5/results/OBS_MPC_NEW_REAL_5/metrics.json` |
| OBS_MPC_NEW_REAL_6 | NEW_OBS_MPC_GMO | 10.11 | 否 | `experiments/OBS_MPC_NEW_REAL_6/results/OBS_MPC_NEW_REAL_6/metrics.json` |
| OBS_MPC_NEW_REAL_7 | NEW_OBS_MPC_GMO | 25.98 | 否 | `experiments/OBS_MPC_NEW_REAL_7/results/OBS_MPC_NEW_REAL_7/metrics.json` |
| OBS_MPC_OLD_REAL_1 | OLD_OBS_MPC | 32.74 | 否 | `experiments/OBS_MPC_OLD_REAL_1/results/OBS_MPC_OLD_REAL_1/metrics.json` |
| OBS_MPC_OLD_REAL_2 | OLD_OBS_MPC | 19.84 | 否 | `experiments/OBS_MPC_OLD_REAL_2/results/OBS_MPC_OLD_REAL_2/metrics.json` |
| OBS_MPC_OLD_REAL_3 | OLD_OBS_MPC | 15.23 | 否 | `experiments/OBS_MPC_OLD_REAL_3/results/OBS_MPC_OLD_REAL_3/metrics.json` |
| OBS_MPC_OLD_REAL_4 | OLD_OBS_MPC | 18.82 | 否 | `experiments/OBS_MPC_OLD_REAL_4/results/OBS_MPC_OLD_REAL_4/metrics.json` |
| OBS_MPC_OLD_REAL_5 | OLD_OBS_MPC | 22.73 | 否 | `experiments/OBS_MPC_OLD_REAL_5/results/OBS_MPC_OLD_REAL_5/metrics.json` |
| RUGG_Walk_NEW_REAL_1 | NEW_RUGG_WALK | 18.01 | 否 | `experiments/RUGG_Walk_NEW_REAL_1/results/RUGG_Walk_NEW_REAL_1/metrics.json` |
| RUGG_Walk_NEW_REAL_2 | NEW_RUGG_WALK | 7.03 | 否 | `experiments/RUGG_Walk_NEW_REAL_2/results/RUGG_Walk_NEW_REAL_2/metrics.json` |
| RUGG_Walk_NEW_REAL_3 | NEW_RUGG_WALK | 18.40 | 否 | `experiments/RUGG_Walk_NEW_REAL_3/results/RUGG_Walk_NEW_REAL_3/metrics.json` |
| RUGG_Walk_NEW_REAL_5 | NEW_RUGG_WALK | 7.48 | 否 | `experiments/RUGG_Walk_NEW_REAL_5/results/RUGG_Walk_NEW_REAL_5/metrics.json` |
| RUGG_Walk_OLD_REAL_1 | OLD_RUGG_WALK | 45.73 | 否 | `experiments/RUGG_Walk_OLD_REAL_1/results/RUGG_Walk_OLD_REAL_1/metrics.json` |
| RUGG_Walk_OLD_REAL_2 | OLD_RUGG_WALK | 32.88 | 否 | `experiments/RUGG_Walk_OLD_REAL_2/results/RUGG_Walk_OLD_REAL_2/metrics.json` |
| RUGG_Walk_OLD_REAL_3 | OLD_RUGG_WALK | 40.13 | 否 | `experiments/RUGG_Walk_OLD_REAL_3/results/RUGG_Walk_OLD_REAL_3/metrics.json` |
| RUGG_Walk_OLD_REAL_4 | OLD_RUGG_WALK | 42.90 | 否 | `experiments/RUGG_Walk_OLD_REAL_4/results/RUGG_Walk_OLD_REAL_4/metrics.json` |
| RUGG_Walk_OLD_REAL_5 | OLD_RUGG_WALK | 40.90 | 否 | `experiments/RUGG_Walk_OLD_REAL_5/results/RUGG_Walk_OLD_REAL_5/metrics.json` |
