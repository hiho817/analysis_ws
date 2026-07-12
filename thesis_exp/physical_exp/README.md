# 論文實機實驗資料集

每個 `experiments/<experiment_name>/` 都是可獨立重跑與微調的單位：

- `bags/`：該筆 ROS 2 bag（僅複製 manifest 納入、未排除的資料）。
- `vicon/`：與該 bag 對應的 VICON CSV。
- `analyze.py`：只執行此筆實驗的入口。
- `analyze_impl.py`：自原始批次分析程式保留的分析邏輯與該筆 manifest 設定；要做單筆分析調整時編輯此檔。
- `results/`：從原始批次整理而來的 `metrics.json`，以及重新分析後的圖表。所有新產生圖表均為 PDF；原始 PNG 圖表不複製。

## 執行

先載入 ROS 2 工作區，再在任一實驗目錄執行：

```bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash
python3 analyze.py
```

在本資料夾根目錄執行下列命令，將所有已存在的 `metrics.json` 彙整為 `results/analysis_report.md`：

```bash
python3 analyze_report.py
```

`common/corgi_analysis/` 為所有實驗共同使用的 VICON、bag 載入、繪圖與指標計算方法。

總報告同時會產生 `results/fig_group_position_rmse.pdf`。
