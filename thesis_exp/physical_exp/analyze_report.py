#!/usr/bin/env python3
"""Aggregate metrics from the copied physical experiments into a thesis report."""
from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
METRICS = sorted(ROOT.glob("experiments/*/results/*/metrics.json"))
OUT = ROOT / "results" / "analysis_report.md"

# Contact calibration is validated only on these WLW NEW trials.  REAL_1 and
# REAL_3 are intentionally excluded because their F_rm residual is phase-aligned
# with swing rather than a usable contact cue.
CONTACT_REPORT_IDS = {
    "FLAT_WLW_NEW_REAL_2",
    "FLAT_WLW_NEW_REAL_4",
    "FLAT_WLW_NEW_REAL_5",
}


def number(value):
    return value if isinstance(value, (int, float)) else None


def summary(values):
    values = [v for v in values if v is not None]
    if not values:
        return "—"
    if len(values) == 1:
        return f"{values[0]:.2f}"
    return f"{mean(values):.2f} ± {stdev(values):.2f}"


def main():
    records = []
    for path in METRICS:
        with path.open() as f:
            record = json.load(f)
        record["source"] = path.relative_to(ROOT)
        records.append(record)

    included = [r for r in records if not r.get("exclude_stats", False)]
    groups = defaultdict(list)
    for record in included:
        groups[record.get("group", "UNSPECIFIED")].append(record)

    lines = [
        "# 實機實驗分析總報告",
        "",
        "本報告彙整 `experiments/*/results/*/metrics.json`。請先個別執行各實驗的 `analyze.py`，再執行本程式更新本報告。",
        "",
        f"- 已找到試驗：{len(records)} 筆",
        f"- 納入統計：{len(included)} 筆",
        f"- 排除統計：{len(records) - len(included)} 筆",
        "",
        "## 分組統計",
        "",
        "| 分組 | n | Position RMSE (cm) | Velocity RMSE (m/s) |",
        "|---|---:|---:|---:|",
    ]
    for group, items in sorted(groups.items()):
        position = [number(i.get("position", {}).get("RMSE_3D_cm")) or number(i.get("position", {}).get("RMSE_2D_cm")) for i in items]
        velocity = [number(i.get("velocity", {}).get("RMSE_3D")) for i in items]
        lines.append(f"| {group} | {len(items)} | {summary(position)} | {summary(velocity)} |")

    group_names = sorted(groups)
    group_means = []
    for group in group_names:
        values = [number(i.get("position", {}).get("RMSE_3D_cm")) or number(i.get("position", {}).get("RMSE_2D_cm")) for i in groups[group]]
        values = [v for v in values if v is not None]
        group_means.append(mean(values) if values else 0.0)
    if group_names:
        fig, ax = plt.subplots(figsize=(max(7, len(group_names) * 1.25), 4.5))
        ax.bar(group_names, group_means, color="#3b82a0")
        ax.set_ylabel("Position RMSE [cm]")
        ax.set_title("Position RMSE by experiment group")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout()
        fig.savefig(OUT.parent / "fig_group_position_rmse.pdf", bbox_inches="tight")
        plt.close(fig)

    contact_records = [
        r for r in records
        if r.get("exp_id") in CONTACT_REPORT_IDS and r.get("contact")
    ]
    lines += [
        "",
        "## WLW NEW 接觸狀態（校正後）",
        "",
        "VICON 真值採腳標記高度 < 20 mm；GMO OR-Schmitt 門檻採 `F_rm` 進／離地 50/25、`tau_beta` 進／離地 3.00/1.50。",
        "",
        "排除 `FLAT_WLW_NEW_REAL_1` 與 `FLAT_WLW_NEW_REAL_3`：兩筆的 `F_rm` 與 VICON swing 同相，不能作為此門檻校正的驗證樣本。",
        "",
        "| 實驗 | 四腳平均 Acc | 總有效時間步 | TP | TN | FP | FN |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    total = {key: 0 for key in ("N", "TP", "TN", "FP", "FN")}
    for r in sorted(contact_records, key=lambda x: x.get("exp_id", "")):
        legs = list(r["contact"].values())
        acc = mean(float(c["acc"]) for c in legs)
        values = {key: sum(int(c[key]) for c in legs) for key in total}
        for key in total:
            total[key] += values[key]
        lines.append(
            f"| {r['exp_id']} | {acc:.1%} | {values['N']} | {values['TP']} | "
            f"{values['TN']} | {values['FP']} | {values['FN']} |"
        )
    if contact_records:
        weighted_acc = (total["TP"] + total["TN"]) / total["N"]
        lines += [
            "",
            f"納入三筆試驗的加權整體 Acc：**{weighted_acc:.1%}**（N = {total['N']}）。",
        ]

    lines += [
        "",
        "## 個別試驗",
        "",
        "| 實驗 | 分組 | Position RMSE (cm) | 排除統計 | metrics |",
        "|---|---|---:|---|---|",
    ]
    for r in sorted(records, key=lambda x: x.get("exp_id", "")):
        pos = number(r.get("position", {}).get("RMSE_3D_cm")) or number(r.get("position", {}).get("RMSE_2D_cm"))
        lines.append(f"| {r.get('exp_id', '—')} | {r.get('group', '—')} | {('%.2f' % pos) if pos is not None else '—'} | {'是' if r.get('exclude_stats') else '否'} | `{r['source']}` |")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
