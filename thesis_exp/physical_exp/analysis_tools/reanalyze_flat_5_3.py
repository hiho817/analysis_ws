#!/usr/bin/env python3
"""Recalculate Section 5.3 from existing metrics; never replay ROS bags."""
from __future__ import annotations

import json
from pathlib import Path
from statistics import mean, stdev


ROOT = Path("/home/hiho817/analysis_ws/thesis_exp/physical_exp")
OUT = ROOT / "results" / "5.3_flat_experiment"


SELECTED = {
    "NEW_WALK": [
        "FLAT_Walk_NEW_REAL_1",
        "FLAT_Walk_NEW_REAL_5",
        "FLAT_Walk_NEW_REAL_6",
    ],
    "NEW_WLW": [
        "FLAT_WLW_NEW_REAL_4",
        "FLAT_WLW_NEW_REAL_5",
        "FLAT_WLW_NEW_REAL_2",
    ],
    "OLD_WALK": [
        "FLAT_Walk_OLD_REAL_1",
        "FLAT_Walk_OLD_REAL_2",
        "FLAT_Walk_OLD_REAL_3",
    ],
    "OLD_WLW": [
        "FLAT_WLW_OLD_REAL_1",
        "FLAT_WLW_OLD_REAL_2",
        "FLAT_WLW_OLD_REAL_3",
    ],
}
PATTERNS = {
    "NEW_WALK": "FLAT_Walk_NEW_REAL_*",
    "OLD_WALK": "FLAT_Walk_OLD_REAL_*",
    "NEW_WLW": "FLAT_WLW_NEW_REAL_*",
    "OLD_WLW": "FLAT_WLW_OLD_REAL_*",
}


def load_existing_metrics() -> tuple[dict[str, list[dict]], dict[str, list[str]]]:
    groups, excluded = {}, {}
    for group, pattern in PATTERNS.items():
        paths = sorted((ROOT / "experiments").glob(f"{pattern}/results/*/metrics.json"))
        records = [json.loads(path.read_text()) for path in paths]
        selected = set(SELECTED[group])
        groups[group] = [record for record in records if record["exp_id"] in selected]
        excluded[group] = [record["exp_id"] for record in records
                           if record["exp_id"] not in selected]
    return groups, excluded


def summary(records: list[dict]) -> dict:
    fields = {
        "position_rmse_x_cm": ("position", "RMSE_X_cm"),
        "position_rmse_y_cm": ("position", "RMSE_Y_cm"),
        "position_rmse_z_cm": ("position", "RMSE_Z_cm"),
        "position_rmse_3d_cm": ("position", "RMSE_3D_cm"),
        "velocity_rmse_vx": ("velocity", "RMSE_vx"),
        "velocity_rmse_vy": ("velocity", "RMSE_vy"),
        "velocity_rmse_vz": ("velocity", "RMSE_vz"),
        "velocity_rmse_3d": ("velocity", "RMSE_3D"),
    }
    result = {"n": len(records), "experiment_ids": [r["exp_id"] for r in records]}
    for name, (section, key) in fields.items():
        values = [float(r[section][key]) for r in records]
        result[name] = {"values": values, "mean": mean(values),
                        "sample_std": stdev(values) if len(values) > 1 else 0.0}
    if all(record.get("attitude") for record in records):
        for axis in ("roll", "pitch", "yaw"):
            values = [float(record["attitude"][f"RMSE_{axis}_deg"])
                      for record in records]
            result[f"attitude_rmse_{axis}_deg"] = {
                "values": values,
                "mean": mean(values),
                "sample_std": stdev(values) if len(values) > 1 else 0.0,
            }
    return result


def fmt(item: dict, digits: int = 2) -> str:
    return f"{item['mean']:.{digits}f} ± {item['sample_std']:.{digits}f}"


def fmt_position_m(item: dict) -> str:
    """Format a position statistic stored internally in centimetres as metres."""
    return f"{item['mean']/100.0:.3f} ± {item['sample_std']/100.0:.3f}"


def statistics_si(stats: dict) -> dict:
    """Return a JSON-ready copy with position fields expressed in metres."""
    converted = {}
    for group, values in stats.items():
        converted[group] = {}
        for key, value in values.items():
            if key.startswith("position_rmse_") and key.endswith("_cm"):
                converted[group][key[:-3] + "_m"] = {
                    "values": [x / 100.0 for x in value["values"]],
                    "mean": value["mean"] / 100.0,
                    "sample_std": value["sample_std"] / 100.0,
                }
            else:
                converted[group][key] = value
    return converted


def make_report(groups: dict[str, list[dict]], stats: dict,
                excluded: dict[str, list[str]]) -> tuple[str, dict]:
    lines = [
        "# 5.3 平地實驗",
        "",
        "## 5.3.1 資料選取與分析方法",
        "",
        "本節僅重新彙整既有 `metrics.json` 中的狀態估測結果，不重新執行節點、不 replay bag，亦不分析觸地狀態。為使 NEW 與 OLD 的樣本數一致，四組皆只納入三筆試驗。NEW Walk 採 REAL_1、REAL_5、REAL_6，OLD Walk 採 REAL_1、REAL_2、REAL_3；NEW WLW 採 REAL_4、REAL_5、REAL_2，OLD WLW 採 REAL_1、REAL_2、REAL_3。其餘 trial 全部排除於分組平均值、標準差與結論之外。",
        "",
        "### 納入與排除清單",
        "",
        "| 組別 | 納入統計 | 排除統計 |",
        "|---|---|---|",
        f"| NEW Walk | {', '.join(stats['NEW_WALK']['experiment_ids'])} | {', '.join(excluded['NEW_WALK']) or '—'} |",
        f"| OLD Walk | {', '.join(stats['OLD_WALK']['experiment_ids'])} | {', '.join(excluded['OLD_WALK']) or '—'} |",
        f"| NEW WLW | {', '.join(stats['NEW_WLW']['experiment_ids'])} | {', '.join(excluded['NEW_WLW']) or '—'} |",
        f"| OLD WLW | {', '.join(stats['OLD_WLW']['experiment_ids'])} | {', '.join(excluded['OLD_WLW']) or '—'} |",
        "",
        "## 5.3.2 位置與速度估測結果",
        "",
        "| 步態 | 方法 | n | 位置 RMSE X / Y / Z [m] | 位置 RMSE 3D [m] | 速度 RMSE vx / vy / vz [m/s] | 速度 RMSE 3D [m/s] |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for gait, label in (("WALK", "Walk"), ("WLW", "WLW")):
        for method, method_label in (("NEW", "NEW（ES-EKF）"), ("OLD", "OLD（Legacy）")):
            s = stats[f"{method}_{gait}"]
            pos = " / ".join(fmt_position_m(s[f"position_rmse_{axis}_cm"]) for axis in "xyz")
            vel = " / ".join(fmt(s[f"velocity_rmse_v{axis}"], 3) for axis in "xyz")
            lines.append(f"| {label} | {method_label} | {s['n']} | {pos} | {fmt_position_m(s['position_rmse_3d_cm'])} | {vel} | {fmt(s['velocity_rmse_3d'], 3)} |")
    lines += ["", "| 步態 | NEW 位置 RMSE 3D | OLD 位置 RMSE 3D | 位置降幅 | NEW 速度 RMSE 3D | OLD 速度 RMSE 3D | 速度降幅 |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    comparisons = {}
    for gait, label in (("WALK", "Walk"), ("WLW", "WLW")):
        new, old = stats[f"NEW_{gait}"], stats[f"OLD_{gait}"]
        p_reduction = (1 - new["position_rmse_3d_cm"]["mean"] / old["position_rmse_3d_cm"]["mean"]) * 100
        v_reduction = (1 - new["velocity_rmse_3d"]["mean"] / old["velocity_rmse_3d"]["mean"]) * 100
        comparisons[gait] = {"position_reduction_pct": p_reduction,
                             "velocity_reduction_pct": v_reduction}
        lines.append(f"| {label} | {fmt_position_m(new['position_rmse_3d_cm'])} m | {fmt_position_m(old['position_rmse_3d_cm'])} m | {p_reduction:.1f}% | {fmt(new['velocity_rmse_3d'], 3)} m/s | {fmt(old['velocity_rmse_3d'], 3)} m/s | {v_reduction:.1f}% |")
    lines += [
        "",
        f"Walk 三筆 NEW 的位置 3D RMSE 為 **{fmt_position_m(stats['NEW_WALK']['position_rmse_3d_cm'])} m**，相較三筆 OLD 的 **{fmt_position_m(stats['OLD_WALK']['position_rmse_3d_cm'])} m** 降低 **{comparisons['WALK']['position_reduction_pct']:.1f}%**；速度 3D RMSE 降低 **{comparisons['WALK']['velocity_reduction_pct']:.1f}%**。",
        "",
        f"WLW 三筆 NEW 的位置 3D RMSE 為 **{fmt_position_m(stats['NEW_WLW']['position_rmse_3d_cm'])} m**，相較三筆 OLD 的 **{fmt_position_m(stats['OLD_WLW']['position_rmse_3d_cm'])} m** 降低 **{comparisons['WLW']['position_reduction_pct']:.1f}%**；速度 3D RMSE 降低 **{comparisons['WLW']['velocity_reduction_pct']:.1f}%**。四組樣本數皆為三筆，但 NEW 與 OLD 的試驗編號並非一一配對，因此本節報告描述統計與改善比例，不進行配對顯著性檢定。",
        "",
        "### 納入試驗之個別位置與速度結果",
        "",
        "| Trial | 組別 | 位置 RMSE 3D [m] | 速度 RMSE 3D [m/s] |",
        "|---|---|---:|---:|",
    ]
    for group in ("NEW_WALK", "OLD_WALK", "NEW_WLW", "OLD_WLW"):
        for record in groups[group]:
            lines.append(f"| {record['exp_id']} | {group} | {record['position']['RMSE_3D_cm']/100.0:.3f} | {record['velocity']['RMSE_3D']:.3f} |")
    lines += [
        "",
        "## 5.3.3 姿態估測結果",
        "",
        "姿態 RMSE 僅分析 NEW（ES-EKF），因 OLD（Legacy）未估測姿態，故無 Roll、Pitch 與 Yaw 可供比較。",
        "",
        "| 步態 | 方法 | n | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for gait, label in (("WALK", "Walk"), ("WLW", "WLW")):
        new = stats[f"NEW_{gait}"]
        lines.append(
            f"| {label} | NEW（ES-EKF） | {new['n']} | "
            f"{fmt(new['attitude_rmse_roll_deg'])} | "
            f"{fmt(new['attitude_rmse_pitch_deg'])} | "
            f"{fmt(new['attitude_rmse_yaw_deg'])} |")
        lines.append(f"| {label} | OLD（Legacy） | 3 | — | — | — |")
    lines += [
        "",
        "### NEW 個別試驗姿態結果",
        "",
        "| Trial | 步態 | Roll RMSE [deg] | Pitch RMSE [deg] | Yaw RMSE [deg] |",
        "|---|---|---:|---:|---:|",
    ]
    for group, gait_label in (("NEW_WALK", "Walk"), ("NEW_WLW", "WLW")):
        for record in groups[group]:
            attitude = record["attitude"]
            lines.append(
                f"| {record['exp_id']} | {gait_label} | "
                f"{attitude['RMSE_roll_deg']:.3f} | "
                f"{attitude['RMSE_pitch_deg']:.3f} | "
                f"{attitude['RMSE_yaw_deg']:.3f} |")
    lines += [
        "",
        f"Walk NEW 的 Roll、Pitch 與 Yaw RMSE 分別為 **{fmt(stats['NEW_WALK']['attitude_rmse_roll_deg'])} deg**、**{fmt(stats['NEW_WALK']['attitude_rmse_pitch_deg'])} deg** 與 **{fmt(stats['NEW_WALK']['attitude_rmse_yaw_deg'])} deg**。WLW NEW 則分別為 **{fmt(stats['NEW_WLW']['attitude_rmse_roll_deg'])} deg**、**{fmt(stats['NEW_WLW']['attitude_rmse_pitch_deg'])} deg** 與 **{fmt(stats['NEW_WLW']['attitude_rmse_yaw_deg'])} deg**。",
        "",
        "## 5.3.4 小結",
        "",
        f"三筆試驗的結果顯示，Walk 與 WLW 的 NEW 位置 3D RMSE 分別為 **{fmt_position_m(stats['NEW_WALK']['position_rmse_3d_cm'])} m** 與 **{fmt_position_m(stats['NEW_WLW']['position_rmse_3d_cm'])} m**。相較 OLD，位置誤差分別降低 **{comparisons['WALK']['position_reduction_pct']:.1f}%** 與 **{comparisons['WLW']['position_reduction_pct']:.1f}%**，速度誤差則降低 **{comparisons['WALK']['velocity_reduction_pct']:.1f}%** 與 **{comparisons['WLW']['velocity_reduction_pct']:.1f}%**。NEW 的 Walk Roll/Pitch RMSE 低於 1 deg，Yaw RMSE 為 1.02 deg；WLW 三軸姿態 RMSE 均低於 1 deg。OLD 未提供姿態估測。本結果完全由既有估測指標重新彙整，不包含觸地分析或 replay 後數據。",
        "",
    ]
    return "\n".join(lines), comparisons


def main() -> None:
    groups, excluded = load_existing_metrics()
    stats = {group: summary(records) for group, records in groups.items()}
    report, comparisons = make_report(groups, stats, excluded)
    payload = {
        "method": "estimation metrics recalculation only; no contact analysis or ROS bag replay",
        "selection": SELECTED,
        "excluded_from_statistics": excluded,
        "group_statistics": statistics_si(stats),
        "comparisons": comparisons,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "metrics_reanalysis.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (OUT / "5.3_平地實驗.md").write_text(report, encoding="utf-8")
    print(f"Wrote {OUT / 'metrics_reanalysis.json'}")
    print(f"Wrote {OUT / '5.3_平地實驗.md'}")


if __name__ == "__main__":
    main()
