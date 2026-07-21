"""Shared visual style for thesis Sections 5.3 and 5.4 state plots."""
from __future__ import annotations

from collections import OrderedDict
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager


FIGSIZE = (8.2, 6.2)
SUBPLOTS_ADJUST = {
    "left": 0.13,
    "right": 0.97,
    "bottom": 0.10,
    "top": 0.82,
    "hspace": 0.24,
}
SUPTITLE_Y = 0.975
LEGEND_Y = 0.915

FONT_SIZE_FIGURE_TITLE = 13
FONT_SIZE_SUBPLOT_TITLE = 12
FONT_SIZE_AXIS_LABEL = 11
FONT_SIZE_TICK_LABEL = 9
FONT_SIZE_LEGEND = 9
FONT_SIZE_ANNOTATION = 9

LINE_WIDTH = 1.2
GRID_ALPHA = 0.25
GRID_LINE_WIDTH = 0.6

# Contact-state figures use four rows, while retaining the typography and
# figure dimensions of the three-row state-estimation comparisons.
CONTACT_HEIGHT_RATIOS = (2.2, 2.2, 1.45, 0.3)
CONTACT_SUBPLOTS_ADJUST = {
    "left": 0.13,
    "right": 0.97,
    "bottom": 0.10,
    "top": 0.80,
    "hspace": 0.24,
}
CONTACT_LEGEND_BBOX = (0.13, 0.835, 0.84, 0.055)
CONTACT_COLORS = {
    "sigma_rm": "#0072B2",
    "sigma_beta": "#E69F00",
    "threshold": "#4D4D4D",
    "g_height": "#7B2CBF",
    "contact": "#DCE6F2",
    "swing": "#F5E6D3",
    "correct": "#009E73",
    "incorrect": "#D55E00",
}

METHOD_STYLES = OrderedDict([
    ("Ground Truth", {
        "color": "#000000", "linestyle": "-", "zorder": 4,
    }),
    ("Proposed Method", {
        "color": "#0072B2", "linestyle": "-", "zorder": 3,
    }),
    ("IF+KLD", {
        "color": "#E69F00", "linestyle": "--", "zorder": 2,
    }),
    ("IMU Integration", {
        "color": "#D55E00", "linestyle": "-.", "zorder": 1,
    }),
])
TIMES_FONT_DIR = Path("/home/hiho817/.local/share/fonts/times-new-roman")


def configure_style() -> None:
    """Apply the shared publication style and fail if Times is unavailable."""
    installed = sorted(TIMES_FONT_DIR.glob("Times New Roman*.ttf"))
    if len(installed) != 4:
        raise RuntimeError(f"expected four Times New Roman faces in {TIMES_FONT_DIR}")
    for font_path in installed:
        font_manager.fontManager.addfont(font_path)
    font_manager.findfont("Times New Roman", fallback_to_default=False)
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman"],
        "mathtext.fontset": "stix",
        "font.size": FONT_SIZE_TICK_LABEL,
        "figure.titlesize": FONT_SIZE_FIGURE_TITLE,
        "axes.titlesize": FONT_SIZE_SUBPLOT_TITLE,
        "axes.labelsize": FONT_SIZE_AXIS_LABEL,
        "xtick.labelsize": FONT_SIZE_TICK_LABEL,
        "ytick.labelsize": FONT_SIZE_TICK_LABEL,
        "legend.fontsize": FONT_SIZE_LEGEND,
        "axes.unicode_minus": True,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.dpi": 300,
    })


def create_three_panel(title: str):
    configure_style()
    figure, axes = plt.subplots(3, 1, figsize=FIGSIZE, sharex=True)
    figure.subplots_adjust(**SUBPLOTS_ADJUST)
    figure.suptitle(title, y=SUPTITLE_Y, fontsize=FONT_SIZE_FIGURE_TITLE)
    return figure, axes


def plot_method(axis, time, values, method: str):
    style = METHOD_STYLES[method]
    return axis.plot(
        time, values, label=method, linewidth=LINE_WIDTH, alpha=1.0,
        color=style["color"], linestyle=style["linestyle"],
        zorder=style["zorder"],
    )[0]


def format_axis(axis, ylabel: str, xlim=None, ylim=None) -> None:
    axis.set_ylabel(ylabel, fontsize=FONT_SIZE_AXIS_LABEL)
    if xlim is not None:
        axis.set_xlim(*xlim)
    if ylim is not None:
        axis.set_ylim(*ylim)
    axis.grid(True, alpha=GRID_ALPHA, linewidth=GRID_LINE_WIDTH,
              linestyle=":", zorder=0)
    axis.tick_params(axis="both", which="both", direction="in",
                     labelsize=FONT_SIZE_TICK_LABEL)


def finish_figure(figure, axes) -> None:
    handles_by_name = {}
    for axis in axes:
        handles, labels = axis.get_legend_handles_labels()
        handles_by_name.update(zip(labels, handles))
    labels = [name for name in METHOD_STYLES if name in handles_by_name]
    figure.legend(
        [handles_by_name[name] for name in labels], labels,
        loc="upper center", bbox_to_anchor=(0.5, LEGEND_Y),
        ncol=len(labels), frameon=True, fancybox=False, framealpha=1.0,
        edgecolor="#808080", fontsize=FONT_SIZE_LEGEND,
    )
    axes[-1].set_xlabel("Time [s]", fontsize=FONT_SIZE_AXIS_LABEL)


def save_figure(figure, output_stem: Path) -> None:
    """Save identical-geometry PDF and >=300 dpi PNG outputs."""
    figure.savefig(output_stem.with_suffix(".pdf"), format="pdf")
    figure.savefig(output_stem.with_suffix(".png"), format="png", dpi=300)
    plt.close(figure)


def create_contact_figure(title: str):
    """Create the shared four-panel contact-state figure geometry."""
    configure_style()
    figure, axes = plt.subplots(
        4, 1, figsize=FIGSIZE, sharex=True,
        gridspec_kw={"height_ratios": CONTACT_HEIGHT_RATIOS},
    )
    figure.subplots_adjust(**CONTACT_SUBPLOTS_ADJUST)
    figure.suptitle(title, y=SUPTITLE_Y, fontsize=FONT_SIZE_FIGURE_TITLE)
    return figure, axes


def format_contact_axis(axis, ylabel: str, ylim=None) -> None:
    """Apply the common contact-state axes formatting to a data panel."""
    axis.set_ylabel(ylabel, fontsize=FONT_SIZE_AXIS_LABEL)
    if ylim is not None:
        axis.set_ylim(*ylim)
    axis.grid(True, alpha=GRID_ALPHA, linewidth=GRID_LINE_WIDTH,
              linestyle=":", zorder=1)
    axis.tick_params(axis="both", which="both", direction="in",
                     labelsize=FONT_SIZE_TICK_LABEL)


def finish_contact_figure(figure, axes, handles) -> None:
    """Place a fixed two-row legend between title and first subplot."""
    figure.legend(
        handles=handles, loc="upper left", bbox_to_anchor=CONTACT_LEGEND_BBOX,
        mode="expand", ncol=5, frameon=True, fancybox=False, framealpha=1.0,
        edgecolor="#808080", fontsize=FONT_SIZE_LEGEND,
    )
    axes[-1].set_xlabel("Time [s]", fontsize=FONT_SIZE_AXIS_LABEL)
    axes[-1].tick_params(axis="both", which="both", direction="in",
                         labelsize=FONT_SIZE_TICK_LABEL)
