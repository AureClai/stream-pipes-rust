"""Shared matplotlib style matching the paper's existing figures (fig7-fig9).

Scheme extracted from the exported field-validation figures: Segoe UI-like
sans font, y-only light grid, open axes (no top/right spines), thick lines,
frameless legends. Color convention: observed = blue, simulated = orange.
"""

import matplotlib as mpl

BLUE = "#1f77b4"    # observed
ORANGE = "#ff7f0e"  # simulated
GRAY = "#4C5258"    # secondary series / ink-dim
LIGHTGRAY = "#9aa3ab"


def apply() -> None:
    mpl.rcParams.update(
        {
            "font.family": ["Segoe UI", "DejaVu Sans"],
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.edgecolor": "#b0b0b0",
            "axes.linewidth": 0.9,
            "axes.grid": True,
            "axes.grid.axis": "y",
            "grid.color": "#dddddd",
            "grid.linewidth": 0.8,
            "axes.axisbelow": True,
            "xtick.color": "#666666",
            "ytick.color": "#666666",
            "axes.labelcolor": "#333333",
            "text.color": "#333333",
            "lines.linewidth": 2.2,
            "legend.frameon": False,
        }
    )
