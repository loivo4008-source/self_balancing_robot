"""
Generate a portfolio-quality static plot from a robot_logger.py CSV log.

Usage:
    python plot_robot_log.py robot_log_20260716_143022.csv
    python plot_robot_log.py robot_log_20260716_143022.csv --out balance_demo.png
"""

import argparse
import sys

import matplotlib.pyplot as plt
import pandas as pd


def load_log(csv_path):
    df = pd.read_csv(csv_path)
    # Convert to seconds since the start of the log, for a friendlier x-axis
    df["t_s"] = (df["timestamp_ms"] - df["timestamp_ms"].iloc[0]) / 1000.0
    return df


def plot_log(df, title, out_path):
    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]},
    )
    fig.suptitle(title, fontsize=14, fontweight="bold")

    # --- Top: pitch angle vs setpoint ---
    ax1.plot(df["t_s"], df["pitch"], label="Pitch angle",
             linewidth=1.6, color="#2563eb")
    ax1.plot(df["t_s"], df["setpoint"], "--",
             label="Setpoint", linewidth=1.2, color="#94a3b8")
    ax1.axhline(0, color="#e2e8f0", linewidth=0.8, zorder=0)
    ax1.set_ylabel("Angle (deg)")
    ax1.legend(loc="upper right", frameon=False)
    ax1.grid(True, alpha=0.25)

    # Annotate peak overshoot after the largest disturbance, if there is one
    peak_idx = df["pitch"].abs().idxmax()
    peak_t, peak_val = df.loc[peak_idx, "t_s"], df.loc[peak_idx, "pitch"]
    ax1.annotate(
        f"peak {peak_val:.1f}°",
        xy=(peak_t, peak_val),
        xytext=(peak_t + df["t_s"].max() * 0.03, peak_val),
        fontsize=9, color="#475569",
        arrowprops=dict(arrowstyle="->", color="#94a3b8", lw=1),
    )

    # --- Bottom: PID terms ---
    ax2.plot(df["t_s"], df["P"], label="P", linewidth=1, color="#dc2626")
    ax2.plot(df["t_s"], df["I"], label="I", linewidth=1, color="#16a34a")
    ax2.plot(df["t_s"], df["D"], label="D", linewidth=1, color="#9333ea")
    ax2.set_ylabel("PID terms")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper right", frameon=False)
    ax2.grid(True, alpha=0.25)

    for ax in (ax1, ax2):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot a robot_logger.py CSV run")
    parser.add_argument("csv_path", help="Path to the logged CSV file")
    parser.add_argument("--out", default=None,
                        help="Output image filename (default: same name, .png)")
    parser.add_argument(
        "--title", default="Balancing Robot — Pitch and PID Response", help="Chart title")
    args = parser.parse_args()

    df = load_log(args.csv_path)
    out_path = args.out or args.csv_path.rsplit(".", 1)[0] + ".png"
    plot_log(df, args.title, out_path)


if __name__ == "__main__":
    main()
