#!/usr/bin/env python3

"""
plot_logs.py - Plot time-series data from ROS2 simulation CSV logs.

Usage:
    python3 plot_logs.py <folder_path> <csv_file1> [<csv_file2> ...]

Supports wildcards:
    python3 plog . "odom*" imu.csv

Each CSV file must contain:
    - A 'Timestamp' column in ISO 8601 format (e.g., 2025-07-08 12:34:56.789)
    - A 'Value' column with numerical data

Author: AVOSUGHI
Created: July 2025
"""

import argparse
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt

# === CONFIGURATION ===
GAP_THRESHOLD_SECONDS = 0.5  # max gap to interpolate

def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot CSV logs with shared time axis.")
    parser.add_argument("folder", help="Path to the folder containing CSV files.")
    parser.add_argument("patterns", nargs='+', help="List of CSV filename patterns (e.g., '*.csv', 'odom*').")
    return parser.parse_args()

def parse_csv(file_path):
    df = pd.read_csv(file_path)
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])
    return df

def main():
    args = parse_arguments()
    folder = args.folder
    patterns = args.patterns

    # Expand patterns into actual file paths
    all_files = []
    for pattern in patterns:
        matched_files = glob.glob(os.path.join(folder, pattern))
        if not matched_files:
            print(f"[Warning] No files matched pattern: {pattern}")
        all_files.extend(matched_files)

    if not all_files:
        print("[Error] No CSV files found to plot.")
        return

    plt.figure(figsize=(12, 6))
    
    for file_path in all_files:
        try:
            print(f"[Plotting] {file_path}")
            df = parse_csv(file_path)
            label = os.path.basename(file_path)
            plt.scatter(df['Timestamp'], df['Value'], label=label, s=15)
        except Exception as e:
            print(f"[Error] Failed to parse {file_path}: {e}")

    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.title("Data Logger Plot")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
