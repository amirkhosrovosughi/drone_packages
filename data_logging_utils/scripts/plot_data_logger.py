#!/usr/bin/env python3

"""
plot_logs.py - Plot time-series data from ROS2 simulation CSV logs.

Usage:
    python3 plot_logs.py <folder_path> <csv_file1> [<csv_file2> ...]

Example:
    python3 plot_logs.py ./ros2_logs odom.csv imu.csv

Each CSV file must contain:
    - A 'Timestamp' column in ISO 8601 format (e.g., 2025-07-08 12:34:56.789)
    - A 'Value' column with numerical data

The script will:
    - Load each CSV file
    - Parse the Timestamp column into datetime
    - Plot each file's Value vs Time on the same graph

Dependencies:
    - pandas
    - matplotlib

Author: AVOSUGHI
Created: July2025
"""

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# === CONFIGURATION ===
GAP_THRESHOLD_SECONDS = 0.5  # max gap to interpolate #TODO: interpolate when timetags does not match

def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot CSV logs with shared time axis.")
    parser.add_argument("folder", help="Path to the folder containing CSV files.")
    parser.add_argument("files", nargs='+', help="List of CSV filenames to plot.")
    return parser.parse_args()

def parse_csv(file_path):
    df = pd.read_csv(file_path)
    df['Timestamp'] = pd.to_datetime(df['Timestamp'])

    return df

def main():
    args = parse_arguments()
    folder = args.folder
    csv_files = args.files

    plt.figure(figsize=(12, 6))
    
    for file in csv_files:
        file_path = os.path.join(folder, file)
        print("......", file_path)
        if not os.path.exists(file_path):
            print(f"[Warning] File does not exist: {file_path}")
            continue
        
        try:
            df = parse_csv(file_path)
            plt.plot(df['Timestamp'].to_numpy(), df['Value'].to_numpy(), label=file)
        except Exception as e:
            print(f"[Error] Failed to parse {file}: {e}")


    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.title("Data Logger Plot")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()