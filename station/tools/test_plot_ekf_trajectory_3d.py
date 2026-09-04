#!/usr/bin/env python3
"""EKF 三维轨迹切段和默认目录的回归测试

cd D:\ForSTM32\xinjian\Underwater-Glider

python station\tools\plot_ekf_trajectory_3d.py --date 20260805
"""

import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
import pandas as pd

import plot_ekf_logs
import plot_ekf_trajectory_3d as trajectory


class TrajectorySegmentTests(unittest.TestCase):
    def test_invalid_rows_and_time_gaps_break_the_line(self):
        frame = pd.DataFrame(
            {
                "status_name": ["RUNNING"] * 6,
                "valid": [1, 1, 0, 1, 1, 1],
                "pN": [0.0, 1.0, 2.0, 3.0, 4.0, np.nan],
                "pE": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],
                "depth": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                "t_ms": [0, 100, 200, 1000, 1100, 1200],
                "elapsed_min": np.array([0, 100, 200, 1000, 1100, 1200])
                / 60000.0,
            }
        )

        valid, segments, segment_time, connected = trajectory.trajectory_segments(
            frame, gap_seconds=0.5
        )

        self.assertEqual(valid.tolist(), [True, True, False, True, True, False])
        self.assertEqual(connected.tolist(), [True, False, False, True, False])
        self.assertEqual(segments.shape, (2, 2, 3))
        self.assertEqual(segment_time.shape, (2,))

    def test_non_running_rows_are_not_valid_trajectory_points(self):
        frame = pd.DataFrame(
            {
                "status_name": ["ALIGNING", "RUNNING"],
                "valid": [1, 1],
                "pN": [0.0, 1.0],
                "pE": [0.0, 1.0],
                "depth": [0.0, 1.0],
                "t_ms": [0, 100],
                "elapsed_min": [0.0, 0.1 / 60.0],
            }
        )

        valid, segments, _, connected = trajectory.trajectory_segments(
            frame, gap_seconds=0.5
        )

        self.assertEqual(valid.tolist(), [False, True])
        self.assertEqual(connected.tolist(), [False])
        self.assertEqual(len(segments), 0)


class AnalysisDefaultPathTests(unittest.TestCase):
    def test_both_plotters_use_station_directories(self):
        station_dir = Path(__file__).resolve().parent.parent
        with mock.patch.object(sys, "argv", ["plot_ekf_logs.py"]):
            overview_args = plot_ekf_logs.parse_args()
        with mock.patch.object(sys, "argv", ["plot_ekf_trajectory_3d.py"]):
            trajectory_args = trajectory.parse_args()

        expected_input = station_dir / "logs" / "ekf"
        expected_output = station_dir / "plots"
        self.assertEqual(overview_args.input_dir, expected_input)
        self.assertEqual(overview_args.output_dir, expected_output)
        self.assertEqual(trajectory_args.input_dir, expected_input)
        self.assertEqual(trajectory_args.output_dir, expected_output)


if __name__ == "__main__":
    unittest.main()
