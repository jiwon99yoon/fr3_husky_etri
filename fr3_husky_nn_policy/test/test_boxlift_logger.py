import csv
import json

import numpy as np
import pytest

from fr3_husky_nn_policy.boxlift_logger import FIELDNAMES, TIMING_FIELDS, BoxLiftRunLogger


def timing(step):
    return {name: float(step) for name in TIMING_FIELDS}


def test_logger_writes_metadata_and_trace(tmp_path):
    logger = BoxLiftRunLogger(tmp_path, "dual fr3/boxlift", {"goal_base": np.asarray([0.5, 0.0, 0.15])})
    assert logger.run_dir.parent.name == "dual_fr3_boxlift"
    for step in range(3):
        logger.append(timing(step), np.full(56, step), np.full(14, 0.5), np.full(14, -1.0), [0, 0, 0, 1])
    run_dir = logger.finalize("operator stop")
    logger.append(timing(9), np.zeros(56), np.zeros(14), np.zeros(14), [0, 0, 0, 1])  # ignored after finalize

    metadata = json.loads((run_dir / "metadata.json").read_text())
    assert metadata["end_reason"] == "operator stop"
    assert metadata["trace_rows"] == 3
    assert metadata["goal_base"] == [0.5, 0.0, 0.15]
    with (run_dir / "trace.csv").open() as handle:
        rows = list(csv.reader(handle))
    assert tuple(rows[0]) == FIELDNAMES
    assert len(rows) == 4
    row = dict(zip(rows[0], rows[2]))
    assert float(row["step"]) == 1.0 and float(row["obs_55"]) == 1.0
    assert float(row["action_13"]) == 0.5 and float(row["q_target_0"]) == -1.0
    assert float(row["box_quat_raw_w"]) == 1.0


def test_logger_rejects_wrong_sizes(tmp_path):
    logger = BoxLiftRunLogger(tmp_path, "boxlift", {})
    with pytest.raises(ValueError, match="observation"):
        logger.append(timing(0), np.zeros(58), np.zeros(14), np.zeros(14), [0, 0, 0, 1])
    with pytest.raises(ValueError, match="missing timing"):
        logger.append({}, np.zeros(56), np.zeros(14), np.zeros(14), [0, 0, 0, 1])
    logger.finalize("test")
