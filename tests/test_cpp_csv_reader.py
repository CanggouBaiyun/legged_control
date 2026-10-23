"""Preserve missing swing fields and detect damaged native simulation logs."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "cpp_csv_summary", Path(__file__).resolve().parents[1] / "tools/summarize_cpp_control.py")
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def test_missing_swing_and_rejected_request_nan(tmp_path):
    path = tmp_path / "control.csv"
    path.write_text("time_s,swing_target_z,requested_vx_mps\n0,,nan\n.002,.1,0\n")
    data = summary.load_numeric_csv(path)
    assert data.shape == (2,)
    np.testing.assert_array_equal(data["time_s"], [0, .002])
    assert np.isnan(data["swing_target_z"][0])
    assert np.isnan(data["requested_vx_mps"][0])
    assert data["swing_target_z"][1] == .1


def test_single_and_zero_rows_remain_one_dimensional(tmp_path):
    path = tmp_path / "control.csv"
    path.write_text("time_s,value\n0,1\n")
    assert summary.load_numeric_csv(path).shape == (1,)
    path.write_text("time_s,value\n")
    with pytest.warns(UserWarning, match="input contained no data"):
        assert summary.load_numeric_csv(path).shape == (0,)


@pytest.mark.parametrize("content", ["time_s,value\n0,1\n.002,2,3\n", "time_s,value\n0,broken\n"])
def test_damaged_numeric_rows_are_rejected(tmp_path, content):
    path = tmp_path / "control.csv"
    path.write_text(content)
    with pytest.raises(ValueError):
        summary.load_numeric_csv(path)


def test_duplicate_header_is_rejected(tmp_path):
    path = tmp_path / "control.csv"
    path.write_text("time_s,time_s\n0,0\n")
    with pytest.raises(RuntimeError, match="duplicate"):
        summary.load_numeric_csv(path)
