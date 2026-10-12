#!/usr/bin/env python3
"""Offline regression for Uno text protocol, signed altitude and rate display.

Importing app starts only an idle SerialWorker; tests never open a port, connect
serial hardware, reset a board, issue radio commands, or flash firmware.
"""
import importlib.util
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP_PATH = ROOT / "groundstation" / "backend" / "app.py"
spec = importlib.util.spec_from_file_location("bozok_groundstation_app_test", APP_PATH)
app = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = app
spec.loader.exec_module(app)

BASE_STATE = dict(app.state)


def reset_state():
    app.state.clear()
    app.state.update(BASE_STATE)
    app._rate_times.clear()
    app.recording["active"] = False
    app.recording["fh"] = None


def line(seq, alt, t_ms=None):
    tick = seq * 100 if t_ms is None else t_ms
    return (f"[LINK] seq={seq} t={tick}ms flight=2 ek7mode=0 alt={alt} "
            "vel=0cm/s sats=0 batt=-- fix=N LED=--- tilt=-- lean=-- pos=--")


class TestGroundStationProtocol(unittest.TestCase):
    def setUp(self):
        reset_state()

    def test_fractional_negative_altitude_is_preserved(self):
        app.apply_link([line(1, "-0.4m")])
        self.assertAlmostEqual(app.state["alt_m"], -0.4)
        self.assertFalse(app.state["stale"])

    def test_decimal_altitude_over_1000m_is_preserved(self):
        app.apply_link([line(1, "1234.5m")])
        self.assertAlmostEqual(app.state["alt_m"], 1234.5)

    def test_legacy_integer_metre_frame_text_remains_compatible(self):
        app.apply_link([line(1, "1250m")])
        self.assertEqual(app.state["alt_m"], 1250)

    def test_unknown_altitude_clears_previous_value(self):
        app.apply_link([line(1, "1234.5m")])
        app.apply_link([line(2, "--")])
        self.assertIsNone(app.state["alt_m"])

    def test_sentinels_and_out_of_range_altitude_are_rejected(self):
        app.apply_link([line(1, "-32768m")])
        self.assertIsNone(app.state["alt_m"])
        self.assertGreaterEqual(app.state["range_rej"], 1)
        app.apply_link([line(2, "214748364.8m")])
        self.assertIsNone(app.state["alt_m"])
        self.assertGreaterEqual(app.state["range_rej"], 2)

    def test_rate_estimator_reports_10hz_not_window_fraction(self):
        ticks = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        self.assertEqual(app._measured_rate_hz(ticks), 10.0)

    def test_rate_estimator_handles_single_sample_and_duplicate_timestamps(self):
        self.assertEqual(app._measured_rate_hz([1.0]), 0.0)
        self.assertEqual(app._measured_rate_hz([1.0, 1.0]), 0.0)
        self.assertEqual(app._measured_rate_hz([0.0, 0.5, 1.0]), 2.0)

    def test_stale_threshold_tracks_10hz_stream(self):
        self.assertEqual(app.stale_threshold_ms(10.0), 500.0)
        self.assertEqual(app.stale_threshold_ms(2.0), 1500.0)
        self.assertEqual(app.stale_threshold_ms(0.0), 500.0)

    def test_sequence_wrap_keeps_packet_and_altitude(self):
        app.apply_link([line(65535, "12.3m")])
        app.apply_link([line(0, "12.4m")])
        self.assertEqual(app.state["gaps"], 0)
        self.assertAlmostEqual(app.state["alt_m"], 12.4)


if __name__ == "__main__":
    result = unittest.main(verbosity=2, exit=False).result
    if result.wasSuccessful():
        print("GROUNDSTATION_PROTOCOL_TESTS_OK")
    else:
        raise SystemExit(1)
