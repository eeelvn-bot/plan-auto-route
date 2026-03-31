import importlib.util
import sys
import unittest
from pathlib import Path

from shapely.geometry import LineString


def _load_plan_auto_route_module():
    module_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "plan_auto_route.py"
    )
    spec = importlib.util.spec_from_file_location("plan_auto_route_under_test", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class LineRiskOverlapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_plan_auto_route_module()

    def test_buffer_overlap_counts_even_without_centerline_overlap(self):
        seg = LineString([(0.0, 120.0), (1000.0, 120.0)])
        line_risk_union = LineString([(0.0, 0.0), (1000.0, 0.0)]).buffer(35.0)

        signals = self.mod._line_risk_overlap_signals(
            seg,
            line_risk_union,
            route_buffer_m=100.0,
        )

        self.assertEqual(signals["center_overlap_ratio"], 0.0)
        self.assertGreater(signals["buffer_overlap_ratio"], 0.0)
        self.assertGreater(signals["effective_overlap_ratio"], 0.0)

    def test_effective_overlap_is_not_weaker_than_centerline_overlap(self):
        seg = LineString([(0.0, 0.0), (1000.0, 0.0)])
        line_risk_union = LineString([(100.0, 0.0), (900.0, 0.0)]).buffer(35.0)

        signals = self.mod._line_risk_overlap_signals(
            seg,
            line_risk_union,
            route_buffer_m=100.0,
        )

        self.assertGreater(signals["center_overlap_ratio"], 0.0)
        self.assertGreaterEqual(
            signals["effective_overlap_ratio"],
            signals["center_overlap_ratio"],
        )

    def test_core_candidate_specs_use_expected_default_weights(self):
        specs = self.mod.build_core_candidate_specs()
        spec_by_id = {spec.id: spec for spec in specs}

        self.assertEqual(spec_by_id["safety_default"].weight_scale["line_cross"], 1.15)
        self.assertEqual(spec_by_id["efficiency"].weight_scale["line_cross"], 0.55)

    def test_core_candidate_specs_allow_weight_overrides(self):
        specs = self.mod.build_core_candidate_specs(
            safety_line_cross_weight=1.1,
            efficiency_line_cross_weight=0.45,
        )
        spec_by_id = {spec.id: spec for spec in specs}

        self.assertEqual(spec_by_id["safety_default"].weight_scale["line_cross"], 1.1)
        self.assertEqual(spec_by_id["efficiency"].weight_scale["line_cross"], 0.45)


if __name__ == "__main__":
    unittest.main()
