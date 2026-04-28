import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon


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


def _identity_transform(x, y, z=None):
    if z is None:
        return x, y
    return x, y, z


class CesiumViewerOutputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_plan_auto_route_module()

    def test_output_paths_add_cesium_files_without_changing_preview_path(self):
        paths = self.mod.build_route_output_paths(Path("/tmp/auto_routes"), "demo_route")

        self.assertEqual(paths["preview_html"], Path("/tmp/auto_routes/demo_route.html"))
        self.assertEqual(paths["cesium_json"], Path("/tmp/auto_routes/demo_route_cesium.json"))
        self.assertEqual(paths["cesium_html"], Path("/tmp/auto_routes/demo_route_3d.html"))

    def test_cesium_payload_contains_route_altitudes_profiles_and_basic_layers(self):
        payload = self.mod.build_cesium_payload(
            name="demo_route",
            candidate_routes_wgs=[
                {
                    "id": "safety_default",
                    "label": "安全优先",
                    "alt_points": [
                        (120.0000, 30.0000, 88.0),
                        (120.0100, 30.0100, 96.5),
                    ],
                    "profile_samples": [
                        {"distance_m": 0.0, "alt_m": 88.0},
                        {"distance_m": 1500.0, "alt_m": 96.5},
                    ],
                    "distance_km": 1.5,
                    "route_buffer_xy": Polygon(
                        [(119.999, 29.999), (120.011, 29.999), (120.011, 30.011), (119.999, 30.011)]
                    ),
                    "show": True,
                }
            ],
            route_points=[(120.0000, 30.0000, 88.0), (120.0100, 30.0100, 96.5)],
            start_wgs=(120.0000, 30.0000),
            end_wgs=(120.0100, 30.0100),
            custom_no_fly_polys_xy=[
                Polygon([(119.99, 29.99), (120.00, 29.99), (120.00, 30.00), (119.99, 30.00)])
            ],
            civil_airport_polys_xy=[],
            military_hard_nofly_polys_xy=[],
            heli_soft_nofly_polys_xy=[],
            existing_route_lines_xy=[LineString([(120.0005, 30.0005), (120.005, 30.005)])],
            existing_route_corridors_xy=[],
            existing_route_relief_union_xy=None,
            dynamic_grb_xy=None,
            landuse_geoms_xy=[
                Polygon([(120.002, 30.002), (120.003, 30.002), (120.003, 30.003), (120.002, 30.003)])
            ],
            landuse_costs=[0.4],
            all_building_polys_xy=[],
            high_building_polys_xy=[
                Polygon([(120.004, 30.004), (120.005, 30.004), (120.005, 30.005), (120.004, 30.005)])
            ],
            crowd_points_xy=[Point(120.006, 30.006)],
            key_points_xy=[],
            infra_geoms_xy=[],
            high_road_lines_xy=[LineString([(120.006, 30.006), (120.007, 30.007)])],
            hsr_lines_xy=[],
            hv_power_lines_xy=[],
            line_risk_union_xy=None,
            school_hard_zones_xy=[],
            school_points_xy=[],
            population_points_wgs=[{"lon": 120.008, "lat": 30.008, "value": 1200}],
            emergency_routes=[],
            quality_variants={},
            layer_source_status={"landuse_source_ready": True},
            inv=_identity_transform,
        )

        self.assertEqual(payload["name"], "demo_route")
        self.assertEqual(payload["schema_version"], "plan-auto-route-cesium-v1")
        self.assertEqual(payload["altitude_reference"], "MSL")
        self.assertEqual(payload["start_wgs84"], {"lon": 120.0, "lat": 30.0})
        self.assertEqual(payload["default_route_id"], "safety_default")
        self.assertEqual(payload["routes"][0]["points"][0]["alt"], 88.0)
        self.assertEqual(payload["routes"][0]["points"][1]["alt"], 96.5)
        self.assertEqual(payload["routes"][0]["corridor"]["horizontal_half_width_m"], 30.0)
        self.assertEqual(payload["routes"][0]["corridor"]["vertical_half_height_m"], 20.0)
        self.assertEqual(payload["routes"][0]["route_buffer_geojson"]["type"], "Polygon")
        self.assertEqual(payload["routes"][0]["profile_samples"][1]["distance_m"], 1500.0)
        layer_ids = {layer["id"] for layer in payload["layers"]}
        self.assertIn("custom_no_fly", layer_ids)
        self.assertIn("population", layer_ids)
        self.assertIn("landuse", layer_ids)
        self.assertIn("high_buildings", layer_ids)
        self.assertIn("line_risk", layer_ids)
        self.assertIn("start_end", layer_ids)
        high_building_layer = next(layer for layer in payload["layers"] if layer["id"] == "high_buildings")
        self.assertEqual(high_building_layer["features"][0]["properties"]["height_m"], 80.0)
        population_layer = next(layer for layer in payload["layers"] if layer["id"] == "population")
        self.assertEqual(population_layer["features"][0]["properties"]["value"], 1200.0)
        self.assertGreaterEqual(payload["metrics"]["route_count"], 1)

    def test_cesium_viewer_html_exposes_basemaps_corridor_and_profile(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            html_path = Path(tmpdir) / "demo_route_3d.html"
            json_path = Path(tmpdir) / "demo_route_cesium.json"

            self.mod.write_cesium_viewer_html(html_path, json_path, "demo_route")
            html = html_path.read_text(encoding="utf-8")

        self.assertIn("普通地图", html)
        self.assertIn("卫星影像", html)
        self.assertIn("卫星注记", html)
        self.assertIn("避让体积", html)
        self.assertIn("addRouteCorridor", html)
        self.assertIn("高度剖面", html)
        self.assertIn("地形", html)
        self.assertIn("航线高度", html)

    def test_cesium_viewer_html_exposes_terrain_corridor_layers_and_profile_linking(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            html_path = Path(tmpdir) / "demo_route_3d.html"
            json_path = Path(tmpdir) / "demo_route_cesium.json"

            self.mod.write_cesium_viewer_html(html_path, json_path, "demo_route")
            html = html_path.read_text(encoding="utf-8")

        self.assertIn("真实地形", html)
        self.assertIn("ArcGISTiledElevationTerrainProvider.fromUrl", html)
        self.assertIn("飞行包络", html)
        self.assertIn("保护外廓", html)
        self.assertIn("addProfileMarker", html)
        self.assertIn("highlightProfileSample", html)
        self.assertIn("data-sample-idx", html)


if __name__ == "__main__":
    unittest.main()
