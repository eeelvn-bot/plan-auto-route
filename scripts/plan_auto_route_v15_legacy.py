#!/usr/bin/env python3
"""Automatic urban UAV route planner with open-data constraints and altitude profiling."""

from __future__ import annotations

import argparse
import hashlib
import heapq
import html
import json
import math
import numbers
import re
import subprocess
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib import error, parse, request

import folium
import pyproj
from osgeo import gdal
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.ops import transform, unary_union
from shapely.strtree import STRtree

DEFAULT_OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]

DEFAULT_OPENTOPO_ENDPOINT = "https://api.opentopodata.org/v1/srtm90m"
ALGORITHM_VERSION = "plan-auto-route-latest-air-corridor-v3-true-height-bounded"
OVERPASS_CACHE_DIR = Path(__file__).resolve().parents[3] / "output" / "overpass_cache"

DEFAULT_AIRFRAME = {
    "speed_ms": 12.0,
    "climb_ms": 3.0,
    "descend_ms": 2.5,
    "turn_radius_m": 35.0,
    "clearance_m": 30.0,
}

GRAPH_NODE_SNAP_M = 8.0
AIR_LATTICE_STEP_M = 420.0
AIR_LATTICE_MARGIN_M = 800.0
AIR_LATTICE_MAX_OFFSET_M = 3200.0
ROUTE_BUFFER_M = 100.0
HIGH_BUILDING_THRESHOLD_M = 80.0
HIGH_BUILDING_AVOID_BUFFER_M = 35.0
WATER_PREF_MIN_FACTOR = 0.18
WATER_PREF_MAX_FACTOR = 1.0
SCHOOL_HARD_BUFFER_M = 90.0
SCHOOL_ENDPOINT_RELIEF_M = 55.0
PREFERRED_CRUISE_ALT_M = 120.0
HARD_CEILING_ALT_M = 200.0
MIN_TRUE_HEIGHT_M = 60.0
MAX_TRUE_HEIGHT_M = 120.0
ENDPOINT_TRUE_HEIGHT_M = 60.0
VERTICAL_DESCEND_ENERGY_FACTOR = 0.65
CRUISE_BASE_QUANTILE = 0.22
CRUISE_SOFT_CAP_RELAX_M = 2.0

TURN_IGNORE_DEG = 14.0
TURN_SHARP_DEG = 95.0
TURN_FIXED_SCALE = 0.35
LOW_VALUE_TURN_KEEP_DEG = 26.0
SHORTCUT_MAX_LEN_RATIO = 1.01
ALT_PROFILE_SPACING_M = 25.0
ALT_PROFILE_LINK_ERR_TOL_M = 1.8

WEIGHT_PROFILES = {
    "fastest": {
        "length": 1.0,
        "population": 0.8,
        "landuse": 0.5,
        "infrastructure": 1.1,
        "altitude": 0.4,
        "turn": 50.0,
        "soft_no_fly": 1.2,
        "crowd": 1.0,
        "key_facility": 1.2,
        "line_cross": 2.2,
        "high_building": 3.5,
    },
    "balanced": {
        "length": 1.0,
        "population": 1.6,
        "landuse": 1.2,
        "infrastructure": 2.0,
        "altitude": 0.8,
        "turn": 90.0,
        "soft_no_fly": 2.2,
        "crowd": 1.8,
        "key_facility": 2.2,
        "line_cross": 4.0,
        "high_building": 6.0,
    },
    "safest": {
        "length": 1.1,
        "population": 2.6,
        "landuse": 2.0,
        "infrastructure": 2.8,
        "altitude": 1.3,
        "turn": 120.0,
        "soft_no_fly": 3.5,
        "crowd": 2.8,
        "key_facility": 3.4,
        "line_cross": 5.5,
        "high_building": 8.5,
    },
}

LANDUSE_COST_RULES = [
    ("natural:water", 0.15),
    ("waterway:", 0.2),
    ("leisure:park", 0.35),
    ("natural:wood", 0.45),
    ("landuse:forest", 0.5),
    ("landuse:grass", 0.5),
    ("landuse:farmland", 0.6),
    ("landuse:industrial", 1.6),
    ("landuse:commercial", 1.7),
    ("landuse:retail", 1.7),
    ("landuse:residential", 2.0),
]

CROWD_POI_TYPES = {
    "amenity:school",
    "amenity:kindergarten",
    "amenity:college",
    "amenity:university",
    "amenity:hospital",
    "amenity:clinic",
    "amenity:bus_station",
    "amenity:marketplace",
    "amenity:place_of_worship",
    "amenity:cinema",
    "amenity:theatre",
    "amenity:ferry_terminal",
    "tourism:attraction",
    "tourism:museum",
    "tourism:viewpoint",
    "leisure:stadium",
    "leisure:sports_centre",
    "shop:mall",
    "shop:supermarket",
}

KEY_FACILITY_POI_TYPES = {
    "amenity:police",
    "amenity:fire_station",
    "amenity:courthouse",
    "amenity:townhall",
    "office:government",
    "amenity:research_institute",
}


def run_cmd(cmd: List[str]) -> None:
    print("RUN:", " ".join(str(x) for x in cmd), flush=True)
    subprocess.run(cmd, check=True)


def parse_kml_coords(kml_path: Path) -> List[Tuple[float, float, float]]:
    text = kml_path.read_text(encoding="utf-8")
    match = re.search(r"<LineString>.*?<coordinates>(.*?)</coordinates>", text, re.DOTALL)
    if not match:
        match = re.search(r"<coordinates>(.*?)</coordinates>", text, re.DOTALL)
    if not match:
        raise ValueError(f"No coordinates found in KML: {kml_path}")
    out: List[Tuple[float, float, float]] = []
    for token in match.group(1).strip().split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        lon = float(parts[0])
        lat = float(parts[1])
        alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
        out.append((lon, lat, alt))
    if len(out) < 2:
        raise ValueError(f"Need at least two coordinates in KML: {kml_path}")
    return out


def _summary_complete(summary: Dict[str, Any]) -> bool:
    outputs = summary.get("outputs", {})
    required = ["poi", "landuse", "population", "transport"]
    return all(k in outputs for k in required)


def _find_city_cache_dir(root: Path, city: str) -> Optional[Path]:
    direct = root / "output" / "city_data_cache" / city
    if direct.exists():
        return direct
    candidates = sorted((root / "output" / "city_data_cache").glob("*"))
    city_norm = city.strip()
    for c in candidates:
        if c.name == city_norm:
            return c
    return None


def ensure_city_data(root: Path, city: str, zoom: str = "8-14") -> Path:
    city_dir = root / "output" / "city_data_cache" / city
    city_dir.mkdir(parents=True, exist_ok=True)
    summary_path = city_dir / "download_summary.json"
    if summary_path.exists():
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            if _summary_complete(summary):
                print(f"[SKIP] city data exists: {city} -> {city_dir}", flush=True)
                return summary_path
        except Exception:
            pass
    script = root / "skills" / "city-data-downloader" / "scripts" / "download_city_data.py"
    run_cmd(
        [
            "python3",
            str(script),
            "--city",
            city,
            "--outdir",
            str(city_dir),
            "--zoom",
            zoom,
        ]
    )
    return summary_path


def build_projectors(points_wgs: List[Tuple[float, float]]) -> Tuple[Any, Any]:
    center_lon = sum(x for x, _ in points_wgs) / len(points_wgs)
    center_lat = sum(y for _, y in points_wgs) / len(points_wgs)
    zone = int((center_lon + 180.0) / 6.0) + 1
    epsg = (32700 if center_lat < 0 else 32600) + zone
    fwd = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True).transform
    inv = pyproj.Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True).transform
    return fwd, inv


def route_bbox_wgs(points_wgs: List[Tuple[float, float]], margin_m: float) -> Tuple[float, float, float, float]:
    lons = [p[0] for p in points_wgs]
    lats = [p[1] for p in points_wgs]
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)
    lat_mid = (min_lat + max_lat) * 0.5
    dlat = margin_m / 111320.0
    dlon = margin_m / max(1e-6, 111320.0 * math.cos(math.radians(lat_mid)))
    return min_lat - dlat, max_lat + dlat, min_lon - dlon, max_lon + dlon


def run_overpass(query: str, timeout: int = 120) -> Dict[str, Any]:
    OVERPASS_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cache_key = hashlib.sha1(query.encode("utf-8")).hexdigest()
    cache_path = OVERPASS_CACHE_DIR / f"{cache_key}.json"
    if cache_path.exists():
        try:
            cached = json.loads(cache_path.read_text(encoding="utf-8"))
            if isinstance(cached, dict) and ("elements" in cached):
                return cached
        except Exception:
            pass
    body = parse.urlencode({"data": query}).encode("utf-8")
    headers = {
        "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
        "User-Agent": "plan-auto-route/1.0",
    }
    last_exc: Optional[Exception] = None
    for endpoint in DEFAULT_OVERPASS_ENDPOINTS:
        for attempt in range(2):
            try:
                req = request.Request(endpoint, data=body, headers=headers, method="POST")
                with request.urlopen(req, timeout=timeout + 20) as resp:
                    obj = json.loads(resp.read().decode("utf-8"))
                    try:
                        cache_path.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")
                    except Exception:
                        pass
                    return obj
            except (error.HTTPError, error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                last_exc = exc
                if attempt < 1:
                    time.sleep(1.0)
    if cache_path.exists():
        try:
            cached = json.loads(cache_path.read_text(encoding="utf-8"))
            if isinstance(cached, dict):
                return cached
        except Exception:
            pass
    raise RuntimeError(f"Overpass failed: {last_exc}")


def load_geojson_features(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    return obj.get("features", [])


def _parse_raw_tags(raw: Any) -> Dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str) and raw.strip():
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, dict):
                return parsed
        except Exception:
            return {}
    return {}


def _first_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    match = re.search(r"[-+]?\d+(\.\d+)?", text)
    if not match:
        return None
    try:
        return float(match.group(0))
    except Exception:
        return None


def _point_from_geojson_geometry(geom_obj: Dict[str, Any]) -> Optional[Point]:
    if not geom_obj:
        return None
    gtype = str(geom_obj.get("type", "")).strip()
    coords = geom_obj.get("coordinates")
    try:
        if gtype == "Point" and isinstance(coords, list) and len(coords) >= 2:
            return Point(float(coords[0]), float(coords[1]))
        g = shape(geom_obj)
        if g.is_empty:
            return None
        return g.representative_point()
    except Exception:
        return None


def build_poi_risk_indices(
    summary: Dict[str, Any],
    fwd,
) -> Tuple[List[Any], Optional[STRtree], Dict[int, int], List[Any], Optional[STRtree], Dict[int, int], Dict[str, int]]:
    outputs = summary.get("outputs", {})
    poi_geojson = Path(outputs.get("poi", {}).get("geojson", ""))
    crowd_points_xy: List[Any] = []
    key_points_xy: List[Any] = []
    counter = {"crowd": 0, "key_facility": 0}
    for feat in load_geojson_features(poi_geojson):
        props = feat.get("properties") or {}
        poi_type = str(props.get("poi_type", "")).strip().lower()
        raw_tags = _parse_raw_tags(props.get("raw_tags"))
        if not poi_type:
            candidates = []
            for k in ("amenity", "office", "shop", "tourism", "leisure"):
                if raw_tags.get(k):
                    candidates.append(f"{k}:{str(raw_tags.get(k)).strip().lower()}")
            if candidates:
                poi_type = candidates[0]
        is_crowd = poi_type in CROWD_POI_TYPES
        is_key = poi_type in KEY_FACILITY_POI_TYPES
        if (not is_crowd) and (not is_key):
            office = str(raw_tags.get("office", "")).strip().lower()
            amenity = str(raw_tags.get("amenity", "")).strip().lower()
            if office == "government":
                is_key = True
            if amenity in {"hospital", "school", "kindergarten", "bus_station"}:
                is_crowd = True
        if not is_crowd and not is_key:
            continue
        pt_wgs = _point_from_geojson_geometry(feat.get("geometry") or {})
        if pt_wgs is None:
            continue
        pt_xy = _to_xy_geometry(pt_wgs, fwd)
        if pt_xy is None:
            continue
        if is_crowd:
            crowd_points_xy.append(pt_xy)
            counter["crowd"] += 1
        if is_key:
            key_points_xy.append(pt_xy)
            counter["key_facility"] += 1
    crowd_tree = STRtree(crowd_points_xy) if crowd_points_xy else None
    key_tree = STRtree(key_points_xy) if key_points_xy else None
    return (
        crowd_points_xy,
        crowd_tree,
        _build_geom_id_map(crowd_points_xy),
        key_points_xy,
        key_tree,
        _build_geom_id_map(key_points_xy),
        counter,
    )


def build_line_risk_geometries(
    summary: Dict[str, Any],
    route_bbox: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], Any, List[Any], Dict[str, int]]:
    outputs = summary.get("outputs", {})
    roads_geojson = Path(outputs.get("transport", {}).get("roads_geojson", ""))
    hsr_geojson = Path(outputs.get("transport", {}).get("hsr_geojson", ""))
    road_lines_xy: List[Any] = []
    hsr_lines_xy: List[Any] = []
    c_road = 0
    c_hsr = 0
    high_road_types = {"motorway", "trunk", "primary"}
    south, north, west, east = route_bbox
    bbox_poly_wgs = Polygon([(west, south), (east, south), (east, north), (west, north), (west, south)])
    bbox_poly_xy = _to_xy_geometry(bbox_poly_wgs, fwd)
    for feat in load_geojson_features(roads_geojson):
        props = feat.get("properties") or {}
        hwy = str(props.get("highway", "")).strip().lower()
        if hwy not in high_road_types:
            continue
        try:
            g = shape(feat.get("geometry") or {})
        except Exception:
            continue
        g_xy = _to_xy_geometry(g, fwd)
        if g_xy is None:
            continue
        if bbox_poly_xy is not None and (not g_xy.intersects(bbox_poly_xy)):
            continue
        road_lines_xy.append(g_xy)
        c_road += 1
    for feat in load_geojson_features(hsr_geojson):
        try:
            g = shape(feat.get("geometry") or {})
        except Exception:
            continue
        g_xy = _to_xy_geometry(g, fwd)
        if g_xy is None:
            continue
        if bbox_poly_xy is not None and (not g_xy.intersects(bbox_poly_xy)):
            continue
        hsr_lines_xy.append(g_xy)
        c_hsr += 1

    query = f"""
[out:json][timeout:120];
(
  way({south},{west},{north},{east})[highway~"^(motorway|trunk|primary)$"];
  way({south},{west},{north},{east})[railway=rail][highspeed=yes];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        data = {"elements": []}
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        coords = _element_coords_wgs(el)
        if len(coords) < 2:
            continue
        g_wgs = LineString(coords)
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        hwy = str(tags.get("highway", "")).strip().lower()
        railway = str(tags.get("railway", "")).strip().lower()
        highspeed = str(tags.get("highspeed", "")).strip().lower()
        if hwy in high_road_types:
            road_lines_xy.append(g_xy)
            c_road += 1
        elif railway == "rail" and highspeed in {"yes", "designated"}:
            hsr_lines_xy.append(g_xy)
            c_hsr += 1

    line_risk_geoms = [g.buffer(35.0) for g in road_lines_xy] + [g.buffer(30.0) for g in hsr_lines_xy]
    line_risk_union = unary_union(line_risk_geoms) if line_risk_geoms else None
    return road_lines_xy, line_risk_union, hsr_lines_xy, {"highway": c_road, "hsr": c_hsr}


class PopulationSampler:
    def __init__(self, tif_path: Optional[Path]) -> None:
        self.data = None
        self.gt = None
        self.nodata = None
        self.width = 0
        self.height = 0
        if tif_path is None or (not tif_path.exists()):
            return
        ds = gdal.Open(str(tif_path))
        if ds is None:
            return
        rb = ds.GetRasterBand(1)
        data = rb.ReadAsArray()
        if data is None:
            return
        self.data = data
        self.gt = ds.GetGeoTransform()
        self.nodata = rb.GetNoDataValue()
        self.height, self.width = data.shape

    def sample(self, lon: float, lat: float) -> float:
        if self.data is None or self.gt is None:
            return 0.0
        gt = self.gt
        px = int((lon - gt[0]) / gt[1])
        py = int((lat - gt[3]) / gt[5])
        if px < 0 or py < 0 or px >= self.width or py >= self.height:
            return 0.0
        val = float(self.data[py][px])
        if self.nodata is not None and val == self.nodata:
            return 0.0
        if math.isnan(val) or val < 0:
            return 0.0
        return val


class TerrainSampler:
    def __init__(self, dem_tif: Optional[Path], opentopo_endpoint: str) -> None:
        self.dem = PopulationSampler(dem_tif) if dem_tif is not None else PopulationSampler(None)
        self.use_dem = self.dem.data is not None
        self.endpoint = opentopo_endpoint

    def sample_points(self, points_wgs: List[Tuple[float, float]]) -> Tuple[List[float], str]:
        if self.use_dem:
            out = [self.dem.sample(lon, lat) for lon, lat in points_wgs]
            return out, "dem_tif"
        out = sample_terrain_opentopodata(points_wgs, self.endpoint)
        return out, "opentopodata"


class TerrainGridSampler:
    def __init__(
        self,
        min_x: float,
        min_y: float,
        step_m: float,
        cols: int,
        rows: int,
        values: List[float],
    ) -> None:
        self.min_x = min_x
        self.min_y = min_y
        self.step_m = max(20.0, step_m)
        self.cols = max(1, cols)
        self.rows = max(1, rows)
        self.values = values

    def sample_xy(self, x: float, y: float) -> float:
        if not self.values:
            return 0.0
        ix = int(round((x - self.min_x) / self.step_m))
        iy = int(round((y - self.min_y) / self.step_m))
        ix = max(0, min(self.cols - 1, ix))
        iy = max(0, min(self.rows - 1, iy))
        idx = iy * self.cols + ix
        if 0 <= idx < len(self.values):
            return max(0.0, float(self.values[idx]))
        return 0.0


def build_terrain_grid_sampler(
    route_bbox_wgs_vals: Tuple[float, float, float, float],
    fwd,
    inv,
    endpoint: str,
    step_m: float = 280.0,
) -> Optional[TerrainGridSampler]:
    south, north, west, east = route_bbox_wgs_vals
    try:
        p1 = transform(fwd, Point(west, south)).coords[0]
        p2 = transform(fwd, Point(east, north)).coords[0]
    except Exception:
        return None
    min_x = min(p1[0], p2[0])
    max_x = max(p1[0], p2[0])
    min_y = min(p1[1], p2[1])
    max_y = max(p1[1], p2[1])
    if (max_x - min_x) < 10.0 or (max_y - min_y) < 10.0:
        return None
    cols = max(3, int((max_x - min_x) / step_m) + 1)
    rows = max(3, int((max_y - min_y) / step_m) + 1)
    points_wgs: List[Tuple[float, float]] = []
    for iy in range(rows):
        y = min_y + iy * step_m
        for ix in range(cols):
            x = min_x + ix * step_m
            lon, lat = transform(inv, Point(x, y)).coords[0]
            points_wgs.append((lon, lat))
    vals = sample_terrain_opentopodata(points_wgs, endpoint)
    if not vals:
        return None
    return TerrainGridSampler(min_x=min_x, min_y=min_y, step_m=step_m, cols=cols, rows=rows, values=vals)


def sample_terrain_opentopodata(points_wgs: List[Tuple[float, float]], endpoint: str) -> List[float]:
    if not points_wgs:
        return []
    headers = {"User-Agent": "plan-auto-route/1.0"}
    out: List[float] = []
    batch_size = 90
    for i in range(0, len(points_wgs), batch_size):
        batch = points_wgs[i : i + batch_size]
        locations = "|".join(f"{lat:.6f},{lon:.6f}" for lon, lat in batch)
        url = f"{endpoint}?{parse.urlencode({'locations': locations})}"
        try:
            req = request.Request(url, headers=headers)
            with request.urlopen(req, timeout=25) as resp:
                obj = json.loads(resp.read().decode("utf-8"))
            res = obj.get("results", [])
            for item in res:
                val = _first_float(item.get("elevation"))
                out.append(float(val) if val is not None else 0.0)
            if len(res) < len(batch):
                out.extend([0.0] * (len(batch) - len(res)))
        except Exception:
            out.extend([0.0] * len(batch))
    return out[: len(points_wgs)]


def _landuse_cost(landuse_type: str) -> float:
    token = str(landuse_type).lower().strip()
    if not token:
        return 1.0
    for prefix, cost in LANDUSE_COST_RULES:
        if token.startswith(prefix):
            return cost
    return 1.0


def _build_geom_id_map(geoms: List[Any]) -> Dict[int, int]:
    return {id(g): i for i, g in enumerate(geoms)}


def _item_to_index(item: Any, geoms: List[Any], geom_id_map: Dict[int, int]) -> Optional[int]:
    if isinstance(item, numbers.Integral):
        idx = int(item)
        if 0 <= idx < len(geoms):
            return idx
        return None
    idx = geom_id_map.get(id(item))
    if idx is not None:
        return idx
    for i, g in enumerate(geoms):
        try:
            if item.equals(g):
                return i
        except Exception:
            continue
    return None


def _tree_candidate_indices(tree: Optional[STRtree], geom, geoms: List[Any], geom_id_map: Dict[int, int]) -> List[int]:
    if tree is None:
        return []
    try:
        out = tree.query(geom)
    except Exception:
        return []
    if out is None:
        return []
    candidates: List[int] = []
    for item in out:
        idx = _item_to_index(item, geoms, geom_id_map)
        if idx is not None:
            candidates.append(idx)
    return candidates


def _nearest_index_and_distance(
    tree: Optional[STRtree],
    geom,
    geoms: List[Any],
    geom_id_map: Dict[int, int],
) -> Tuple[Optional[int], float]:
    if tree is None or not geoms:
        return None, float("inf")
    try:
        near = tree.nearest(geom)
    except Exception:
        return None, float("inf")
    idx = _item_to_index(near, geoms, geom_id_map)
    if idx is None:
        return None, float("inf")
    return idx, geom.distance(geoms[idx])


def _count_geoms_intersecting(
    tree: Optional[STRtree],
    geom,
    geoms: List[Any],
    geom_id_map: Dict[int, int],
) -> int:
    if tree is None or not geoms:
        return 0
    cands = _tree_candidate_indices(tree, geom, geoms, geom_id_map)
    count = 0
    for idx in cands:
        try:
            if geoms[idx].intersects(geom):
                count += 1
        except Exception:
            continue
    return count


def _to_xy_geometry(geom_wgs, fwd) -> Optional[Any]:
    try:
        g = transform(fwd, geom_wgs)
    except Exception:
        return None
    if g.is_empty:
        return None
    return g


def _element_coords_wgs(el: Dict[str, Any]) -> List[Tuple[float, float]]:
    geom = el.get("geometry") or []
    out: List[Tuple[float, float]] = []
    for p in geom:
        if "lon" in p and "lat" in p:
            out.append((float(p["lon"]), float(p["lat"])))
    return out


def _geometry_from_overpass_element(el: Dict[str, Any]) -> Optional[Any]:
    etype = str(el.get("type", "")).lower()
    if etype == "node":
        lon = _first_float(el.get("lon"))
        lat = _first_float(el.get("lat"))
        if lon is None or lat is None:
            return None
        return Point(lon, lat)
    coords = _element_coords_wgs(el)
    if len(coords) < 2:
        return None
    if coords[0] == coords[-1] and len(coords) >= 4:
        poly = Polygon(coords)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if (not poly.is_empty) and poly.area > 0:
            return poly
    return LineString(coords)


def _no_fly_buffer_m(tags: Dict[str, Any], hard: bool) -> float:
    aeroway = str(tags.get("aeroway", "")).lower()
    if aeroway in {"aerodrome", "airport"}:
        return 2000.0
    military = str(tags.get("military", "")).strip().lower()
    if military in {"airfield", "air_base", "airbase", "naval_air_station"}:
        return 1600.0
    return 0.0


def _is_hard_no_fly(tags: Dict[str, Any], geom_xy: Any) -> bool:
    aeroway = str(tags.get("aeroway", "")).strip().lower()
    military = str(tags.get("military", "")).strip().lower()
    if aeroway in {"aerodrome", "airport"}:
        return True
    if military in {"airfield", "air_base", "airbase", "naval_air_station"}:
        return True
    return False


def fetch_open_data_no_fly_zones(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], List[Any], Dict[str, int]]:
    south, north, west, east = bbox_wgs
    query = f"""
[out:json][timeout:120];
(
  nwr({south},{west},{north},{east})[aeroway~"^(aerodrome|airport)$"];
  nwr({south},{west},{north},{east})[military~"^(airfield|air_base|airbase|naval_air_station)$"];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        return [], [], {"civil_airport": 0, "military_airport": 0, "hard": 0, "soft": 0}
    hard_polygons_xy: List[Any] = []
    soft_polygons_xy: List[Any] = []
    counter = {"civil_airport": 0, "military_airport": 0, "hard": 0, "soft": 0}
    max_zone_area_m2 = 180_000_000.0
    max_span_m = 80_000.0
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        g_wgs = _geometry_from_overpass_element(el)
        if g_wgs is None:
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        if str(tags.get("aeroway", "")).strip():
            counter["civil_airport"] += 1
        if str(tags.get("military", "")).strip():
            counter["military_airport"] += 1
        try:
            minx, miny, maxx, maxy = g_xy.bounds
            if (maxx - minx) > max_span_m or (maxy - miny) > max_span_m:
                continue
            is_hard = _is_hard_no_fly(tags, g_xy)
            buf_m = _no_fly_buffer_m(tags, hard=is_hard)
            if buf_m <= 0:
                continue
            if g_xy.geom_type in {"Polygon", "MultiPolygon"}:
                zone = g_xy.buffer(buf_m)
            else:
                zone = g_xy.buffer(max(220.0, buf_m))
            if zone.area > max_zone_area_m2:
                continue
            if is_hard:
                hard_polygons_xy.append(zone)
                counter["hard"] += 1
        except Exception:
            continue
    return hard_polygons_xy, soft_polygons_xy, counter


def fetch_school_kindergarten_zones(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], List[Any], Dict[str, int]]:
    south, north, west, east = bbox_wgs
    query = f"""
[out:json][timeout:120];
(
  nwr({south},{west},{north},{east})[amenity~"^(school|kindergarten)$"];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        return [], [], {"school": 0, "kindergarten": 0}
    hard_zones_xy: List[Any] = []
    points_xy: List[Any] = []
    counter = {"school": 0, "kindergarten": 0}
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        amenity = str(tags.get("amenity", "")).strip().lower()
        if amenity not in {"school", "kindergarten"}:
            continue
        g_wgs = _geometry_from_overpass_element(el)
        if g_wgs is None:
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        if amenity in counter:
            counter[amenity] += 1
        try:
            if g_xy.geom_type == "Point":
                zone = g_xy.buffer(SCHOOL_HARD_BUFFER_M)
                points_xy.append(g_xy)
            elif g_xy.geom_type in {"Polygon", "MultiPolygon"}:
                zone = g_xy.buffer(18.0)
                points_xy.append(g_xy.representative_point())
            else:
                zone = g_xy.buffer(26.0)
                points_xy.append(g_xy.representative_point())
            hard_zones_xy.append(zone)
        except Exception:
            continue
    return hard_zones_xy, points_xy, counter


def fetch_osm_buildings_and_obstacles(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], List[float], List[Any], List[float], Dict[str, int]]:
    south, north, west, east = bbox_wgs
    query = f"""
[out:json][timeout:150];
(
  way({south},{west},{north},{east})[building];
  relation({south},{west},{north},{east})[building];
  nwr({south},{west},{north},{east})[man_made~"^(tower|mast|chimney|communications_tower)$"];
  nwr({south},{west},{north},{east})[power~"^(tower|pole|line|substation|plant)$"];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=150)
    except Exception:
        return [], [], [], [], {"building": 0, "obstacle": 0}
    b_geoms: List[Any] = []
    b_heights: List[float] = []
    o_geoms: List[Any] = []
    o_heights: List[float] = []
    c_building = 0
    c_obstacle = 0
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        g_wgs = _geometry_from_overpass_element(el)
        if g_wgs is None:
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        is_building = str(tags.get("building", "")).strip() != ""
        if is_building:
            if g_xy.geom_type not in {"Polygon", "MultiPolygon"}:
                continue
            raw_h = _first_float(tags.get("height"))
            raw_lv = _first_float(tags.get("building:levels"))
            if raw_h is not None:
                h = raw_h
            elif raw_lv is not None:
                h = max(3.0, raw_lv * 3.2)
            else:
                btype = str(tags.get("building", "")).lower()
                if btype in {"industrial", "commercial"}:
                    h = 18.0
                elif btype in {"apartments", "residential"}:
                    h = 12.0
                else:
                    h = 10.0
            h = max(5.0, min(180.0, float(h)))
            b_geoms.append(g_xy)
            b_heights.append(h)
            c_building += 1
            continue
        raw_h = _first_float(tags.get("height"))
        if raw_h is not None:
            oh = raw_h
        else:
            man_made = str(tags.get("man_made", "")).lower()
            power = str(tags.get("power", "")).lower()
            if man_made == "chimney":
                oh = 60.0
            elif man_made in {"tower", "communications_tower"}:
                oh = 45.0
            elif man_made == "mast":
                oh = 35.0
            elif power == "tower":
                oh = 35.0
            elif power == "pole":
                oh = 20.0
            elif power in {"line", "substation", "plant"}:
                oh = 25.0
            else:
                oh = 18.0
        oh = max(8.0, min(200.0, float(oh)))
        if g_xy.geom_type == "Point":
            g2 = g_xy.buffer(15.0)
        elif g_xy.geom_type in {"Polygon", "MultiPolygon"}:
            g2 = g_xy
        else:
            g2 = g_xy.buffer(8.0)
        o_geoms.append(g2)
        o_heights.append(oh)
        c_obstacle += 1
    return b_geoms, b_heights, o_geoms, o_heights, {"building": c_building, "obstacle": c_obstacle}


def _is_usable_road_class(highway: str) -> bool:
    h = highway.strip().lower()
    if not h:
        return False
    blocked = {
        "motorway",
        "motorway_link",
        "trunk",
        "trunk_link",
        "footway",
        "path",
        "cycleway",
        "pedestrian",
        "steps",
        "raceway",
        "bridleway",
        "corridor",
        "construction",
        "proposed",
    }
    return h not in blocked


def _simplify_coords_wgs(coords: List[Tuple[float, float]], tol_deg: float) -> List[Tuple[float, float]]:
    if len(coords) < 3:
        return coords
    try:
        line = LineString(coords)
        sim = list(line.simplify(tol_deg, preserve_topology=False).coords)
        if len(sim) < 2:
            return coords
        sim[0] = coords[0]
        sim[-1] = coords[-1]
        return [(float(x), float(y)) for x, y in sim]
    except Exception:
        return coords


def load_city_networks(
    summary: Dict[str, Any],
    route_bbox: Tuple[float, float, float, float],
) -> List[Dict[str, Any]]:
    outputs = summary.get("outputs", {})
    roads_geojson = Path(outputs.get("transport", {}).get("roads_geojson", ""))
    roads_wgs: List[Dict[str, Any]] = []
    for feat in load_geojson_features(roads_geojson):
        geom = feat.get("geometry") or {}
        props = feat.get("properties") or {}
        hwy = str(props.get("highway", "")).lower().strip()
        if not _is_usable_road_class(hwy):
            continue
        name = str(props.get("name", "")).strip()
        if geom.get("type") == "LineString":
            coords = geom.get("coordinates") or []
            if len(coords) >= 2:
                coords2 = _simplify_coords_wgs([(float(x), float(y)) for x, y in coords], tol_deg=0.00008)
                roads_wgs.append(
                    {
                        "coords": coords2,
                        "network_type": "road",
                        "highway": hwy,
                        "name": name,
                    }
                )
        elif geom.get("type") == "MultiLineString":
            for group in geom.get("coordinates") or []:
                if len(group) >= 2:
                    coords2 = _simplify_coords_wgs([(float(x), float(y)) for x, y in group], tol_deg=0.00008)
                    roads_wgs.append(
                        {
                            "coords": coords2,
                            "network_type": "road",
                            "highway": hwy,
                            "name": name,
                        }
                    )
    south, north, west, east = route_bbox
    query = f"""
[out:json][timeout:120];
(
  way({south},{west},{north},{east})[highway][highway!~"^(motorway|trunk|motorway_link|trunk_link)$"];
  way({south},{west},{north},{east})[waterway~"^(river|canal|stream|ditch)$"];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        return roads_wgs
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        coords = _element_coords_wgs(el)
        if len(coords) < 2:
            continue
        waterway = str(tags.get("waterway", "")).strip().lower()
        ntype = "water" if waterway else "road"
        hwy = str(tags.get("highway", "")).strip().lower()
        if ntype == "road" and (not _is_usable_road_class(hwy)):
            continue
        tol = 0.00005 if ntype == "water" else 0.00008
        coords2 = _simplify_coords_wgs(coords, tol_deg=tol)
        roads_wgs.append(
            {
                "coords": coords2,
                "network_type": ntype,
                "highway": hwy,
                "name": str(tags.get("name", "")).strip(),
            }
        )
    return roads_wgs


def project_lines(
    lines_wgs: List[Dict[str, Any]],
    fwd,
) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for item in lines_wgs:
        coords = item.get("coords") or []
        pts_xy: List[Tuple[float, float]] = []
        for lon, lat in coords:
            try:
                x, y = transform(fwd, Point(float(lon), float(lat))).coords[0]
            except Exception:
                continue
            pts_xy.append((x, y))
        if len(pts_xy) < 2:
            continue
        out.append(
            {
                "coords": pts_xy,
                "network_type": str(item.get("network_type", "road")),
                "highway": str(item.get("highway", "")),
                "name": str(item.get("name", "")),
            }
        )
    return out


def build_landuse_index(
    summary: Dict[str, Any],
    fwd,
) -> Tuple[List[Any], List[float], Optional[STRtree]]:
    outputs = summary.get("outputs", {})
    landuse_geojson = Path(outputs.get("landuse", {}).get("geojson", ""))
    geoms: List[Any] = []
    costs: List[float] = []
    for feat in load_geojson_features(landuse_geojson):
        geom = feat.get("geometry")
        if not geom:
            continue
        props = feat.get("properties") or {}
        landuse_type = str(props.get("landuse_type", "")).strip().lower()
        c = _landuse_cost(landuse_type)
        try:
            g = shape(geom)
            if g.is_empty:
                continue
        except Exception:
            continue
        g_xy = _to_xy_geometry(g, fwd)
        if g_xy is None:
            continue
        if g_xy.geom_type not in {"Polygon", "MultiPolygon"}:
            continue
        geoms.append(g_xy)
        costs.append(c)
    tree = STRtree(geoms) if geoms else None
    return geoms, costs, tree


def load_low_risk_landuse_polygons(summary: Dict[str, Any], fwd) -> Tuple[List[Any], Dict[str, int]]:
    outputs = summary.get("outputs", {})
    landuse_geojson = Path(outputs.get("landuse", {}).get("geojson", ""))
    out: List[Any] = []
    counter = {"water": 0, "forest": 0, "green": 0}
    for feat in load_geojson_features(landuse_geojson):
        props = feat.get("properties") or {}
        token = str(props.get("landuse_type", "")).strip().lower()
        if not token:
            continue
        is_low_risk = False
        if token.startswith("natural:water") or token.startswith("waterway:"):
            counter["water"] += 1
            is_low_risk = True
        elif token.startswith("landuse:forest") or token.startswith("natural:wood"):
            counter["forest"] += 1
            is_low_risk = True
        elif token.startswith("leisure:park") or token.startswith("landuse:grass"):
            counter["green"] += 1
            is_low_risk = True
        if not is_low_risk:
            continue
        try:
            g = shape(feat.get("geometry") or {})
        except Exception:
            continue
        g_xy = _to_xy_geometry(g, fwd)
        if g_xy is None:
            continue
        if g_xy.geom_type in {"Polygon", "MultiPolygon"}:
            out.append(g_xy)
    return out, counter


def build_infrastructure_index(
    summary: Dict[str, Any],
    route_bbox: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], List[float], Optional[STRtree]]:
    geoms: List[Any] = []
    severities: List[float] = []
    outputs = summary.get("outputs", {})
    hsr_geojson = Path(outputs.get("transport", {}).get("hsr_geojson", ""))
    for feat in load_geojson_features(hsr_geojson):
        geom = feat.get("geometry") or {}
        try:
            g = shape(geom)
        except Exception:
            continue
        g_xy = _to_xy_geometry(g, fwd)
        if g_xy is None:
            continue
        geoms.append(g_xy.buffer(12.0))
        severities.append(2.5)

    south, north, west, east = route_bbox
    query = f"""
[out:json][timeout:120];
(
  nwr({south},{west},{north},{east})[power];
  nwr({south},{west},{north},{east})[man_made~"^(tower|mast|chimney|communications_tower)$"];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        data = {"elements": []}
    for el in data.get("elements", []):
        tags = el.get("tags") or {}
        g_wgs = _geometry_from_overpass_element(el)
        if g_wgs is None:
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        power = str(tags.get("power", "")).lower()
        man_made = str(tags.get("man_made", "")).lower()
        sev = 1.3
        if power in {"plant", "substation"}:
            sev = 2.2
        elif power in {"tower", "line"}:
            sev = 1.8
        elif man_made in {"tower", "communications_tower", "chimney"}:
            sev = 1.8
        if g_xy.geom_type == "Point":
            g2 = g_xy.buffer(20.0)
        elif g_xy.geom_type in {"Polygon", "MultiPolygon"}:
            g2 = g_xy
        else:
            g2 = g_xy.buffer(8.0)
        geoms.append(g2)
        severities.append(sev)
    tree = STRtree(geoms) if geoms else None
    return geoms, severities, tree


def _round_key(p: Tuple[float, float], snap_m: float = 3.0) -> Tuple[float, float]:
    return (round(p[0] / snap_m) * snap_m, round(p[1] / snap_m) * snap_m)


def landuse_cost_at_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    costs: List[float],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
) -> float:
    if tree is None:
        return 1.0
    point = Point(pt_xy)
    candidates = _tree_candidate_indices(tree, point, geoms, geom_id_map)
    best = 1.0
    for idx in candidates:
        g = geoms[idx]
        if not g.intersects(point):
            continue
        best = max(best, costs[idx])
    return best


def infrastructure_penalty_at_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    severities: List[float],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = Point(pt_xy)
    idx, dist = _nearest_index_and_distance(tree, point, geoms, geom_id_map)
    if idx is None or math.isinf(dist):
        return 0.0
    sev = severities[idx] if 0 <= idx < len(severities) else 1.0
    if dist <= 30.0:
        return 4.0 * sev
    if dist <= 80.0:
        return 2.0 * sev
    if dist <= 180.0:
        return 0.8 * sev
    if dist <= 300.0:
        return 0.3 * sev
    return 0.0


def building_height_at_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    heights: List[float],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
) -> float:
    if tree is None:
        return 0.0
    point = Point(pt_xy)
    candidates = _tree_candidate_indices(tree, point, geoms, geom_id_map)
    best = 0.0
    for idx in candidates:
        g = geoms[idx]
        if not g.intersects(point):
            continue
        best = max(best, heights[idx])
    return best


def obstacle_height_near_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    heights: List[float],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = Point(pt_xy)
    idx, dist = _nearest_index_and_distance(tree, point, geoms, geom_id_map)
    if idx is None or dist > 40.0:
        return 0.0
    if 0 <= idx < len(heights):
        return heights[idx]
    return 0.0


def point_risk_penalty_at_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
    inner_m: float = 60.0,
    buffer_m: float = ROUTE_BUFFER_M,
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = Point(pt_xy)
    idx, dist = _nearest_index_and_distance(tree, point, geoms, geom_id_map)
    if idx is None or math.isinf(dist):
        return 0.0
    if dist <= inner_m:
        return 2.5
    if dist <= buffer_m:
        return 1.5
    if dist <= buffer_m + 80.0:
        return 0.5
    return 0.0


def _vector_angle(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    n1 = math.hypot(a[0], a[1])
    n2 = math.hypot(b[0], b[1])
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cosv = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosv))


def _water_step_factor(edge_meta: Dict[str, float], water_pref_factor: float) -> float:
    factor = max(WATER_PREF_MIN_FACTOR, min(WATER_PREF_MAX_FACTOR, float(water_pref_factor)))
    pop_p90 = float(edge_meta.get("pop_p90", 0.0))
    # In dense areas, water corridors should get higher relative utility.
    dense_bonus = 1.0
    if pop_p90 >= 2200.0:
        dense_bonus = 0.68
    elif pop_p90 >= 1500.0:
        dense_bonus = 0.74
    elif pop_p90 >= 1000.0:
        dense_bonus = 0.82
    elif pop_p90 >= 700.0:
        dense_bonus = 0.9
    # Keep base search conservative; amplify dense-water reward mainly in water-priority runs.
    blend = (1.0 - factor) / max(1e-6, 1.0 - WATER_PREF_MIN_FACTOR)
    blend = max(0.0, min(1.0, blend))
    return factor * ((1.0 - blend) + blend * dense_bonus)


def _segment_overlap_ratio(seg: LineString, zone_union) -> float:
    if zone_union is None or zone_union.is_empty:
        return 0.0
    try:
        if not seg.intersects(zone_union):
            return 0.0
        overlap = seg.intersection(zone_union).length
        return max(0.0, min(1.0, overlap / max(1e-6, seg.length)))
    except Exception:
        return 0.0


def _percentile(values: List[float], q: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return float(values[0])
    xs = sorted(float(v) for v in values)
    pos = max(0.0, min(1.0, q)) * (len(xs) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs[lo]
    w = pos - lo
    return xs[lo] * (1.0 - w) + xs[hi] * w


def _sample_ratios_for_segment(length_m: float) -> List[float]:
    if length_m <= 120.0:
        return [0.2, 0.5, 0.8]
    if length_m <= 360.0:
        return [0.12, 0.3, 0.5, 0.7, 0.88]
    if length_m <= 1200.0:
        return [0.08, 0.2, 0.32, 0.44, 0.56, 0.68, 0.8, 0.92]
    return [0.05, 0.14, 0.23, 0.32, 0.41, 0.5, 0.59, 0.68, 0.77, 0.86, 0.95]


def _line_population_stats(seg_xy: LineString, pop_sampler: PopulationSampler, inv) -> Tuple[float, float, float]:
    if seg_xy.length < 1.0:
        x0, y0 = seg_xy.coords[0]
        lon0, lat0 = transform(inv, Point(x0, y0)).coords[0]
        v = pop_sampler.sample(lon0, lat0)
        return v, v, v
    vals: List[float] = []
    for r in _sample_ratios_for_segment(seg_xy.length):
        p = seg_xy.interpolate(seg_xy.length * r)
        lon, lat = transform(inv, p).coords[0]
        vals.append(pop_sampler.sample(lon, lat))
    if not vals:
        return 0.0, 0.0, 0.0
    return sum(vals) / len(vals), _percentile(vals, 0.9), max(vals)


def build_navigation_graph(
    lines_xy: List[Dict[str, Any]],
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    weights: Dict[str, float],
    no_fly_hard_union_xy,
    no_fly_soft_union_xy,
    infra_hard_union_xy,
    landuse_geoms: List[Any],
    landuse_costs: List[float],
    landuse_tree: Optional[STRtree],
    landuse_id_map: Dict[int, int],
    infra_geoms: List[Any],
    infra_severities: List[float],
    infra_tree: Optional[STRtree],
    infra_id_map: Dict[int, int],
    pop_sampler: PopulationSampler,
    inv,
    b_geoms: List[Any],
    b_heights: List[float],
    b_tree: Optional[STRtree],
    b_id_map: Dict[int, int],
    o_geoms: List[Any],
    o_heights: List[float],
    o_tree: Optional[STRtree],
    o_id_map: Dict[int, int],
    crowd_points_xy: List[Any],
    crowd_tree: Optional[STRtree],
    crowd_id_map: Dict[int, int],
    crowd_hard_union_xy,
    school_penalty_air: float,
    school_penalty_ground: float,
    key_points_xy: List[Any],
    key_tree: Optional[STRtree],
    key_id_map: Dict[int, int],
    line_risk_union_xy,
    high_building_union_xy,
    enable_water_endpoint_connectors: bool = True,
) -> Tuple[
    Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]],
    Dict[str, int],
]:
    graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]] = defaultdict(list)
    node_set: Dict[Tuple[float, float], Tuple[float, float]] = {}
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    nofly_soft = no_fly_soft_union_xy if no_fly_soft_union_xy is not None and (not no_fly_soft_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    crowd_hard = crowd_hard_union_xy if crowd_hard_union_xy is not None and (not crowd_hard_union_xy.is_empty) else None
    line_risk_union = line_risk_union_xy if line_risk_union_xy is not None and (not line_risk_union_xy.is_empty) else None
    high_building_union = (
        high_building_union_xy if high_building_union_xy is not None and (not high_building_union_xy.is_empty) else None
    )
    skipped_nofly = 0
    skipped_infra_hard = 0
    skipped_crowd_hard = 0
    kept_edges = 0
    air_edges = 0
    lattice_nodes: set[Tuple[float, float]] = set()
    water_nodes: set[Tuple[float, float]] = set()

    def canonical(p: Tuple[float, float]) -> Tuple[float, float]:
        k = _round_key(p, GRAPH_NODE_SNAP_M)
        if k not in node_set:
            node_set[k] = k
        return node_set[k]

    def edge_meta_for_segment(seg: LineString, ntype: str) -> Dict[str, float]:
        a = seg.coords[0]
        b = seg.coords[-1]
        dist = float(seg.length)
        ratios = _sample_ratios_for_segment(dist)
        pop_values: List[float] = []
        land_samples: List[float] = []
        infra_samples: List[float] = []
        height_samples: List[float] = []
        crowd_samples: List[float] = []
        key_samples: List[float] = []
        for ratio in ratios:
            sx = a[0] * (1.0 - ratio) + b[0] * ratio
            sy = a[1] * (1.0 - ratio) + b[1] * ratio
            lon, lat = transform(inv, Point(sx, sy)).coords[0]
            pop_values.append(pop_sampler.sample(lon, lat))
            land_samples.append(landuse_cost_at_point((sx, sy), landuse_geoms, landuse_costs, landuse_tree, landuse_id_map))
            infra_samples.append(
                infrastructure_penalty_at_point((sx, sy), infra_geoms, infra_severities, infra_tree, infra_id_map)
            )
            building_h = building_height_at_point((sx, sy), b_geoms, b_heights, b_tree, b_id_map)
            obstacle_h = obstacle_height_near_point((sx, sy), o_geoms, o_heights, o_tree, o_id_map)
            height_samples.append(max(building_h, obstacle_h) / 50.0)
            crowd_samples.append(
                point_risk_penalty_at_point((sx, sy), crowd_points_xy, crowd_tree, crowd_id_map, inner_m=50.0)
            )
            key_samples.append(
                point_risk_penalty_at_point((sx, sy), key_points_xy, key_tree, key_id_map, inner_m=70.0)
            )
        pop_avg = sum(pop_values) / max(1, len(pop_values))
        pop_p90 = _percentile(pop_values, 0.9)
        pop_peak = max(pop_values) if pop_values else 0.0
        pop_mix = 0.35 * pop_avg + 0.45 * pop_p90 + 0.2 * pop_peak
        if ntype == "air":
            pop_norm = min(5.0, pop_mix / 1000.0)
        elif ntype == "water":
            pop_norm = min(2.5, pop_mix / 2800.0)
        else:
            pop_norm = min(3.2, pop_mix / 1900.0)
        land_cost = sum(land_samples) / max(1, len(land_samples))
        infra_pen = 0.45 * (sum(infra_samples) / max(1, len(infra_samples))) + 0.55 * _percentile(infra_samples, 0.75)
        height_proxy = 0.35 * (sum(height_samples) / max(1, len(height_samples))) + 0.65 * (
            max(height_samples) if height_samples else 0.0
        )
        soft_overlap = _segment_overlap_ratio(seg, nofly_soft)
        line_overlap = _segment_overlap_ratio(seg, line_risk_union)
        high_build_overlap = _segment_overlap_ratio(seg, high_building_union)
        school_overlap = _segment_overlap_ratio(seg, crowd_hard)
        crowd_pen = 0.5 * (sum(crowd_samples) / max(1, len(crowd_samples))) + 0.5 * _percentile(crowd_samples, 0.8)
        key_pen = 0.45 * (sum(key_samples) / max(1, len(key_samples))) + 0.55 * _percentile(key_samples, 0.8)
        water_like_ratio = (
            sum(1 for c in land_samples if c <= 0.25) / max(1, len(land_samples))
            if land_samples
            else 0.0
        )
        low_risk_land_ratio = (
            sum(1 for c in land_samples if c <= 0.6) / max(1, len(land_samples))
            if land_samples
            else 0.0
        )
        if ntype == "water":
            ground_mult = 0.52
            network_mult = 0.8
        elif ntype == "road":
            ground_mult = 1.14
            network_mult = 1.15
        else:
            ground_mult = 0.72
            network_mult = 0.92
        context_mult = 1.0
        if ntype == "air":
            if pop_p90 <= 120.0:
                context_mult *= 0.58
            elif pop_p90 <= 260.0:
                context_mult *= 0.72
            elif pop_p90 <= 520.0:
                context_mult *= 0.88
            if low_risk_land_ratio >= 0.5:
                context_mult *= 0.93
            if water_like_ratio >= 0.18:
                context_mult *= 0.72
        elif ntype == "road" and pop_p90 >= 1800.0:
            context_mult *= 1.18
        per_m = (
            weights["length"] * 1.0
            + weights["population"] * pop_norm
            + weights["landuse"] * land_cost * ground_mult
            + weights["infrastructure"] * infra_pen
            + weights["altitude"] * height_proxy
            + weights["soft_no_fly"] * soft_overlap
            + weights["crowd"] * crowd_pen
            + weights["key_facility"] * key_pen
            + weights["line_cross"] * line_overlap
            + weights["high_building"] * high_build_overlap
            + (school_penalty_air if ntype == "air" else school_penalty_ground) * school_overlap
        ) * network_mult * context_mult
        base_cost = dist * max(0.01, per_m)
        return {
            "dist": dist,
            "base": base_cost,
            "pop": pop_norm,
            "land": land_cost,
            "infra": infra_pen,
            "height_proxy": height_proxy,
            "soft_no_fly": soft_overlap,
            "crowd_pen": crowd_pen,
            "key_pen": key_pen,
            "line_overlap": line_overlap,
            "high_building_overlap": high_build_overlap,
            "school_overlap": school_overlap,
            "pop_p90": pop_p90,
            "water_like_ratio": water_like_ratio,
            "air_edge": 1.0 if ntype == "air" else 0.0,
            "water_edge": 1.0 if ntype == "water" else 0.0,
            "network_type": ntype,
        }

    def try_add_edge(a: Tuple[float, float], b: Tuple[float, float], ntype: str) -> bool:
        nonlocal kept_edges, skipped_nofly, skipped_infra_hard, skipped_crowd_hard, air_edges
        if a == b:
            return False
        seg = LineString([a, b])
        if seg.length < 1.0:
            return False
        if nofly_hard is not None and seg.intersects(nofly_hard):
            skipped_nofly += 1
            return False
        if infra_hard is not None and seg.intersects(infra_hard):
            skipped_infra_hard += 1
            return False
        if crowd_hard is not None and seg.intersects(crowd_hard):
            skipped_crowd_hard += 1
            return False
        meta = edge_meta_for_segment(seg, ntype=ntype)
        vector_ab = (b[0] - a[0], b[1] - a[1])
        vector_ba = (a[0] - b[0], a[1] - b[1])
        graph[a].append((b, meta, vector_ab))
        graph[b].append((a, meta, vector_ba))
        kept_edges += 1
        if ntype == "air":
            air_edges += 1
        return True

    for item in lines_xy:
        coords = item.get("coords") or []
        ntype = str(item.get("network_type", "road")).lower()
        if len(coords) < 2:
            continue
        for i in range(len(coords) - 1):
            a = canonical(coords[i])
            b = canonical(coords[i + 1])
            try_add_edge(a, b, ntype=ntype)
            if ntype == "water":
                water_nodes.add(a)
                water_nodes.add(b)

    start_node = _round_key(start_xy, GRAPH_NODE_SNAP_M)
    end_node = _round_key(end_xy, GRAPH_NODE_SNAP_M)
    graph[start_node] = graph.get(start_node, [])
    graph[end_node] = graph.get(end_node, [])

    # Add oblique air lattice so low-risk straight crossings are available by default.
    sx, sy = start_node
    ex, ey = end_node
    dx = ex - sx
    dy = ey - sy
    dist_se = math.hypot(dx, dy)
    if dist_se > 10.0:
        ux = dx / dist_se
        uy = dy / dist_se
        px = -uy
        py = ux
        offs = [abs((n[0] - sx) * px + (n[1] - sy) * py) for n in graph.keys() if n not in {start_node, end_node}]
        max_off = min(AIR_LATTICE_MAX_OFFSET_M, max(1200.0, (max(offs) if offs else 0.0) + 500.0))
        offset_vals: List[float] = []
        o = -max_off
        while o <= max_off + 1.0:
            offset_vals.append(o)
            o += AIR_LATTICE_STEP_M
        t_vals: List[float] = []
        t = -AIR_LATTICE_MARGIN_M
        while t <= dist_se + AIR_LATTICE_MARGIN_M + 1.0:
            t_vals.append(t)
            t += AIR_LATTICE_STEP_M
        rows: List[Dict[int, Tuple[float, float]]] = []
        for tv in t_vals:
            row: Dict[int, Tuple[float, float]] = {}
            bx = sx + ux * tv
            by = sy + uy * tv
            for idx, off in enumerate(offset_vals):
                p = _round_key((bx + px * off, by + py * off), 6.0)
                if nofly_hard is not None and Point(p).intersects(nofly_hard):
                    continue
                graph[p] = graph.get(p, [])
                lattice_nodes.add(p)
                row[idx] = p
            rows.append(row)
        for ridx, row in enumerate(rows):
            for cidx in range(1, len(offset_vals)):
                a = row.get(cidx - 1)
                b = row.get(cidx)
                if a is not None and b is not None:
                    try_add_edge(a, b, ntype="air")
            if ridx == 0:
                continue
            prev = rows[ridx - 1]
            for cidx in range(len(offset_vals)):
                a = prev.get(cidx)
                b = row.get(cidx)
                if a is not None and b is not None:
                    try_add_edge(a, b, ntype="air")
            for cidx in range(1, len(offset_vals)):
                a = prev.get(cidx - 1)
                b = row.get(cidx)
                if a is not None and b is not None:
                    try_add_edge(a, b, ntype="air")
                a2 = prev.get(cidx)
                b2 = row.get(cidx - 1)
                if a2 is not None and b2 is not None:
                    try_add_edge(a2, b2, ntype="air")

    all_nodes = [n for n in graph.keys() if n not in {start_node, end_node}]
    if not all_nodes:
        return graph, {
            "kept_edges": kept_edges,
            "air_edges": air_edges,
            "skipped_nofly": skipped_nofly,
            "skipped_infra_hard": skipped_infra_hard,
            "skipped_crowd_hard": skipped_crowd_hard,
        }

    connector_levels = [220.0, 450.0, 800.0, 1300.0, 1800.0]
    for anchor, tag in [(start_node, "start"), (end_node, "end")]:
        linked = 0
        linked_air = 0
        for radius in connector_levels:
            dists_air: List[Tuple[float, Tuple[float, float]]] = []
            for n in lattice_nodes:
                d = math.hypot(n[0] - anchor[0], n[1] - anchor[1])
                if d <= radius:
                    dists_air.append((d, n))
            dists_air.sort(key=lambda x: x[0])
            for _, n in dists_air:
                if try_add_edge(anchor, n, ntype="air"):
                    linked += 1
                    linked_air += 1
                    if linked_air >= 8:
                        break
            if linked_air >= 3:
                break
        for radius in connector_levels:
            dists: List[Tuple[float, Tuple[float, float]]] = []
            for n in all_nodes:
                d = math.hypot(n[0] - anchor[0], n[1] - anchor[1])
                if d <= radius:
                    dists.append((d, n))
            dists.sort(key=lambda x: x[0])
            for _, n in dists:
                if try_add_edge(anchor, n, ntype="air"):
                    linked += 1
                    if linked >= 18:
                        break
            if linked > 0:
                break
        # Ensure each endpoint has at least a few candidate entries to the water network.
        if enable_water_endpoint_connectors and water_nodes:
            linked_water = 0
            for radius in [320.0, 550.0, 900.0, 1400.0]:
                dists_w: List[Tuple[float, Tuple[float, float]]] = []
                for n in water_nodes:
                    d = math.hypot(n[0] - anchor[0], n[1] - anchor[1])
                    if d <= radius:
                        dists_w.append((d, n))
                dists_w.sort(key=lambda x: x[0])
                for _, n in dists_w:
                    if try_add_edge(anchor, n, ntype="air"):
                        linked += 1
                        linked_water += 1
                        if linked_water >= 3:
                            break
                if linked_water >= 1:
                    break
        if linked == 0:
            raise RuntimeError(f"Could not connect {tag} point to graph within {int(connector_levels[-1])}m.")

    return graph, {
        "kept_edges": kept_edges,
        "air_edges": air_edges,
        "skipped_nofly": skipped_nofly,
        "skipped_infra_hard": skipped_infra_hard,
        "skipped_crowd_hard": skipped_crowd_hard,
    }


def astar_with_turn_penalty(
    graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    turn_weight: float,
    turn_radius_m: float,
    water_pref_factor: float = 1.0,
    max_turn_deflection_deg: Optional[float] = None,
) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
    if start not in graph or goal not in graph:
        return None, {}
    min_per_m = 1.0
    for neighbors in graph.values():
        for _, meta, _ in neighbors:
            dist = max(1e-6, meta.get("dist", 1.0))
            min_per_m = min(min_per_m, meta.get("base", dist) / dist)
    start_state = (None, start)
    pq: List[Tuple[float, float, Tuple[Optional[Tuple[float, float]], Tuple[float, float]]]] = []
    heapq.heappush(pq, (0.0, 0.0, start_state))
    g_score: Dict[Tuple[Optional[Tuple[float, float]], Tuple[float, float]], float] = {start_state: 0.0}
    parent: Dict[
        Tuple[Optional[Tuple[float, float]], Tuple[float, float]],
        Tuple[Optional[Tuple[float, float]], Tuple[float, float]],
    ] = {}
    best_goal_state = None
    visited = set()
    comp = {
        "distance_m": 0.0,
        "turn_penalty": 0.0,
        "edge_cost": 0.0,
        "water_distance_m": 0.0,
        "road_distance_m": 0.0,
        "air_distance_m": 0.0,
        "turn_count": 0,
    }

    while pq:
        _, g_cur, state = heapq.heappop(pq)
        if state in visited:
            continue
        visited.add(state)
        prev_node, cur_node = state
        if cur_node == goal:
            best_goal_state = state
            break
        for next_node, edge_meta, vec in graph.get(cur_node, []):
            turn_penalty = 0.0
            if prev_node is not None:
                pv = (cur_node[0] - prev_node[0], cur_node[1] - prev_node[1])
                ang = _vector_angle(pv, vec)
                if max_turn_deflection_deg is not None and ang > max_turn_deflection_deg:
                    continue
                if ang > TURN_IGNORE_DEG:
                    turn_severity = (ang - TURN_IGNORE_DEG) / max(1e-6, 180.0 - TURN_IGNORE_DEG)
                    turn_severity = max(0.0, min(1.0, turn_severity)) ** 1.7
                    bend_radius_proxy = edge_meta["dist"] / max(0.1, math.radians(max(5.0, ang)))
                    radius_pen = max(0.0, (turn_radius_m - bend_radius_proxy) / max(1.0, turn_radius_m))
                    ntype = edge_meta.get("network_type", "road")
                    road_turn_boost = 1.75 if ntype == "road" else 1.0
                    sharp_bonus = 0.85 if ang >= TURN_SHARP_DEG else 0.0
                    fixed_pen = turn_weight * TURN_FIXED_SCALE * (1.2 if ntype == "road" else 0.65)
                    turn_penalty = fixed_pen + turn_weight * road_turn_boost * (turn_severity + 0.65 * radius_pen + sharp_bonus)
            base_step = edge_meta.get("base", edge_meta.get("dist", 1.0))
            if edge_meta.get("network_type") == "water":
                base_step *= _water_step_factor(edge_meta, water_pref_factor)
            step_cost = base_step + turn_penalty
            next_state = (cur_node, next_node)
            ng = g_cur + step_cost
            if ng < g_score.get(next_state, float("inf")):
                g_score[next_state] = ng
                parent[next_state] = state
                h = min_per_m * math.hypot(next_node[0] - goal[0], next_node[1] - goal[1])
                heapq.heappush(pq, (ng + h, ng, next_state))

    if best_goal_state is None:
        return None, {}

    rev_nodes = [best_goal_state[1]]
    st = best_goal_state
    while st in parent:
        st = parent[st]
        rev_nodes.append(st[1])
    rev_nodes.reverse()

    total_dist = 0.0
    total_edge = 0.0
    total_turn = 0.0
    turn_count = 0
    for i in range(1, len(rev_nodes)):
        a = rev_nodes[i - 1]
        b = rev_nodes[i]
        for nb, meta, vec in graph.get(a, []):
            if nb != b:
                continue
            total_dist += meta.get("dist", 0.0)
            edge_base = meta.get("base", 0.0)
            ntype = meta.get("network_type", "road")
            if ntype == "water":
                edge_base *= _water_step_factor(meta, water_pref_factor)
                comp["water_distance_m"] += meta.get("dist", 0.0)
            elif ntype == "air":
                comp["air_distance_m"] += meta.get("dist", 0.0)
            else:
                comp["road_distance_m"] += meta.get("dist", 0.0)
            total_edge += edge_base
            if i >= 2:
                pv = (a[0] - rev_nodes[i - 2][0], a[1] - rev_nodes[i - 2][1])
                ang = _vector_angle(pv, vec)
                if ang > TURN_IGNORE_DEG:
                    turn_count += 1
                    turn_severity = (ang - TURN_IGNORE_DEG) / max(1e-6, 180.0 - TURN_IGNORE_DEG)
                    turn_severity = max(0.0, min(1.0, turn_severity)) ** 1.7
                    bend_radius_proxy = meta["dist"] / max(0.1, math.radians(max(5.0, ang)))
                    radius_pen = max(0.0, (turn_radius_m - bend_radius_proxy) / max(1.0, turn_radius_m))
                    road_turn_boost = 1.75 if ntype == "road" else 1.0
                    sharp_bonus = 0.85 if ang >= TURN_SHARP_DEG else 0.0
                    fixed_pen = turn_weight * TURN_FIXED_SCALE * (1.2 if ntype == "road" else 0.65)
                    total_turn += fixed_pen + turn_weight * road_turn_boost * (turn_severity + 0.65 * radius_pen + sharp_bonus)
            break
    comp["distance_m"] = round(total_dist, 2)
    comp["edge_cost"] = round(total_edge, 2)
    comp["turn_penalty"] = round(total_turn, 2)
    comp["turn_count"] = int(turn_count)
    comp["water_distance_m"] = round(comp["water_distance_m"], 2)
    comp["road_distance_m"] = round(comp["road_distance_m"], 2)
    comp["air_distance_m"] = round(comp["air_distance_m"], 2)
    comp["total_cost"] = round(total_edge + total_turn, 2)
    return rev_nodes, comp


def simplify_polyline(points: List[Tuple[float, float]], tol_m: float = 8.0) -> List[Tuple[float, float]]:
    if len(points) < 3:
        return points
    line = LineString(points)
    sim = list(line.simplify(tol_m, preserve_topology=False).coords)
    if len(sim) < 2:
        return points
    sim[0] = points[0]
    sim[-1] = points[-1]
    return [(float(x), float(y)) for x, y in sim]


def polyline_turn_count(points: List[Tuple[float, float]], angle_threshold_deg: float = TURN_IGNORE_DEG) -> int:
    if len(points) < 3:
        return 0
    c = 0
    for i in range(1, len(points) - 1):
        v1 = (points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        v2 = (points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        ang = _vector_angle(v1, v2)
        if ang > angle_threshold_deg:
            c += 1
    return c


def polyline_min_interior_angle(points: List[Tuple[float, float]]) -> float:
    if len(points) < 3:
        return 180.0
    best = 180.0
    for i in range(1, len(points) - 1):
        v1 = (points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        v2 = (points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
        deflection = _vector_angle(v1, v2)
        interior = 180.0 - deflection
        if interior < best:
            best = interior
    return max(0.0, min(180.0, best))


def _is_shortcut_safe(
    direct: LineString,
    original: LineString,
    pop_sampler: PopulationSampler,
    inv,
    nofly_hard,
    infra_hard,
    crowd_hard,
    max_len_ratio: float = SHORTCUT_MAX_LEN_RATIO,
    p90_ratio: float = 1.05,
    p90_add: float = 30.0,
    avg_ratio: float = 1.08,
    avg_add: float = 30.0,
) -> bool:
    if direct.length <= 0.0:
        return False
    if direct.length > original.length * max(1.0, max_len_ratio):
        return False
    if nofly_hard is not None and direct.intersects(nofly_hard):
        return False
    if infra_hard is not None and direct.intersects(infra_hard):
        return False
    if crowd_hard is not None and direct.intersects(crowd_hard):
        return False
    d_avg, d_p90, _ = _line_population_stats(direct, pop_sampler, inv)
    o_avg, o_p90, _ = _line_population_stats(original, pop_sampler, inv)
    if d_p90 > o_p90 * p90_ratio + p90_add:
        return False
    if d_avg > o_avg * avg_ratio + avg_add:
        return False
    return True


def shortcut_polyline(
    points: List[Tuple[float, float]],
    pop_sampler: PopulationSampler,
    inv,
    no_fly_hard_union_xy,
    infra_hard_union_xy,
    crowd_hard_union_xy=None,
    passes: int = 3,
    max_hop: int = 20,
    max_jump_m: float = 2000.0,
) -> List[Tuple[float, float]]:
    if len(points) < 4:
        return points
    pts = points[:]
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    crowd_hard = crowd_hard_union_xy if crowd_hard_union_xy is not None and (not crowd_hard_union_xy.is_empty) else None
    for _ in range(max(1, passes)):
        changed = False
        out: List[Tuple[float, float]] = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            best = i + 1
            upper = min(len(pts) - 1, i + max_hop)
            for j in range(upper, i + 1, -1):
                if j <= i + 1:
                    continue
                direct = LineString([pts[i], pts[j]])
                if direct.length > max_jump_m:
                    continue
                original = LineString(pts[i : j + 1])
                if not _is_shortcut_safe(
                    direct=direct,
                    original=original,
                    pop_sampler=pop_sampler,
                    inv=inv,
                    nofly_hard=nofly_hard,
                    infra_hard=infra_hard,
                    crowd_hard=crowd_hard,
                    max_len_ratio=SHORTCUT_MAX_LEN_RATIO,
                    p90_ratio=1.05,
                    p90_add=30.0,
                    avg_ratio=1.08,
                    avg_add=30.0,
                ):
                    continue
                best = j
                break
            if best > i + 1:
                changed = True
            out.append(pts[best])
            i = best
        pts = out
        if not changed or len(pts) < 4:
            break
    return pts


def prune_low_value_turns(
    points: List[Tuple[float, float]],
    pop_sampler: PopulationSampler,
    inv,
    no_fly_hard_union_xy,
    infra_hard_union_xy,
    crowd_hard_union_xy=None,
    min_turn_keep_deg: float = LOW_VALUE_TURN_KEEP_DEG,
    passes: int = 3,
) -> List[Tuple[float, float]]:
    if len(points) < 4:
        return points
    pts = points[:]
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    crowd_hard = crowd_hard_union_xy if crowd_hard_union_xy is not None and (not crowd_hard_union_xy.is_empty) else None
    for _ in range(max(1, passes)):
        changed = False
        out: List[Tuple[float, float]] = [pts[0]]
        i = 1
        while i < len(pts) - 1:
            prev = out[-1]
            cur = pts[i]
            nxt = pts[i + 1]
            v1 = (cur[0] - prev[0], cur[1] - prev[1])
            v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
            ang = _vector_angle(v1, v2)
            if ang >= min_turn_keep_deg:
                out.append(cur)
                i += 1
                continue
            direct = LineString([prev, nxt])
            original = LineString([prev, cur, nxt])
            if _is_shortcut_safe(
                direct=direct,
                original=original,
                pop_sampler=pop_sampler,
                inv=inv,
                nofly_hard=nofly_hard,
                infra_hard=infra_hard,
                crowd_hard=crowd_hard,
                max_len_ratio=1.012,
                p90_ratio=1.10,
                p90_add=45.0,
                avg_ratio=1.12,
                avg_add=45.0,
            ):
                changed = True
                i += 1
                continue
            out.append(cur)
            i += 1
        out.append(pts[-1])
        pts = out
        if not changed or len(pts) < 4:
            break
    return pts


def enforce_min_turn_angle(
    points: List[Tuple[float, float]],
    min_turn_angle_deg: float,
    pop_sampler: PopulationSampler,
    inv,
    no_fly_hard_union_xy,
    infra_hard_union_xy,
    crowd_hard_union_xy=None,
    passes: int = 4,
) -> List[Tuple[float, float]]:
    if len(points) < 4:
        return points
    max_deflection = max(0.0, 180.0 - float(min_turn_angle_deg))
    pts = points[:]
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    crowd_hard = crowd_hard_union_xy if crowd_hard_union_xy is not None and (not crowd_hard_union_xy.is_empty) else None
    for _ in range(max(1, passes)):
        changed = False
        out: List[Tuple[float, float]] = [pts[0]]
        i = 1
        while i < len(pts) - 1:
            prev = out[-1]
            cur = pts[i]
            nxt = pts[i + 1]
            v1 = (cur[0] - prev[0], cur[1] - prev[1])
            v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
            deflection = _vector_angle(v1, v2)
            if deflection <= max_deflection + 1e-6:
                out.append(cur)
                i += 1
                continue
            direct = LineString([prev, nxt])
            original = LineString([prev, cur, nxt])
            if _is_shortcut_safe(
                direct=direct,
                original=original,
                pop_sampler=pop_sampler,
                inv=inv,
                nofly_hard=nofly_hard,
                infra_hard=infra_hard,
                crowd_hard=crowd_hard,
                max_len_ratio=1.02,
                p90_ratio=1.10,
                p90_add=45.0,
                avg_ratio=1.12,
                avg_add=45.0,
            ):
                changed = True
                i += 1
                continue
            out.append(cur)
            i += 1
        out.append(pts[-1])
        pts = out
        if not changed or len(pts) < 4:
            break
    return pts


def enforce_waypoint_budget(
    points: List[Tuple[float, float]],
    max_waypoints: int,
    pop_sampler: PopulationSampler,
    inv,
    no_fly_hard_union_xy,
    infra_hard_union_xy,
    crowd_hard_union_xy=None,
) -> List[Tuple[float, float]]:
    if max_waypoints <= 2 or len(points) <= max_waypoints:
        return points
    pts = points[:]
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    crowd_hard = crowd_hard_union_xy if crowd_hard_union_xy is not None and (not crowd_hard_union_xy.is_empty) else None
    while len(pts) > max_waypoints and len(pts) > 2:
        best_idx = -1
        best_score = float("inf")
        for i in range(1, len(pts) - 1):
            prev = pts[i - 1]
            cur = pts[i]
            nxt = pts[i + 1]
            direct = LineString([prev, nxt])
            original = LineString([prev, cur, nxt])
            if not _is_shortcut_safe(
                direct=direct,
                original=original,
                pop_sampler=pop_sampler,
                inv=inv,
                nofly_hard=nofly_hard,
                infra_hard=infra_hard,
                crowd_hard=crowd_hard,
                max_len_ratio=1.018,
                p90_ratio=1.12,
                p90_add=55.0,
                avg_ratio=1.14,
                avg_add=55.0,
            ):
                continue
            v1 = (cur[0] - prev[0], cur[1] - prev[1])
            v2 = (nxt[0] - cur[0], nxt[1] - cur[1])
            ang = _vector_angle(v1, v2)
            dist_gain = max(0.0, original.length - direct.length)
            score = (ang * 2.0) + dist_gain * 0.06
            if score < best_score:
                best_score = score
                best_idx = i
        if best_idx <= 0:
            break
        del pts[best_idx]
    return pts


def sample_polyline_by_spacing(points_xy: List[Tuple[float, float]], spacing_m: float = ALT_PROFILE_SPACING_M) -> List[Tuple[float, float]]:
    if len(points_xy) < 2:
        return points_xy
    line = LineString(points_xy)
    if line.length <= spacing_m:
        return points_xy
    n = max(2, int(line.length / spacing_m))
    out = [line.interpolate(line.length * i / n).coords[0] for i in range(n + 1)]
    return [(float(x), float(y)) for x, y in out]


def sample_polyline_with_vertices(
    points_xy: List[Tuple[float, float]],
    spacing_m: float = ALT_PROFILE_SPACING_M,
) -> Tuple[List[Tuple[float, float]], List[int], List[float]]:
    if len(points_xy) < 2:
        return points_xy[:], [0] if points_xy else [], [0.0] if points_xy else []
    spacing = max(5.0, float(spacing_m))
    samples: List[Tuple[float, float]] = [points_xy[0]]
    vertex_indices: List[int] = [0]
    cum_dist_m: List[float] = [0.0]
    for i in range(1, len(points_xy)):
        ax, ay = points_xy[i - 1]
        bx, by = points_xy[i]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len <= 1e-6:
            if samples[-1] != points_xy[i]:
                samples.append(points_xy[i])
                cum_dist_m.append(cum_dist_m[-1])
            vertex_indices.append(len(samples) - 1)
            continue
        steps = max(1, int(math.ceil(seg_len / spacing)))
        for s in range(1, steps + 1):
            t = s / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            px, py = samples[-1]
            ds = math.hypot(x - px, y - py)
            if ds <= 1e-7:
                continue
            samples.append((float(x), float(y)))
            cum_dist_m.append(cum_dist_m[-1] + ds)
        vertex_indices.append(len(samples) - 1)
    if vertex_indices[-1] != len(samples) - 1:
        vertex_indices[-1] = len(samples) - 1
    return samples, vertex_indices, cum_dist_m


def _can_link_profile_samples(
    i: int,
    j: int,
    z: List[float],
    z_min: List[float],
    z_hard_cap: List[float],
    cum_dist_m: List[float],
    climb_ratio: float,
    descend_ratio: float,
    err_tol_m: float,
) -> bool:
    if j <= i + 1:
        return True
    ds = max(1e-6, float(cum_dist_m[j] - cum_dist_m[i]))
    dz = float(z[j] - z[i])
    if dz > climb_ratio * ds + 1e-6:
        return False
    if -dz > descend_ratio * ds + 1e-6:
        return False
    for k in range(i + 1, j):
        r = (cum_dist_m[k] - cum_dist_m[i]) / ds
        z_interp = z[i] + dz * r
        if z_interp + 1e-6 < z_min[k]:
            return False
        if z_interp > z_hard_cap[k] + 1e-6:
            return False
        if abs(z_interp - z[k]) > err_tol_m:
            return False
    return True


def compress_altitude_waypoints(
    samples_xy: List[Tuple[float, float]],
    samples_wgs: List[Tuple[float, float]],
    z: List[float],
    z_min: List[float],
    z_hard_cap: List[float],
    cum_dist_m: List[float],
    mandatory_indices: List[int],
    climb_ms: float,
    descend_ms: float,
    speed_ms: float,
    err_tol_m: float = ALT_PROFILE_LINK_ERR_TOL_M,
) -> List[Tuple[float, float, float]]:
    n = len(samples_xy)
    if n < 2:
        return [(samples_wgs[0][0], samples_wgs[0][1], max(0.0, z[0]))] if n == 1 else []
    mandatory = sorted({max(0, min(n - 1, int(i))) for i in mandatory_indices} | {0, n - 1})
    climb_ratio = max(0.05, float(climb_ms) / max(0.5, float(speed_ms)))
    descend_ratio = max(0.05, float(descend_ms) / max(0.5, float(speed_ms)))
    keep_idx: List[int] = [0]
    cur = 0
    while cur < n - 1:
        next_mandatory = n - 1
        for idx in mandatory:
            if idx > cur:
                next_mandatory = idx
                break
        chosen = cur + 1
        for j in range(next_mandatory, cur, -1):
            if _can_link_profile_samples(
                cur,
                j,
                z=z,
                z_min=z_min,
                z_hard_cap=z_hard_cap,
                cum_dist_m=cum_dist_m,
                climb_ratio=climb_ratio,
                descend_ratio=descend_ratio,
                err_tol_m=max(0.6, float(err_tol_m)),
            ):
                chosen = j
                break
        keep_idx.append(chosen)
        cur = chosen
    out_wgs_alt: List[Tuple[float, float, float]] = []
    dedup = set()
    for i in keep_idx:
        lon, lat = samples_wgs[i]
        key = (round(lon, 8), round(lat, 8), round(float(z[i]), 3))
        if key in dedup:
            continue
        dedup.add(key)
        out_wgs_alt.append((float(lon), float(lat), max(0.0, float(z[i]))))
    if len(out_wgs_alt) >= 2:
        out_wgs_alt[0] = (samples_wgs[0][0], samples_wgs[0][1], max(0.0, z[0]))
        out_wgs_alt[-1] = (samples_wgs[-1][0], samples_wgs[-1][1], max(0.0, z[-1]))
    return out_wgs_alt


def plan_altitude_profile(
    path_xy: List[Tuple[float, float]],
    inv,
    terrain_sampler: TerrainSampler,
    b_geoms: List[Any],
    b_heights: List[float],
    b_tree: Optional[STRtree],
    b_id_map: Dict[int, int],
    o_geoms: List[Any],
    o_heights: List[float],
    o_tree: Optional[STRtree],
    o_id_map: Dict[int, int],
    speed_ms: float,
    climb_ms: float,
    descend_ms: float,
    clearance_m: float,
    preferred_cruise_max_m: float,
    hard_ceiling_m: float,
    min_true_height_m: float,
    max_true_height_m: float,
    endpoint_true_height_m: float,
) -> Tuple[List[Tuple[float, float, float]], Dict[str, Any], List[Dict[str, float]]]:
    samples_xy, vertex_indices, cum_dist_m = sample_polyline_with_vertices(path_xy, spacing_m=ALT_PROFILE_SPACING_M)
    samples_wgs: List[Tuple[float, float]] = []
    for x, y in samples_xy:
        lon, lat = transform(inv, Point(x, y)).coords[0]
        samples_wgs.append((lon, lat))
    terrain_vals, terrain_source = terrain_sampler.sample_points(samples_wgs)
    if len(terrain_vals) < len(samples_xy):
        terrain_vals.extend([0.0] * (len(samples_xy) - len(terrain_vals)))
    terrain_msl_vals: List[float] = []
    z_min: List[float] = []
    top_surface_vals: List[float] = []
    agl_target: List[float] = []
    building_vals: List[float] = []
    obstacle_vals: List[float] = []
    building_hits = 0
    obstacle_hits = 0
    for i, p in enumerate(samples_xy):
        terrain = max(0.0, float(terrain_vals[i]))
        terrain_msl_vals.append(float(terrain))
        bh = building_height_at_point(p, b_geoms, b_heights, b_tree, b_id_map)
        oh = obstacle_height_near_point(p, o_geoms, o_heights, o_tree, o_id_map)
        building_vals.append(float(bh))
        obstacle_vals.append(float(oh))
        if bh > 0:
            building_hits += 1
        if oh > 0:
            obstacle_hits += 1
        top_local = terrain + max(bh, oh)
        need = top_local + clearance_m
        z_min.append(need)
        top_surface_vals.append(top_local)
        agl_target.append(min(float(max_true_height_m), float(preferred_cruise_max_m)))

    n = len(samples_xy)
    min_true_h = max(float(clearance_m), float(min_true_height_m))
    max_true_h = max(min_true_h, float(max_true_height_m))
    endpoint_true_h = max(min_true_h, min(max_true_h, float(endpoint_true_height_m)))

    if n < 2:
        lon, lat = samples_wgs[0]
        lower0 = max(float(terrain_msl_vals[0]) + float(min_true_h), float(z_min[0]))
        upper0 = max(lower0, float(terrain_msl_vals[0]) + float(hard_ceiling_m))
        endpoint_target0 = float(terrain_msl_vals[0]) + endpoint_true_h
        z0 = min(upper0, max(lower0, endpoint_target0))
        clear0 = max(0.0, float(z0) - float(top_surface_vals[0]))
        true0 = max(0.0, float(z0) - float(terrain_msl_vals[0]))
        out_one = [(lon, lat, max(0.0, z0))]
        one_sample = [
            {
                "distance_m": 0.0,
                "route_alt_msl_m": round(max(0.0, z0), 2),
                "terrain_msl_m": round(float(terrain_msl_vals[0]), 2),
                "surface_msl_m": round(float(top_surface_vals[0]), 2),
                "building_h_m": round(float(building_vals[0]), 2),
                "obstacle_h_m": round(float(obstacle_vals[0]), 2),
                "surface_clearance_m": round(clear0, 2),
                "true_height_m": round(true0, 2),
            }
        ]
        meta_one = {
            "terrain_source": terrain_source,
            "altitude_points": 1,
            "altitude_points_raw": 1,
            "min_msl_m": round(float(z0), 2),
            "max_msl_m": round(float(z0), 2),
            "mean_msl_m": round(float(z0), 2),
            "min_agl_m": round(true0, 2),
            "max_agl_m": round(true0, 2),
            "mean_agl_m": round(true0, 2),
            "min_true_height_m": round(true0, 2),
            "max_true_height_m": round(true0, 2),
            "mean_true_height_m": round(true0, 2),
            "min_surface_clearance_m": round(clear0, 2),
            "max_surface_clearance_m": round(clear0, 2),
            "mean_surface_clearance_m": round(clear0, 2),
            "clearance_m": float(clearance_m),
            "preferred_cruise_max_m": float(preferred_cruise_max_m),
            "hard_ceiling_m": float(hard_ceiling_m),
            "min_true_height_m_target": float(min_true_h),
            "max_true_height_m_target": float(max_true_h),
            "endpoint_true_height_m_target": float(endpoint_true_h),
            "total_climb_m": 0.0,
            "total_descent_m": 0.0,
            "vertical_energy_proxy_m": 0.0,
            "max_climb_rate_ms": 0.0,
            "max_descend_rate_ms": 0.0,
            "building_hit_count": int(building_hits),
            "obstacle_hit_count": int(obstacle_hits),
            "agl_target_mean_m": round(sum(agl_target) / len(agl_target), 2) if agl_target else 0.0,
            "profile_sample_count": 1,
        }
        return out_one, meta_one, one_sample

    z_hard_cap = [float(terrain_msl_vals[i]) + float(hard_ceiling_m) for i in range(n)]
    z_pref_cap_local = [float(terrain_msl_vals[i]) + float(preferred_cruise_max_m) for i in range(n)]
    z_true_cap_local = [float(terrain_msl_vals[i]) + float(max_true_h) for i in range(n)]
    z_soft_cap_local = [min(z_pref_cap_local[i], z_true_cap_local[i]) for i in range(n)]
    z_soft_floor_local = [float(terrain_msl_vals[i]) + float(min_true_h) for i in range(n)]
    # Hard bounds: min true-height floor + obstacle clearance + hard ceiling.
    z_lower: List[float] = []
    for i in range(n):
        z_lower.append(max(float(z_soft_floor_local[i]), float(z_min[i])))
    z_upper = [max(z_lower[i], z_hard_cap[i]) for i in range(n)]

    max_up_per_seg = []
    max_down_per_seg = []
    for i in range(1, n):
        ds = math.hypot(samples_xy[i][0] - samples_xy[i - 1][0], samples_xy[i][1] - samples_xy[i - 1][1])
        ds = max(1.0, ds)
        max_up_per_seg.append((climb_ms / max(0.5, speed_ms)) * ds)
        max_down_per_seg.append((descend_ms / max(0.5, speed_ms)) * ds)

    # Forward-looking hard lower-bound envelope:
    # raise earlier points when needed so future obstacle clearance remains reachable
    # under climb-rate limits, instead of failing late near the obstacle.
    z_lower_req = z_lower[:]
    for i in range(n - 2, -1, -1):
        z_lower_req[i] = max(z_lower_req[i], z_lower_req[i + 1] - max_up_per_seg[i])
    for i in range(1, n):
        z_lower_req[i] = max(z_lower_req[i], z_lower_req[i - 1] - max_down_per_seg[i - 1])
    z_lower = z_lower_req
    for i in range(n):
        if z_lower[i] > z_upper[i] + 1e-6:
            raise RuntimeError(
                f"Altitude infeasible at {cum_dist_m[i]:.1f}m: lower bound exceeds hard ceiling"
            )

    # Endpoint true height is a minimum target (>=), not a fixed value.
    endpoint_start_target = float(terrain_msl_vals[0]) + endpoint_true_h
    endpoint_end_target = float(terrain_msl_vals[-1]) + endpoint_true_h
    endpoint_start_msl = min(z_upper[0], max(z_lower[0], endpoint_start_target))
    endpoint_end_msl = min(z_upper[-1], max(z_lower[-1], endpoint_end_target))

    climb_ratio = max(0.05, float(climb_ms) / max(0.5, float(speed_ms)))
    descend_ratio = max(0.05, float(descend_ms) / max(0.5, float(speed_ms)))
    base_cruise_msl = _percentile(z_soft_cap_local, CRUISE_BASE_QUANTILE)
    base_cruise_msl = max(base_cruise_msl, _percentile(z_lower, 0.18))
    cruise_ref_msl = float(base_cruise_msl)
    climb_need_dist = max(0.0, cruise_ref_msl - endpoint_start_msl) / climb_ratio
    descend_need_dist = max(0.0, cruise_ref_msl - endpoint_end_msl) / descend_ratio

    z_desired: List[float] = []
    for i in range(n):
        # Keep a long flat cruise baseline and only climb locally when hard bounds demand it.
        target = max(cruise_ref_msl, z_lower[i])
        target = min(target, z_soft_cap_local[i] + CRUISE_SOFT_CAP_RELAX_M)
        z_desired.append(min(z_upper[i], max(z_lower[i], target)))

    z = z_desired[:]

    z[0] = endpoint_start_msl
    z[-1] = endpoint_end_msl

    for _ in range(26):
        changed = False
        z[0] = endpoint_start_msl
        z[-1] = endpoint_end_msl
        for i in range(1, n):
            if i == n - 1:
                continue
            lower = max(z_lower[i], z[i - 1] - max_down_per_seg[i - 1])
            upper = min(z_upper[i], z[i - 1] + max_up_per_seg[i - 1])
            if upper + 1e-6 < lower:
                raise RuntimeError(
                    f"Altitude infeasible (forward pass) at {cum_dist_m[i]:.1f}m: "
                    "clearance vs climb/descend limits conflict"
                )
            zi = min(upper, max(lower, z[i]))
            if abs(zi - z[i]) > 1e-6:
                z[i] = zi
                changed = True
        for i in range(n - 2, -1, -1):
            if i == 0:
                continue
            lower = max(z_lower[i], z[i + 1] - max_up_per_seg[i])
            upper = min(z_upper[i], z[i + 1] + max_down_per_seg[i])
            if upper + 1e-6 < lower:
                raise RuntimeError(
                    f"Altitude infeasible (backward pass) at {cum_dist_m[i]:.1f}m: "
                    "clearance vs climb/descend limits conflict"
                )
            zi = min(upper, max(lower, z[i]))
            if abs(zi - z[i]) > 1e-6:
                z[i] = zi
                changed = True
        # Pull profile toward soft true-height target while keeping hard feasibility.
        for i in range(1, n - 1):
            lower = max(z_lower[i], z[i - 1] - max_down_per_seg[i - 1], z[i + 1] - max_up_per_seg[i])
            upper = min(z_upper[i], z[i - 1] + max_up_per_seg[i - 1], z[i + 1] + max_down_per_seg[i])
            if upper + 1e-6 < lower:
                raise RuntimeError(
                    f"Altitude infeasible (coupled pass) at {cum_dist_m[i]:.1f}m: "
                    "clearance vs climb/descend limits conflict"
                )
            zi = min(upper, max(lower, z_desired[i]))
            if abs(zi - z[i]) > 1e-6:
                z[i] = zi
                changed = True
        z[0] = endpoint_start_msl
        z[-1] = endpoint_end_msl
        if not changed:
            break

    out_wgs_alt = compress_altitude_waypoints(
        samples_xy=samples_xy,
        samples_wgs=samples_wgs,
        z=z,
        z_min=z_lower,
        z_hard_cap=z_upper,
        cum_dist_m=cum_dist_m,
        mandatory_indices=vertex_indices,
        climb_ms=float(climb_ms),
        descend_ms=float(descend_ms),
        speed_ms=float(speed_ms),
        err_tol_m=ALT_PROFILE_LINK_ERR_TOL_M,
    )
    if len(out_wgs_alt) < 2:
        out_wgs_alt = [(samples_wgs[0][0], samples_wgs[0][1], max(0.0, z[0])), (samples_wgs[-1][0], samples_wgs[-1][1], max(0.0, z[-1]))]

    agl_vals = [max(0.0, z[i] - terrain_msl_vals[i]) for i in range(n)]
    true_h_vals = [max(0.0, z[i] - terrain_msl_vals[i]) for i in range(n)]
    surface_clearance_vals = [max(0.0, z[i] - top_surface_vals[i]) for i in range(n)]
    total_climb_m = 0.0
    total_descent_m = 0.0
    max_climb_rate_ms = 0.0
    max_descend_rate_ms = 0.0
    for i in range(1, n):
        dz = float(z[i] - z[i - 1])
        if dz > 0.0:
            total_climb_m += dz
        elif dz < 0.0:
            total_descent_m += -dz
        ds = max(1e-6, float(cum_dist_m[i] - cum_dist_m[i - 1]))
        rate = abs(dz) * float(speed_ms) / ds
        if dz > 0.0 and rate > max_climb_rate_ms:
            max_climb_rate_ms = rate
        if dz < 0.0 and rate > max_descend_rate_ms:
            max_descend_rate_ms = rate
    vertical_energy_proxy_m = total_climb_m + VERTICAL_DESCEND_ENERGY_FACTOR * total_descent_m

    profile_samples: List[Dict[str, float]] = []
    for i in range(n):
        profile_samples.append(
            {
                "distance_m": round(cum_dist_m[i], 2),
                "route_alt_msl_m": round(float(z[i]), 2),
                "terrain_msl_m": round(float(terrain_msl_vals[i]), 2),
                "surface_msl_m": round(float(top_surface_vals[i]), 2),
                "building_h_m": round(float(building_vals[i]), 2),
                "obstacle_h_m": round(float(obstacle_vals[i]), 2),
                "surface_clearance_m": round(max(0.0, float(z[i]) - float(top_surface_vals[i])), 2),
                "true_height_m": round(max(0.0, float(z[i]) - float(terrain_msl_vals[i])), 2),
            }
        )
    meta = {
        "terrain_source": terrain_source,
        "altitude_points": len(out_wgs_alt),
        "altitude_points_raw": len(samples_xy),
        "route_soft_cap_msl_m": round(float(max(z_pref_cap_local)), 2) if z_pref_cap_local else 0.0,
        "cruise_soft_cap_relax_m": float(CRUISE_SOFT_CAP_RELAX_M),
        "cruise_reference_msl_m": round(float(cruise_ref_msl), 2),
        "planned_climb_phase_m": round(float(climb_need_dist), 2),
        "planned_descend_phase_m": round(float(descend_need_dist), 2),
        "endpoint_start_msl_m": round(float(endpoint_start_msl), 2),
        "endpoint_end_msl_m": round(float(endpoint_end_msl), 2),
        "endpoint_true_height_m_target": float(endpoint_true_h),
        "min_true_height_m_target": float(min_true_h),
        "max_true_height_m_target": float(max_true_h),
        "total_climb_m": round(float(total_climb_m), 2),
        "total_descent_m": round(float(total_descent_m), 2),
        "vertical_energy_proxy_m": round(float(vertical_energy_proxy_m), 2),
        "max_climb_rate_ms": round(float(max_climb_rate_ms), 3),
        "max_descend_rate_ms": round(float(max_descend_rate_ms), 3),
        "min_msl_m": round(min(z), 2) if z else 0.0,
        "max_msl_m": round(max(z), 2) if z else 0.0,
        "mean_msl_m": round(sum(z) / len(z), 2) if z else 0.0,
        "min_agl_m": round(min(agl_vals), 2) if agl_vals else 0.0,
        "max_agl_m": round(max(agl_vals), 2) if agl_vals else 0.0,
        "mean_agl_m": round(sum(agl_vals) / len(agl_vals), 2) if agl_vals else 0.0,
        "min_true_height_m": round(min(true_h_vals), 2) if true_h_vals else 0.0,
        "max_true_height_m": round(max(true_h_vals), 2) if true_h_vals else 0.0,
        "mean_true_height_m": round(sum(true_h_vals) / len(true_h_vals), 2) if true_h_vals else 0.0,
        "min_surface_clearance_m": round(min(surface_clearance_vals), 2) if surface_clearance_vals else 0.0,
        "max_surface_clearance_m": round(max(surface_clearance_vals), 2) if surface_clearance_vals else 0.0,
        "mean_surface_clearance_m": round(sum(surface_clearance_vals) / len(surface_clearance_vals), 2) if surface_clearance_vals else 0.0,
        "clearance_m": float(clearance_m),
        "preferred_cruise_max_m": float(preferred_cruise_max_m),
        "hard_ceiling_m": float(hard_ceiling_m),
        "building_hit_count": building_hits,
        "obstacle_hit_count": obstacle_hits,
        "agl_target_mean_m": round(sum(agl_target) / len(agl_target), 2) if agl_target else 0.0,
        "profile_sample_count": len(profile_samples),
    }
    return out_wgs_alt, meta, profile_samples


def write_kml_absolute(path: Path, points_wgs_alt: List[Tuple[float, float, float]], name: str) -> None:
    coords = " ".join([f"{lon:.6f},{lat:.6f},{alt:.2f}" for lon, lat, alt in points_wgs_alt])
    text = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document>
<name>{name}</name>
<Placemark><name>{name}</name><Style><LineStyle><color>ff2a6df4</color><width>4</width></LineStyle></Style>
<LineString><extrude>1</extrude><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode><coordinates>{coords}</coordinates></LineString></Placemark>
</Document></kml>"""
    path.write_text(text, encoding="utf-8")


def _downsample_profile_samples(samples: List[Dict[str, float]], max_points: int = 260) -> List[Dict[str, float]]:
    if len(samples) <= max_points:
        return samples
    step = max(1, len(samples) // max_points)
    out = samples[::step]
    if out[-1] != samples[-1]:
        out.append(samples[-1])
    return out


def _svg_path(
    samples: List[Dict[str, float]],
    field: str,
    x0: float,
    y0: float,
    plot_w: float,
    plot_h: float,
    dist_max: float,
    h_min: float,
    h_max: float,
) -> str:
    pts = []
    span_h = max(1e-6, h_max - h_min)
    for s in samples:
        x = x0 + (float(s.get("distance_m", 0.0)) / max(1e-6, dist_max)) * plot_w
        v = float(s.get(field, 0.0))
        y = y0 + (1.0 - (v - h_min) / span_h) * plot_h
        pts.append(f"{x:.2f},{y:.2f}")
    return " ".join(pts)


def _build_profile_panel_html(name: str, profile_samples: List[Dict[str, float]]) -> str:
    samples = _downsample_profile_samples(profile_samples, max_points=260)
    if len(samples) < 2:
        return ""

    width = 1100.0
    height = 280.0
    left = 64.0
    right = 18.0
    top = 20.0
    bottom = 36.0
    plot_w = width - left - right
    plot_h = height - top - bottom

    dist_max = max(1.0, float(samples[-1].get("distance_m", 0.0)))
    vals = []
    for s in samples:
        vals.append(float(s.get("route_alt_msl_m", 0.0)))
        vals.append(float(s.get("surface_msl_m", 0.0)))
        vals.append(float(s.get("terrain_msl_m", 0.0)))
    h_lo = min(vals)
    h_hi = max(vals)
    span = max(1.0, h_hi - h_lo)
    h_min = max(0.0, h_lo - span * 0.08)
    h_max = h_hi + span * 0.08

    y_ticks = 5
    grid_parts = []
    label_parts = []
    for i in range(y_ticks + 1):
        y = top + (plot_h * i / y_ticks)
        v = h_max - (h_max - h_min) * i / y_ticks
        grid_parts.append(
            f'<line x1="{left:.1f}" y1="{y:.2f}" x2="{left + plot_w:.1f}" y2="{y:.2f}" stroke="#e0e0e0" stroke-width="1"/>'
        )
        label_parts.append(
            f'<text x="{left - 8:.1f}" y="{y + 4:.2f}" text-anchor="end" fill="#616161" font-size="11">{v:.0f}</text>'
        )

    x_ticks = 6
    for i in range(x_ticks + 1):
        x = left + (plot_w * i / x_ticks)
        km = (dist_max * i / x_ticks) / 1000.0
        grid_parts.append(
            f'<line x1="{x:.2f}" y1="{top:.1f}" x2="{x:.2f}" y2="{top + plot_h:.1f}" stroke="#eeeeee" stroke-width="1"/>'
        )
        label_parts.append(
            f'<text x="{x:.2f}" y="{top + plot_h + 18:.1f}" text-anchor="middle" fill="#616161" font-size="11">{km:.2f} km</text>'
        )

    terrain_path = _svg_path(samples, "terrain_msl_m", left, top, plot_w, plot_h, dist_max, h_min, h_max)
    surface_path = _svg_path(samples, "surface_msl_m", left, top, plot_w, plot_h, dist_max, h_min, h_max)
    route_path = _svg_path(samples, "route_alt_msl_m", left, top, plot_w, plot_h, dist_max, h_min, h_max)
    true_vals = [float(s.get("true_height_m", 0.0)) for s in samples]
    min_true = min(true_vals) if true_vals else 0.0
    mean_true = (sum(true_vals) / len(true_vals)) if true_vals else 0.0

    panel = f"""
<div class="route-profile-panel">
  <div class="route-profile-panel__head">
    <div>
      <div class="route-profile-panel__title">航线垂直剖面</div>
      <div class="route-profile-panel__meta">
        {html.escape(name)} | 距离 {dist_max/1000.0:.2f} km | 真高最小/均值 {min_true:.1f}/{mean_true:.1f} m
      </div>
    </div>
    <button type="button" class="route-profile-panel__toggle" data-role="profile-toggle">收起剖面</button>
  </div>
  <div class="route-profile-panel__body">
    <svg viewBox="0 0 {width:.0f} {height:.0f}" class="route-profile-panel__chart">
      {''.join(grid_parts)}
      <polyline fill="none" stroke="#94a9b8" stroke-width="2" points="{terrain_path}"/>
      <polyline fill="none" stroke="#8d6e63" stroke-width="2.5" points="{surface_path}"/>
      <polyline fill="none" stroke="#1e5ea8" stroke-width="2.8" points="{route_path}"/>
      {''.join(label_parts)}
    </svg>
    <div class="route-profile-panel__legend">
      <span><span class="route-profile-panel__line route-profile-panel__line--route"></span>航线高度（MSL）</span>
      <span><span class="route-profile-panel__line route-profile-panel__line--surface"></span>地表顶面（地形+建筑/障碍物）</span>
      <span><span class="route-profile-panel__line route-profile-panel__line--terrain"></span>地形高度（MSL）</span>
    </div>
  </div>
</div>
"""
    return panel


def _build_preview_theme_head_html() -> str:
    return """
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600;700&family=Noto+Sans+SC:wght@400;500;700&display=swap" rel="stylesheet">
<style>
  :root {
    --panel-bg: rgba(255, 255, 255, 0.94);
    --panel-border: #cfdce7;
    --panel-shadow: 0 18px 36px rgba(13, 44, 82, 0.14);
    --text-main: #16324f;
    --text-sub: #4f6c86;
    --accent: #1e5ea8;
    --accent-soft: #e6f0fb;
    --chip-bg: #f4f8fd;
  }

  html, body {
    background: radial-gradient(circle at 18% 12%, #f6fbff 0%, #eef6ff 42%, #fdfefe 100%);
    font-family: "IBM Plex Sans", "Noto Sans SC", "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
  }

  .leaflet-container {
    font-family: "IBM Plex Sans", "Noto Sans SC", "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
    background: #edf4fb;
  }

  .leaflet-top.leaflet-right {
    top: 14px;
    right: 14px;
  }

  .leaflet-control-zoom {
    border: 1px solid var(--panel-border) !important;
    border-radius: 14px !important;
    overflow: hidden;
    box-shadow: 0 10px 22px rgba(23, 64, 110, 0.15);
  }

  .leaflet-control-zoom a {
    width: 32px;
    height: 32px;
    line-height: 32px;
    color: var(--text-main);
    background: var(--panel-bg);
  }

  .leaflet-control-layers {
    border: 1px solid var(--panel-border) !important;
    border-radius: 16px !important;
    box-shadow: var(--panel-shadow);
    background: var(--panel-bg);
    backdrop-filter: blur(8px);
    min-width: 260px;
  }

  .leaflet-control-layers-expanded {
    padding: 10px 12px;
  }

  .leaflet-control-layers label {
    margin: 2px 0;
    color: var(--text-main);
    font-size: 13px;
    line-height: 1.45;
    font-weight: 500;
  }

  .leaflet-control-layers-list {
    max-height: 45vh;
    overflow: auto;
    padding-right: 4px;
  }

  .route-map-toolbar {
    position: absolute;
    top: 14px;
    left: 14px;
    z-index: 1100;
    width: min(336px, calc(100vw - 28px));
    border: 1px solid var(--panel-border);
    border-radius: 16px;
    background: var(--panel-bg);
    box-shadow: var(--panel-shadow);
    backdrop-filter: blur(10px);
    padding: 12px;
    color: var(--text-main);
  }

  .route-map-toolbar__title {
    font-size: 17px;
    line-height: 1.25;
    font-weight: 700;
    letter-spacing: 0.01em;
  }

  .route-map-toolbar__subtitle {
    margin-top: 3px;
    font-size: 12px;
    color: var(--text-sub);
  }

  .route-map-toolbar__block {
    margin-top: 10px;
  }

  .route-map-toolbar__label {
    font-size: 12px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: #4d6b83;
    margin-bottom: 6px;
  }

  .route-map-toolbar__chips {
    display: flex;
    flex-wrap: wrap;
    gap: 7px;
  }

  .route-chip {
    border: 1px solid #bfd1e3;
    background: var(--chip-bg);
    color: var(--text-main);
    border-radius: 999px;
    padding: 6px 12px;
    font-size: 13px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.16s ease;
  }

  .route-chip.is-active {
    background: var(--accent);
    border-color: var(--accent);
    color: #ffffff;
  }

  .route-map-toolbar__layers {
    display: grid;
    gap: 6px;
    max-height: min(34vh, 280px);
    overflow: auto;
    padding-right: 3px;
  }

  .route-map-toolbar__group {
    margin-top: 6px;
    margin-bottom: 2px;
    font-size: 11px;
    font-weight: 700;
    color: #66819a;
    letter-spacing: 0.06em;
    text-transform: uppercase;
  }

  .route-toggle {
    display: flex;
    align-items: center;
    gap: 9px;
    border-radius: 10px;
    padding: 6px 8px;
    background: #f9fcff;
    border: 1px solid #e0ebf5;
    cursor: pointer;
    font-size: 13px;
    color: #203a56;
    line-height: 1.35;
  }

  .route-toggle input {
    margin: 0;
    width: 15px;
    height: 15px;
    accent-color: var(--accent);
  }

  .route-map-toolbar__actions {
    margin-top: 10px;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .route-map-toolbar__actions button {
    border: 1px solid #b8cde2;
    background: #f4f9ff;
    color: var(--text-main);
    border-radius: 10px;
    padding: 7px 10px;
    font-size: 13px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.16s ease;
  }

  .route-map-toolbar__actions button:hover {
    background: var(--accent-soft);
    border-color: #9fbde0;
  }

  .route-profile-panel {
    position: fixed;
    left: 12px;
    right: 12px;
    bottom: 10px;
    z-index: 1090;
    max-width: 1220px;
    margin: 0 auto;
    border: 1px solid var(--panel-border);
    border-radius: 16px;
    background: var(--panel-bg);
    box-shadow: var(--panel-shadow);
    backdrop-filter: blur(9px);
    padding: 12px 14px;
    color: var(--text-main);
  }

  .route-profile-panel__head {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 12px;
  }

  .route-profile-panel__title {
    font-size: 18px;
    font-weight: 700;
    line-height: 1.2;
  }

  .route-profile-panel__meta {
    margin-top: 3px;
    font-size: 13px;
    color: var(--text-sub);
    line-height: 1.45;
  }

  .route-profile-panel__toggle {
    border: 1px solid #bfd0e2;
    background: #f5faff;
    color: var(--text-main);
    border-radius: 10px;
    padding: 6px 11px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
    white-space: nowrap;
  }

  .route-profile-panel__body {
    margin-top: 8px;
  }

  .route-profile-panel__chart {
    width: 100%;
    height: auto;
    display: block;
    border: 1px solid #dbe8f4;
    border-radius: 12px;
    background: linear-gradient(180deg, #fdfefe 0%, #f2f7fd 100%);
  }

  .route-profile-panel__legend {
    display: flex;
    gap: 16px;
    flex-wrap: wrap;
    margin-top: 8px;
    font-size: 12px;
    color: #2f4b67;
  }

  .route-profile-panel__line {
    display: inline-block;
    width: 17px;
    height: 2px;
    vertical-align: middle;
    margin-right: 6px;
  }

  .route-profile-panel__line--route {
    background: #1e5ea8;
  }

  .route-profile-panel__line--surface {
    background: #8d6e63;
  }

  .route-profile-panel__line--terrain {
    background: #94a9b8;
  }

  .route-profile-panel.is-collapsed .route-profile-panel__body {
    display: none;
  }

  @media (max-width: 960px) {
    .route-map-toolbar {
      width: min(92vw, 332px);
    }
  }

  @media (max-width: 768px) {
    .route-map-toolbar {
      top: auto;
      bottom: 172px;
      left: 10px;
      right: 10px;
      width: auto;
      max-height: 44vh;
      overflow: auto;
    }

    .route-profile-panel {
      left: 8px;
      right: 8px;
      bottom: 8px;
      padding: 10px;
    }

    .route-profile-panel__title {
      font-size: 16px;
    }

    .route-profile-panel__meta {
      font-size: 12px;
    }
  }
</style>
"""


def _build_preview_toolbar_script_html() -> str:
    return """
<script>
(function () {
  function hasAny(text, keywords) {
    const source = String(text || "");
    return keywords.some(function (k) {
      return source.indexOf(k) >= 0;
    });
  }

  function groupName(layerName) {
    if (hasAny(layerName, ["候选航线", "Candidate:", "主航线", "Route (3D"])) return "航线方案";
    if (hasAny(layerName, ["禁飞", "No-fly", "缓冲", "Buffer", "起点", "Start/End"])) return "安全边界";
    if (hasAny(layerName, ["高层", "High Buildings", "学校", "School", "人群", "Crowd", "关键", "Line Risks", "低风险"])) return "风险参考";
    return "其他";
  }

  function setupProfilePanel() {
    const panel = document.querySelector(".route-profile-panel");
    if (!panel || panel.dataset.ready === "1") return;
    const toggle = panel.querySelector('[data-role="profile-toggle"]');
    if (!toggle) return;
    toggle.addEventListener("click", function () {
      const collapsed = panel.classList.toggle("is-collapsed");
      toggle.textContent = collapsed ? "展开剖面" : "收起剖面";
    });
    panel.dataset.ready = "1";
  }

  function setOverlayVisible(map, entry, visible) {
    if (!entry || !entry.layer) return;
    if (visible) {
      if (!map.hasLayer(entry.layer)) map.addLayer(entry.layer);
    } else {
      if (map.hasLayer(entry.layer)) map.removeLayer(entry.layer);
    }
  }

  function setupToolbar(map) {
    if (!map || map._routeToolbarReady) return true;
    const control = map._controlLayers;
    if (!control || !control._layers) return false;
    const entries = Object.values(control._layers);
    if (!entries.length) return false;

    const container = map.getContainer();
    if (!container || container.querySelector(".route-map-toolbar")) return true;

    const baseEntries = entries.filter(function (entry) {
      return !entry.overlay;
    });
    const overlayEntries = entries.filter(function (entry) {
      return !!entry.overlay;
    });
    if (!baseEntries.length) return false;

    const toolbar = document.createElement("div");
    toolbar.className = "route-map-toolbar";
    toolbar.innerHTML =
      '<div class="route-map-toolbar__title">地图工具</div>' +
      '<div class="route-map-toolbar__subtitle">快速切换底图与图层</div>' +
      '<div class="route-map-toolbar__block">' +
      '<div class="route-map-toolbar__label">底图</div>' +
      '<div class="route-map-toolbar__chips" data-role="base"></div>' +
      '</div>' +
      '<div class="route-map-toolbar__block">' +
      '<div class="route-map-toolbar__label">图层</div>' +
      '<div class="route-map-toolbar__layers" data-role="layers"></div>' +
      '</div>' +
      '<div class="route-map-toolbar__actions">' +
      '<button type="button" data-role="preset-clear">清爽视图</button>' +
      '<button type="button" data-role="preset-risk">风险视图</button>' +
      "</div>";
    container.appendChild(toolbar);

    const baseWrap = toolbar.querySelector('[data-role="base"]');
    const layerWrap = toolbar.querySelector('[data-role="layers"]');
    const baseButtons = [];
    const overlayInputs = [];
    const satLabels = overlayEntries.find(function (entry) {
      return hasAny(entry.name, ["卫星注记"]);
    });

    function applyBase(target) {
      baseEntries.forEach(function (entry) {
        if (entry.layer && map.hasLayer(entry.layer)) {
          map.removeLayer(entry.layer);
        }
      });
      if (target && target.layer) map.addLayer(target.layer);
      if (satLabels) {
        const useSat = !!target && hasAny(target.name, ["卫星"]);
        setOverlayVisible(map, satLabels, useSat);
      }
      syncState();
    }

    function syncState() {
      baseButtons.forEach(function (item) {
        const active = !!item.entry.layer && map.hasLayer(item.entry.layer);
        item.button.classList.toggle("is-active", active);
      });
      overlayInputs.forEach(function (item) {
        item.input.checked = !!item.entry.layer && map.hasLayer(item.entry.layer);
      });
    }

    baseEntries.forEach(function (entry) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "route-chip";
      button.textContent = String(entry.name || "底图");
      button.addEventListener("click", function () {
        applyBase(entry);
      });
      baseWrap.appendChild(button);
      baseButtons.push({ entry: entry, button: button });
    });

    const grouped = overlayEntries.map(function (entry) {
      return { entry: entry, group: groupName(entry.name), label: String(entry.name || "图层") };
    });
    grouped.sort(function (a, b) {
      const order = { "航线方案": 1, "安全边界": 2, "风险参考": 3, "其他": 4 };
      const gDiff = (order[a.group] || 9) - (order[b.group] || 9);
      if (gDiff !== 0) return gDiff;
      return a.label.localeCompare(b.label, "zh-Hans-CN");
    });

    let currentGroup = "";
    grouped.forEach(function (item) {
      if (item.group !== currentGroup) {
        currentGroup = item.group;
        const heading = document.createElement("div");
        heading.className = "route-map-toolbar__group";
        heading.textContent = currentGroup;
        layerWrap.appendChild(heading);
      }
      const row = document.createElement("label");
      row.className = "route-toggle";
      const input = document.createElement("input");
      input.type = "checkbox";
      input.checked = !!item.entry.layer && map.hasLayer(item.entry.layer);
      input.addEventListener("change", function () {
        setOverlayVisible(map, item.entry, input.checked);
      });
      const text = document.createElement("span");
      text.textContent = item.label;
      row.appendChild(input);
      row.appendChild(text);
      layerWrap.appendChild(row);
      overlayInputs.push({ entry: item.entry, input: input });
    });

    function applyPreset(mode) {
      overlayEntries.forEach(function (entry) {
        if (satLabels && entry === satLabels) return;
        const name = String(entry.name || "");
        let visible = false;
        if (mode === "clear") {
          visible = hasAny(name, ["主航线", "Route (3D", "起点/终点", "Start/End", "禁飞", "No-fly", "缓冲", "Buffer", "候选航线：2", "Candidate: 2"]);
        } else if (mode === "risk") {
          visible = hasAny(name, ["主航线", "Route (3D", "起点/终点", "Start/End", "禁飞", "No-fly", "高层", "High Buildings", "学校", "School", "人群", "Crowd", "关键设施", "Key Facility", "关键基础设施", "Critical Infrastructure", "线性风险", "Line Risks"]);
        }
        setOverlayVisible(map, entry, visible);
      });
      syncState();
    }

    const presetClear = toolbar.querySelector('[data-role="preset-clear"]');
    const presetRisk = toolbar.querySelector('[data-role="preset-risk"]');
    if (presetClear) {
      presetClear.addEventListener("click", function () {
        applyPreset("clear");
      });
    }
    if (presetRisk) {
      presetRisk.addEventListener("click", function () {
        applyPreset("risk");
      });
    }

    map.on("layeradd layerremove baselayerchange", syncState);
    map._routeToolbarReady = true;
    syncState();
    return true;
  }

  function boot() {
    setupProfilePanel();
    if (typeof L === "undefined") return false;
    const map = Object.values(window).find(function (value) {
      return value && value instanceof L.Map;
    });
    if (!map) return false;
    return setupToolbar(map);
  }

  let attempts = 0;
  if (boot()) return;
  const timer = window.setInterval(function () {
    attempts += 1;
    if (boot() || attempts > 60) {
      window.clearInterval(timer);
    }
  }, 150);
})();
</script>
"""


def write_preview_html(
    path: Path,
    route_points: List[Tuple[float, float, float]],
    candidate_routes_wgs: List[Dict[str, Any]],
    start_wgs: Tuple[float, float],
    end_wgs: Tuple[float, float],
    nofly_polys_xy: List[Any],
    route_buffer_xy,
    high_building_polys_xy: List[Any],
    crowd_points_xy: List[Any],
    key_points_xy: List[Any],
    infra_geoms_xy: List[Any],
    high_road_lines_xy: List[Any],
    hsr_lines_xy: List[Any],
    line_risk_union_xy,
    low_risk_landuse_xy: List[Any],
    school_hard_zones_xy: List[Any],
    school_points_xy: List[Any],
    profile_samples: List[Dict[str, float]],
    inv,
    name: str,
) -> None:
    def _poly_to_latlon(poly_wgs) -> List[Tuple[float, float]]:
        return [(lat, lon) for lon, lat in poly_wgs.exterior.coords]

    def _add_polygon_layer(
        layer: folium.FeatureGroup,
        geoms_xy: List[Any],
        color: str,
        fill_opacity: float,
        max_items: int,
    ) -> None:
        for geom_xy in geoms_xy[:max_items]:
            try:
                geom_wgs = transform(inv, geom_xy)
                if geom_wgs.geom_type == "Polygon":
                    folium.Polygon(
                        _poly_to_latlon(geom_wgs),
                        color=color,
                        weight=1,
                        fill=True,
                        fill_opacity=fill_opacity,
                    ).add_to(layer)
                elif geom_wgs.geom_type == "MultiPolygon":
                    for p in geom_wgs.geoms:
                        folium.Polygon(
                            _poly_to_latlon(p),
                            color=color,
                            weight=1,
                            fill=True,
                            fill_opacity=fill_opacity,
                        ).add_to(layer)
            except Exception:
                continue

    def _add_line_layer(layer: folium.FeatureGroup, geoms_xy: List[Any], color: str, max_items: int) -> None:
        for geom_xy in geoms_xy[:max_items]:
            try:
                geom_wgs = transform(inv, geom_xy)
                if geom_wgs.geom_type == "LineString":
                    coords = [(lat, lon) for lon, lat in geom_wgs.coords]
                    folium.PolyLine(coords, color=color, weight=2, opacity=0.85).add_to(layer)
                elif geom_wgs.geom_type == "MultiLineString":
                    for part in geom_wgs.geoms:
                        coords = [(lat, lon) for lon, lat in part.coords]
                        folium.PolyLine(coords, color=color, weight=2, opacity=0.85).add_to(layer)
            except Exception:
                continue

    center_lat = (start_wgs[1] + end_wgs[1]) / 2.0
    center_lon = (start_wgs[0] + end_wgs[0]) / 2.0
    m = folium.Map(location=[center_lat, center_lon], zoom_start=12, control_scale=True, tiles=None, prefer_canvas=True)

    folium.TileLayer(
        tiles="OpenStreetMap",
        name="普通地图",
        overlay=False,
        control=True,
        show=True,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png",
        attr='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors '
        '&copy; <a href="https://carto.com/">CARTO</a>',
        name="浅色底图",
        overlay=False,
        control=True,
        show=False,
        max_zoom=20,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Tiles &copy; Esri",
        name="卫星影像",
        overlay=False,
        control=True,
        show=False,
        max_zoom=20,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}",
        attr="Labels &copy; Esri",
        name="卫星注记",
        overlay=True,
        control=True,
        show=False,
        opacity=0.9,
        max_zoom=20,
    ).add_to(m)

    palette = {
        "safety_water": "#00897b",
        "safety_default": "#1565c0",
        "efficiency": "#ef6c00",
    }
    for cand in candidate_routes_wgs:
        cid = str(cand.get("id", "candidate"))
        label = str(cand.get("label", cid))
        coords = cand.get("coords") or []
        if len(coords) < 2:
            continue
        color = palette.get(cid, "#455a64")
        show_flag = bool(cand.get("show", False))
        fg = folium.FeatureGroup(name=f"候选航线：{label}", show=show_flag)
        folium.PolyLine(
            coords,
            color=color,
            weight=4 if show_flag else 3,
            opacity=0.92 if show_flag else 0.78,
            tooltip=f"{label} | {cand.get('distance_km', 0.0):.2f} km",
        ).add_to(fg)
        fg.add_to(m)

    route_layer = folium.FeatureGroup(name="主航线（3D高度）", show=True)
    route_coords = [(lat, lon) for lon, lat, _ in route_points]
    alts = [alt for _, _, alt in route_points]
    tip = f"{name} | 高度最小/最大 {min(alts):.1f}/{max(alts):.1f} m"
    folium.PolyLine(route_coords, color="#1565c0", weight=4, tooltip=tip).add_to(route_layer)
    step = max(1, len(route_points) // 120)
    for i in range(0, len(route_points), step):
        lon, lat, alt = route_points[i]
        c = "#0d47a1" if alt <= 90 else ("#f57f17" if alt <= 120 else "#c62828")
        folium.CircleMarker(
            [lat, lon],
            radius=2,
            color=c,
            fill=True,
            fill_opacity=0.8,
            tooltip=f"高度 {alt:.1f} m",
        ).add_to(route_layer)
    route_layer.add_to(m)

    buf_layer = folium.FeatureGroup(name=f"航线缓冲区 {int(ROUTE_BUFFER_M)}m", show=True)
    if route_buffer_xy is not None and (not route_buffer_xy.is_empty):
        _add_polygon_layer(buf_layer, [route_buffer_xy], color="#1e88e5", fill_opacity=0.08, max_items=1)
    buf_layer.add_to(m)

    nofly_layer = folium.FeatureGroup(name="禁飞区（民航/军用机场）", show=True)
    _add_polygon_layer(nofly_layer, nofly_polys_xy, color="#d32f2f", fill_opacity=0.2, max_items=200)
    nofly_layer.add_to(m)

    high_build_layer = folium.FeatureGroup(name=f"高层建筑（>={int(HIGH_BUILDING_THRESHOLD_M)}m）", show=False)
    _add_polygon_layer(high_build_layer, high_building_polys_xy, color="#ef6c00", fill_opacity=0.22, max_items=700)
    high_build_layer.add_to(m)

    low_risk_layer = folium.FeatureGroup(name="低风险地表（水域/林地/绿地）", show=False)
    _add_polygon_layer(low_risk_layer, low_risk_landuse_xy, color="#2e7d32", fill_opacity=0.08, max_items=800)
    low_risk_layer.add_to(m)

    school_layer = folium.FeatureGroup(name="学校/幼儿园避让区", show=False)
    _add_polygon_layer(school_layer, school_hard_zones_xy, color="#c62828", fill_opacity=0.16, max_items=600)
    for pxy in school_points_xy[:800]:
        try:
            pwgs = transform(inv, pxy)
            folium.CircleMarker([pwgs.y, pwgs.x], radius=2, color="#b71c1c", fill=True, fill_opacity=0.85).add_to(school_layer)
        except Exception:
            continue
    school_layer.add_to(m)

    crowd_layer = folium.FeatureGroup(name="人群敏感 POI", show=False)
    for pxy in crowd_points_xy[:1200]:
        try:
            pwgs = transform(inv, pxy)
            folium.CircleMarker([pwgs.y, pwgs.x], radius=2, color="#ad1457", fill=True, fill_opacity=0.7).add_to(crowd_layer)
        except Exception:
            continue
    crowd_layer.add_to(m)

    key_layer = folium.FeatureGroup(name="关键设施 POI", show=False)
    for pxy in key_points_xy[:800]:
        try:
            pwgs = transform(inv, pxy)
            folium.CircleMarker([pwgs.y, pwgs.x], radius=3, color="#6a1b9a", fill=True, fill_opacity=0.85).add_to(key_layer)
        except Exception:
            continue
    key_layer.add_to(m)

    infra_layer = folium.FeatureGroup(name="关键基础设施", show=False)
    _add_polygon_layer(infra_layer, infra_geoms_xy, color="#455a64", fill_opacity=0.14, max_items=500)
    infra_layer.add_to(m)

    line_layer = folium.FeatureGroup(name="线性风险（高速/高铁）", show=False)
    _add_line_layer(line_layer, high_road_lines_xy, color="#b71c1c", max_items=800)
    _add_line_layer(line_layer, hsr_lines_xy, color="#1b5e20", max_items=500)
    if line_risk_union_xy is not None and (not line_risk_union_xy.is_empty):
        _add_polygon_layer(line_layer, [line_risk_union_xy], color="#8d6e63", fill_opacity=0.1, max_items=1)
    line_layer.add_to(m)

    start_end_layer = folium.FeatureGroup(name="起点/终点", show=True)
    folium.Marker([start_wgs[1], start_wgs[0]], tooltip="起点", icon=folium.Icon(color="green")).add_to(start_end_layer)
    folium.Marker([end_wgs[1], end_wgs[0]], tooltip="终点", icon=folium.Icon(color="red")).add_to(start_end_layer)
    start_end_layer.add_to(m)

    folium.LayerControl(collapsed=True).add_to(m)
    html_text = m.get_root().render()
    if "</head>" in html_text:
        html_text = html_text.replace("</head>", _build_preview_theme_head_html() + "\n</head>")
    panel_html = _build_profile_panel_html(name=name, profile_samples=profile_samples)
    if panel_html and "</body>" in html_text:
        html_text = html_text.replace("</body>", panel_html + "\n</body>")
    if "</html>" in html_text:
        html_text = html_text.replace("</html>", _build_preview_toolbar_script_html() + "\n</html>")
    path.write_text(html_text, encoding="utf-8")


def dedup_routes(routes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
    seen = set()
    out = []
    for r in routes:
        key = tuple((_round_key(p, 25.0) for p in r[:: max(1, len(r) // 80)]))
        if key in seen:
            continue
        seen.add(key)
        out.append(r)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Automatic OD route planner for urban UAV logistics.")
    parser.add_argument("--city", required=True, help="City name for cache and context.")
    parser.add_argument("--start-lon", type=float, default=None)
    parser.add_argument("--start-lat", type=float, default=None)
    parser.add_argument("--end-lon", type=float, default=None)
    parser.add_argument("--end-lat", type=float, default=None)
    parser.add_argument("--od-kml", default="", help="Optional KML. Use first and last points as OD.")
    parser.add_argument("--name", default="auto_route", help="Output route base name.")
    parser.add_argument("--city-zoom", default="8-14")
    parser.add_argument("--profile", choices=["fastest", "balanced", "safest"], default="balanced")
    parser.add_argument(
        "--select-candidate",
        choices=["safety_water", "safety_default", "efficiency"],
        default="safety_default",
        help="Select which internal candidate to export as main route output.",
    )
    parser.add_argument("--top-k", type=int, default=1, help="Candidate routes to evaluate (phase-1 summary).")
    parser.add_argument(
        "--open-data-no-fly",
        dest="open_data_no_fly",
        action="store_true",
        default=True,
        help="Enable airport/military-airfield no-fly from OSM.",
    )
    parser.add_argument(
        "--no-open-data-no-fly",
        dest="open_data_no_fly",
        action="store_false",
        help="Disable open-data no-fly filtering.",
    )
    parser.add_argument("--soft-no-fly-scale", type=float, default=1.0, help="Scale factor for soft no-fly penalty.")
    parser.add_argument("--infra-hard-buffer-m", type=float, default=0.0, help="Hard exclusion around key infrastructure.")
    parser.add_argument("--clearance-m", type=float, default=DEFAULT_AIRFRAME["clearance_m"])
    parser.add_argument(
        "--endpoint-true-height-m",
        type=float,
        default=ENDPOINT_TRUE_HEIGHT_M,
        help="Target true height (above local top surface) at start/end points.",
    )
    parser.add_argument(
        "--min-true-height-m",
        type=float,
        default=MIN_TRUE_HEIGHT_M,
        help="Minimum true height floor along route (above local top surface).",
    )
    parser.add_argument(
        "--max-true-height-m",
        type=float,
        default=MAX_TRUE_HEIGHT_M,
        help="Maximum true height target along route (above local top surface).",
    )
    parser.add_argument("--speed-ms", type=float, default=DEFAULT_AIRFRAME["speed_ms"])
    parser.add_argument("--climb-ms", type=float, default=DEFAULT_AIRFRAME["climb_ms"])
    parser.add_argument("--descend-ms", type=float, default=DEFAULT_AIRFRAME["descend_ms"])
    parser.add_argument("--turn-radius-m", type=float, default=DEFAULT_AIRFRAME["turn_radius_m"])
    parser.add_argument(
        "--min-turn-keep-deg",
        type=float,
        default=LOW_VALUE_TURN_KEEP_DEG,
        help="Post-process: keep turns above this angle, prune lower-yield bends when safe.",
    )
    parser.add_argument(
        "--turn-prune-passes",
        type=int,
        default=2,
        help="Post-process passes for low-value turn pruning.",
    )
    parser.add_argument(
        "--max-waypoints",
        type=int,
        default=0,
        help="Optional hard cap for horizontal waypoints after smoothing (0 means auto/no hard cap).",
    )
    parser.add_argument(
        "--min-turn-angle-deg",
        type=float,
        default=120.0,
        help="Minimum interior turn angle for final horizontal route (acute turns below this are forbidden).",
    )
    parser.add_argument(
        "--vertical-tradeoff",
        dest="vertical_tradeoff",
        action="store_true",
        default=True,
        help="Enable detour-vs-vertical workload trade-off when selecting candidate route.",
    )
    parser.add_argument(
        "--no-vertical-tradeoff",
        dest="vertical_tradeoff",
        action="store_false",
        help="Disable detour-vs-vertical workload trade-off.",
    )
    parser.add_argument(
        "--vertical-detour-limit-ratio",
        type=float,
        default=1.18,
        help="Max allowed distance ratio (vs requested candidate) for vertical workload trade-off switching.",
    )
    parser.add_argument(
        "--vertical-improve-ratio",
        type=float,
        default=0.18,
        help="Minimum vertical workload reduction ratio required to accept a detour switch.",
    )
    parser.add_argument(
        "--vertical-energy-weight",
        type=float,
        default=1.25,
        help="Trade-off weight: larger values prioritize reduced climb/descend workload.",
    )
    parser.add_argument("--preferred-cruise-max-m", type=float, default=PREFERRED_CRUISE_ALT_M)
    parser.add_argument("--hard-ceiling-m", type=float, default=HARD_CEILING_ALT_M)
    parser.add_argument("--dem-tif", default="", help="Optional DEM GeoTIFF for terrain.")
    parser.add_argument("--opentopo-endpoint", default=DEFAULT_OPENTOPO_ENDPOINT)
    parser.add_argument("--out-dir", default="output/auto_routes")
    parser.add_argument("--workflow-out-dir", default="output/full-workflow-v2-auto-route")
    parser.add_argument("--workflow-xlsx", default="RA_v2_auto_route.xlsx")
    parser.add_argument("--run-workflow-v2", action="store_true")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[3]
    if args.od_kml:
        od_coords = parse_kml_coords(Path(args.od_kml).resolve())
        start_wgs = (od_coords[0][0], od_coords[0][1])
        end_wgs = (od_coords[-1][0], od_coords[-1][1])
    else:
        required = [args.start_lon, args.start_lat, args.end_lon, args.end_lat]
        if any(v is None for v in required):
            raise ValueError("Provide either --od-kml or full OD coordinates.")
        start_wgs = (float(args.start_lon), float(args.start_lat))
        end_wgs = (float(args.end_lon), float(args.end_lat))

    ensure_city_data(root, args.city, args.city_zoom)
    cache_dir = _find_city_cache_dir(root, args.city)
    if cache_dir is None:
        raise FileNotFoundError(f"City cache dir not found for: {args.city}")
    summary_path = cache_dir / "download_summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(f"download_summary.json missing for city: {args.city}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    fwd, inv = build_projectors([start_wgs, end_wgs])
    start_xy = transform(fwd, Point(start_wgs)).coords[0]
    end_xy = transform(fwd, Point(end_wgs)).coords[0]
    route_bbox = route_bbox_wgs([start_wgs, end_wgs], margin_m=4500.0)
    route_bbox_building = route_bbox_wgs([start_wgs, end_wgs], margin_m=2800.0)

    pop_tif = Path(summary.get("outputs", {}).get("population", {}).get("clipped_tif", ""))
    pop_sampler = PopulationSampler(pop_tif if pop_tif.exists() else None)
    landuse_geoms, landuse_costs, landuse_tree = build_landuse_index(summary, fwd)
    landuse_id_map = _build_geom_id_map(landuse_geoms)
    infra_geoms, infra_severities, infra_tree = build_infrastructure_index(summary, route_bbox, fwd)
    infra_id_map = _build_geom_id_map(infra_geoms)
    crowd_points_xy, crowd_tree, crowd_id_map, key_points_xy, key_tree, key_id_map, poi_risk_counter = build_poi_risk_indices(
        summary,
        fwd,
    )
    high_road_lines_xy, line_risk_union_xy, hsr_lines_xy, line_risk_counter = build_line_risk_geometries(summary, route_bbox, fwd)
    school_hard_zones_xy, school_points_xy, school_counter = fetch_school_kindergarten_zones(route_bbox, fwd)
    if school_points_xy:
        crowd_points_xy.extend(school_points_xy)
        poi_risk_counter["crowd"] = int(poi_risk_counter.get("crowd", 0) + len(school_points_xy))
        crowd_tree = STRtree(crowd_points_xy) if crowd_points_xy else None
        crowd_id_map = _build_geom_id_map(crowd_points_xy)
    school_hard_union_xy = unary_union(school_hard_zones_xy) if school_hard_zones_xy else None
    if school_hard_union_xy is not None and (not school_hard_union_xy.is_empty):
        relief = Point(start_xy).buffer(SCHOOL_ENDPOINT_RELIEF_M).union(Point(end_xy).buffer(SCHOOL_ENDPOINT_RELIEF_M))
        try:
            school_hard_union_xy = school_hard_union_xy.difference(relief)
        except Exception:
            pass
        if school_hard_union_xy.is_empty:
            school_hard_union_xy = None

    nofly_hard_polys_xy: List[Any] = []
    nofly_soft_polys_xy: List[Any] = []
    nofly_counter = {"civil_airport": 0, "military_airport": 0, "hard": 0, "soft": 0}
    if args.open_data_no_fly:
        nofly_hard_polys_xy, nofly_soft_polys_xy, nofly_counter = fetch_open_data_no_fly_zones(route_bbox, fwd)
    nofly_hard_union_xy = unary_union(nofly_hard_polys_xy) if nofly_hard_polys_xy else None
    nofly_soft_union_xy = unary_union(nofly_soft_polys_xy) if nofly_soft_polys_xy else None
    if nofly_hard_union_xy is not None and (not nofly_hard_union_xy.is_empty):
        if Point(start_xy).intersects(nofly_hard_union_xy):
            raise RuntimeError("Start point is inside open-data no-fly zone. Please move OD or disable --open-data-no-fly.")
        if Point(end_xy).intersects(nofly_hard_union_xy):
            raise RuntimeError("End point is inside open-data no-fly zone. Please move OD or disable --open-data-no-fly.")

    b_geoms, b_heights, o_geoms, o_heights, obstacle_counter = fetch_osm_buildings_and_obstacles(route_bbox_building, fwd)
    b_tree = STRtree(b_geoms) if b_geoms else None
    o_tree = STRtree(o_geoms) if o_geoms else None
    b_id_map = _build_geom_id_map(b_geoms)
    o_id_map = _build_geom_id_map(o_geoms)
    high_building_polys_xy = [g.buffer(HIGH_BUILDING_AVOID_BUFFER_M) for g, h in zip(b_geoms, b_heights) if h >= HIGH_BUILDING_THRESHOLD_M]
    high_building_union_xy = unary_union(high_building_polys_xy) if high_building_polys_xy else None

    infra_hard_union_xy = None
    if args.infra_hard_buffer_m > 0 and infra_geoms:
        hard_geoms = [g.buffer(args.infra_hard_buffer_m) for g, sev in zip(infra_geoms, infra_severities) if sev >= 2.0]
        if hard_geoms:
            infra_hard_union_xy = unary_union(hard_geoms)

    networks_wgs = load_city_networks(summary, route_bbox)
    networks_xy = project_lines(networks_wgs, fwd)
    if not networks_xy:
        raise RuntimeError("No route networks available.")

    direct_dist_m = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    water_lines_xy = []
    for item in networks_xy:
        if str(item.get("network_type", "")).lower() != "water":
            continue
        coords = item.get("coords") or []
        if len(coords) < 2:
            continue
        try:
            water_lines_xy.append(LineString(coords))
        except Exception:
            continue

    def _nearest_dist_to_water(pt_xy: Tuple[float, float]) -> float:
        if not water_lines_xy:
            return float("inf")
        p = Point(pt_xy)
        best = float("inf")
        for line in water_lines_xy:
            d = p.distance(line)
            if d < best:
                best = d
        return best

    direct_corridor_xy = LineString([start_xy, end_xy]).buffer(900.0)
    water_total_len_m = 0.0
    for line in water_lines_xy:
        try:
            if line.intersects(direct_corridor_xy):
                water_total_len_m += float(line.intersection(direct_corridor_xy).length)
        except Exception:
            continue
    water_near_start_m = _nearest_dist_to_water(start_xy)
    water_near_end_m = _nearest_dist_to_water(end_xy)
    long_water_available = bool(
        water_total_len_m >= 1800.0
        and water_near_start_m <= 520.0
        and water_near_end_m <= 520.0
    )

    def _solve_scheme(
        scheme_id: str,
        label: str,
        profile_key: str,
        enable_water_connectors: bool,
        water_pref_factor: float,
        allow_water_choice: bool,
        water_detour_limit: float,
        min_water_share: float,
        weight_scale: Dict[str, float],
        school_penalty_air: float,
        school_penalty_ground: float,
    ) -> Optional[Dict[str, Any]]:
        local_weights = WEIGHT_PROFILES[profile_key].copy()
        min_turn_angle_deg = max(60.0, min(179.0, float(args.min_turn_angle_deg)))
        max_turn_deflection_deg = max(1.0, 180.0 - min_turn_angle_deg)

        def _turn_angle_ok(poly: List[Tuple[float, float]]) -> bool:
            return polyline_min_interior_angle(poly) >= (min_turn_angle_deg - 1e-6)

        for wk, sv in (weight_scale or {}).items():
            if wk in local_weights:
                local_weights[wk] = max(0.01, float(local_weights[wk]) * float(sv))
        local_weights["soft_no_fly"] = max(0.0, local_weights["soft_no_fly"] * float(args.soft_no_fly_scale))
        graph_local, graph_stats_local = build_navigation_graph(
            networks_xy,
            start_xy,
            end_xy,
            weights=local_weights,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            no_fly_soft_union_xy=nofly_soft_union_xy,
            infra_hard_union_xy=infra_hard_union_xy,
            landuse_geoms=landuse_geoms,
            landuse_costs=landuse_costs,
            landuse_tree=landuse_tree,
            landuse_id_map=landuse_id_map,
            infra_geoms=infra_geoms,
            infra_severities=infra_severities,
            infra_tree=infra_tree,
            infra_id_map=infra_id_map,
            pop_sampler=pop_sampler,
            inv=inv,
            b_geoms=b_geoms,
            b_heights=b_heights,
            b_tree=b_tree,
            b_id_map=b_id_map,
            o_geoms=o_geoms,
            o_heights=o_heights,
            o_tree=o_tree,
            o_id_map=o_id_map,
            crowd_points_xy=crowd_points_xy,
            crowd_tree=crowd_tree,
            crowd_id_map=crowd_id_map,
            crowd_hard_union_xy=school_hard_union_xy,
            school_penalty_air=float(school_penalty_air),
            school_penalty_ground=float(school_penalty_ground),
            key_points_xy=key_points_xy,
            key_tree=key_tree,
            key_id_map=key_id_map,
            line_risk_union_xy=line_risk_union_xy,
            high_building_union_xy=high_building_union_xy,
            enable_water_endpoint_connectors=enable_water_connectors,
        )
        start_node = _round_key(start_xy, GRAPH_NODE_SNAP_M)
        end_node = _round_key(end_xy, GRAPH_NODE_SNAP_M)
        base_nodes, base_cost = astar_with_turn_penalty(
            graph_local,
            start_node,
            end_node,
            turn_weight=local_weights["turn"],
            turn_radius_m=args.turn_radius_m,
            water_pref_factor=1.0,
            max_turn_deflection_deg=max_turn_deflection_deg,
        )
        water_nodes, water_cost = astar_with_turn_penalty(
            graph_local,
            start_node,
            end_node,
            turn_weight=local_weights["turn"],
            turn_radius_m=args.turn_radius_m,
            water_pref_factor=water_pref_factor,
            max_turn_deflection_deg=max_turn_deflection_deg,
        )
        chosen_nodes = base_nodes
        chosen_cost = base_cost
        strategy = "base"
        if chosen_nodes is None or len(chosen_nodes) < 2:
            chosen_nodes = water_nodes
            chosen_cost = water_cost
            strategy = "water_fallback"
        if chosen_nodes is None or len(chosen_nodes) < 2 or not chosen_cost:
            return None
        if allow_water_choice and water_nodes is not None and water_cost and base_cost:
            base_dist = max(1e-6, base_cost.get("distance_m", 0.0))
            water_dist = water_cost.get("distance_m", 0.0)
            base_share = base_cost.get("water_distance_m", 0.0) / base_dist
            water_share = water_cost.get("water_distance_m", 0.0) / max(1e-6, water_dist)
            base_total = base_cost.get("total_cost", float("inf"))
            water_total = water_cost.get("total_cost", float("inf"))
            water_detour_ok = water_dist <= max(1e-6, direct_dist_m) * max(1.0, water_detour_limit)
            water_gain_ok = water_share >= max(min_water_share, base_share + 0.05)
            if water_detour_ok and water_gain_ok and (water_total <= base_total * 1.08 or water_share >= 0.55):
                chosen_nodes = water_nodes
                chosen_cost = water_cost
                strategy = "water_priority_selected"

        if not _turn_angle_ok(chosen_nodes):
            return None

        turns_before_local = polyline_turn_count(chosen_nodes, angle_threshold_deg=TURN_IGNORE_DEG)
        nodes = chosen_nodes[:]
        cand = simplify_polyline(nodes, tol_m=14.0)
        if _turn_angle_ok(cand):
            nodes = cand
        cand = shortcut_polyline(
            nodes,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=infra_hard_union_xy,
            crowd_hard_union_xy=school_hard_union_xy,
            passes=3,
            max_hop=20,
            max_jump_m=2000.0,
        )
        cand = enforce_min_turn_angle(
            cand,
            min_turn_angle_deg=min_turn_angle_deg,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=infra_hard_union_xy,
            crowd_hard_union_xy=school_hard_union_xy,
            passes=4,
        )
        if _turn_angle_ok(cand):
            nodes = cand
        cand = prune_low_value_turns(
            nodes,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=infra_hard_union_xy,
            crowd_hard_union_xy=school_hard_union_xy,
            min_turn_keep_deg=float(args.min_turn_keep_deg),
            passes=max(1, int(args.turn_prune_passes)),
        )
        cand = enforce_min_turn_angle(
            cand,
            min_turn_angle_deg=min_turn_angle_deg,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=infra_hard_union_xy,
            crowd_hard_union_xy=school_hard_union_xy,
            passes=3,
        )
        if _turn_angle_ok(cand):
            nodes = cand
        cand = simplify_polyline(nodes, tol_m=20.0)
        if _turn_angle_ok(cand):
            nodes = cand
        if int(args.max_waypoints) > 2:
            cand = enforce_waypoint_budget(
                nodes,
                max_waypoints=int(args.max_waypoints),
                pop_sampler=pop_sampler,
                inv=inv,
                no_fly_hard_union_xy=nofly_hard_union_xy,
                infra_hard_union_xy=infra_hard_union_xy,
                crowd_hard_union_xy=school_hard_union_xy,
            )
            cand = enforce_min_turn_angle(
                cand,
                min_turn_angle_deg=min_turn_angle_deg,
                pop_sampler=pop_sampler,
                inv=inv,
                no_fly_hard_union_xy=nofly_hard_union_xy,
                infra_hard_union_xy=infra_hard_union_xy,
                crowd_hard_union_xy=school_hard_union_xy,
                passes=3,
            )
            if _turn_angle_ok(cand):
                nodes = cand
        if len(nodes) < 2:
            return None
        if not _turn_angle_ok(nodes):
            return None
        turns_after_local = polyline_turn_count(nodes, angle_threshold_deg=TURN_IGNORE_DEG)
        min_turn_angle_local = polyline_min_interior_angle(nodes)
        route_line_local = LineString(nodes)
        route_buffer_local = route_line_local.buffer(ROUTE_BUFFER_M)
        crowd_in_buf_local = _count_geoms_intersecting(crowd_tree, route_buffer_local, crowd_points_xy, crowd_id_map)
        key_in_buf_local = _count_geoms_intersecting(key_tree, route_buffer_local, key_points_xy, key_id_map)
        infra_in_buf_local = _count_geoms_intersecting(infra_tree, route_buffer_local, infra_geoms, infra_id_map)
        line_overlap_local = 0.0
        high_build_overlap_local = 0.0
        if line_risk_union_xy is not None and (not line_risk_union_xy.is_empty):
            try:
                line_overlap_local = float(route_line_local.intersection(line_risk_union_xy).length)
            except Exception:
                line_overlap_local = 0.0
        if high_building_union_xy is not None and (not high_building_union_xy.is_empty):
            try:
                high_build_overlap_local = float(route_line_local.intersection(high_building_union_xy).length)
            except Exception:
                high_build_overlap_local = 0.0
        nodes_wgs = []
        for x, y in nodes:
            lon, lat = transform(inv, Point(x, y)).coords[0]
            nodes_wgs.append((float(lon), float(lat)))
        return {
            "id": scheme_id,
            "label": label,
            "profile_key": profile_key,
            "weights": local_weights,
            "route_nodes_xy": nodes,
            "route_nodes_wgs": nodes_wgs,
            "route_line_xy": route_line_local,
            "route_buffer_xy": route_buffer_local,
            "route_cost": chosen_cost,
            "route_cost_base": base_cost,
            "route_cost_water": water_cost,
            "strategy": strategy,
            "turns_before": int(turns_before_local),
            "turns_after": int(turns_after_local),
            "min_turn_angle_deg": round(float(min_turn_angle_local), 2),
            "buffer_metrics": {
                "crowd_points_in_buffer": int(crowd_in_buf_local),
                "key_facility_points_in_buffer": int(key_in_buf_local),
                "critical_infra_geoms_in_buffer": int(infra_in_buf_local),
                "line_risk_overlap_m": round(line_overlap_local, 2),
                "high_building_overlap_m": round(high_build_overlap_local, 2),
            },
            "graph_stats": graph_stats_local,
        }

    candidate_specs = [
        {
            "id": "safety_water",
            "label": "1) 安全优先 + 水路偏好",
            "profile_key": "safest",
            "enable_water_connectors": bool(long_water_available),
            "water_pref_factor": 0.5,
            "allow_water_choice": True,
            "water_detour_limit": 2.0 if long_water_available else 1.18,
            "min_water_share": 0.2,
            "weight_scale": {
                "length": 0.86,
                "population": 1.45,
                "landuse": 1.38,
                "infrastructure": 1.42,
                "altitude": 1.25,
                "turn": 1.2,
                "crowd": 1.5,
                "key_facility": 1.45,
                "line_cross": 1.4,
                "high_building": 1.35,
            },
            "school_penalty_air": 14.0,
            "school_penalty_ground": 10.5,
        },
        {
            "id": "safety_default",
            "label": "2) 安全优先（默认）",
            "profile_key": "safest",
            "enable_water_connectors": False,
            "water_pref_factor": 0.72,
            "allow_water_choice": True,
            "water_detour_limit": 1.18,
            "min_water_share": 0.08,
            "weight_scale": {
                "length": 0.92,
                "population": 1.26,
                "landuse": 1.22,
                "infrastructure": 1.24,
                "altitude": 1.12,
                "turn": 1.08,
                "crowd": 1.28,
                "key_facility": 1.2,
                "line_cross": 1.18,
                "high_building": 1.15,
            },
            "school_penalty_air": 11.0,
            "school_penalty_ground": 8.0,
        },
        {
            "id": "efficiency",
            "label": "3) 效率优先",
            "profile_key": "fastest",
            "enable_water_connectors": False,
            "water_pref_factor": 1.0,
            "allow_water_choice": False,
            "water_detour_limit": 1.35,
            "min_water_share": 0.0,
            "weight_scale": {
                "length": 2.15,
                "population": 0.26,
                "landuse": 0.22,
                "infrastructure": 0.27,
                "altitude": 0.32,
                "turn": 0.42,
                "soft_no_fly": 0.5,
                "crowd": 0.24,
                "key_facility": 0.28,
                "line_cross": 0.3,
                "high_building": 0.38,
            },
            "school_penalty_air": 5.0,
            "school_penalty_ground": 3.8,
        },
    ]

    candidate_results: List[Dict[str, Any]] = []
    candidate_failures: List[Dict[str, Any]] = []
    for spec in candidate_specs:
        failure_reason = ""
        try:
            solved = _solve_scheme(
                scheme_id=str(spec["id"]),
                label=str(spec["label"]),
                profile_key=str(spec["profile_key"]),
                enable_water_connectors=bool(spec["enable_water_connectors"]),
                water_pref_factor=float(spec["water_pref_factor"]),
                allow_water_choice=bool(spec["allow_water_choice"]),
                water_detour_limit=float(spec["water_detour_limit"]),
                min_water_share=float(spec["min_water_share"]),
                weight_scale=dict(spec.get("weight_scale", {})),
                school_penalty_air=float(spec.get("school_penalty_air", 9.0)),
                school_penalty_ground=float(spec.get("school_penalty_ground", 6.5)),
            )
        except Exception as exc:
            solved = None
            failure_reason = str(exc)
        if solved is None:
            candidate_failures.append({"id": spec["id"], "error": failure_reason or "no feasible route"})
            continue
        route_len_m = float(solved["route_line_xy"].length)
        solved["distance_km"] = round(route_len_m / 1000.0, 3)
        solved["detour_ratio"] = round(route_len_m / max(1e-6, direct_dist_m), 3)
        solved["water_share"] = round(
            float(solved["route_cost"].get("water_distance_m", 0.0)) / max(1e-6, float(solved["route_cost"].get("distance_m", 0.0))),
            3,
        )
        candidate_results.append(solved)

    if not candidate_results:
        raise RuntimeError(f"No feasible route found for all candidate schemes: {candidate_failures}")

    terrain_sampler = TerrainSampler(Path(args.dem_tif).resolve() if args.dem_tif else None, args.opentopo_endpoint)
    altitude_ok_results: List[Dict[str, Any]] = []
    for cand in candidate_results:
        try:
            pts_wgs_alt, cand_alt_meta, cand_profile = plan_altitude_profile(
                cand["route_nodes_xy"],
                inv=inv,
                terrain_sampler=terrain_sampler,
                b_geoms=b_geoms,
                b_heights=b_heights,
                b_tree=b_tree,
                b_id_map=b_id_map,
                o_geoms=o_geoms,
                o_heights=o_heights,
                o_tree=o_tree,
                o_id_map=o_id_map,
                speed_ms=float(args.speed_ms),
                climb_ms=float(args.climb_ms),
                descend_ms=float(args.descend_ms),
                clearance_m=float(args.clearance_m),
                preferred_cruise_max_m=float(args.preferred_cruise_max_m),
                hard_ceiling_m=float(args.hard_ceiling_m),
                min_true_height_m=float(args.min_true_height_m),
                max_true_height_m=float(args.max_true_height_m),
                endpoint_true_height_m=float(args.endpoint_true_height_m),
            )
        except Exception as exc:
            candidate_failures.append({"id": cand.get("id", "unknown"), "error": f"altitude_infeasible: {exc}"})
            continue
        cand["altitude_points_wgs_alt"] = pts_wgs_alt
        cand["altitude_meta"] = cand_alt_meta
        cand["altitude_profile_samples"] = cand_profile
        cand["vertical_energy_proxy_m"] = float(cand_alt_meta.get("vertical_energy_proxy_m", 0.0))
        cand["total_climb_m"] = float(cand_alt_meta.get("total_climb_m", 0.0))
        cand["total_descent_m"] = float(cand_alt_meta.get("total_descent_m", 0.0))
        cand["max_true_height_m"] = float(cand_alt_meta.get("max_true_height_m", 0.0))
        altitude_ok_results.append(cand)
    candidate_results = altitude_ok_results
    if not candidate_results:
        raise RuntimeError(f"No feasible route after altitude constraints: {candidate_failures}")

    selected_candidate = None
    for c in candidate_results:
        if c["id"] == args.select_candidate:
            selected_candidate = c
            break
    if selected_candidate is None:
        # Fallback to existing default behavior for robustness.
        for c in candidate_results:
            if c["id"] == "safety_default":
                selected_candidate = c
                break
    if selected_candidate is None:
        selected_candidate = candidate_results[0]

    if args.vertical_tradeoff and len(candidate_results) > 1:
        base = selected_candidate
        base_dist = max(1e-6, float(base.get("distance_km", 0.0)))
        base_energy = max(1e-6, float(base.get("vertical_energy_proxy_m", 0.0)))
        best = base
        best_score = 1.0 + float(args.vertical_energy_weight)
        for cand in candidate_results:
            if cand is base:
                continue
            cand_dist = max(1e-6, float(cand.get("distance_km", 0.0)))
            dist_ratio = cand_dist / base_dist
            if dist_ratio > max(1.0, float(args.vertical_detour_limit_ratio)):
                continue
            cand_energy = max(0.0, float(cand.get("vertical_energy_proxy_m", base_energy)))
            improve = (base_energy - cand_energy) / max(1e-6, base_energy)
            if improve < max(0.0, float(args.vertical_improve_ratio)):
                continue
            score = dist_ratio + float(args.vertical_energy_weight) * (cand_energy / max(1e-6, base_energy))
            if score < best_score:
                best = cand
                best_score = score
        if best is not base:
            best["strategy"] = f"{best.get('strategy', 'base')}+vertical_tradeoff"
            selected_candidate = best

    weights = selected_candidate["weights"]
    graph_stats = selected_candidate["graph_stats"]
    route_nodes = selected_candidate["route_nodes_xy"]
    route_line_xy = selected_candidate["route_line_xy"]
    route_buffer_xy = selected_candidate["route_buffer_xy"]
    route_cost = selected_candidate["route_cost"]
    route_cost_base = selected_candidate["route_cost_base"]
    route_cost_water = selected_candidate["route_cost_water"]
    selected_strategy = selected_candidate["strategy"]
    turns_before = int(selected_candidate["turns_before"])
    turns_after = int(selected_candidate["turns_after"])
    crowd_in_buffer = int(selected_candidate["buffer_metrics"]["crowd_points_in_buffer"])
    key_in_buffer = int(selected_candidate["buffer_metrics"]["key_facility_points_in_buffer"])
    infra_in_buffer = int(selected_candidate["buffer_metrics"]["critical_infra_geoms_in_buffer"])
    line_overlap_m = float(selected_candidate["buffer_metrics"]["line_risk_overlap_m"])
    high_build_overlap_m = float(selected_candidate["buffer_metrics"]["high_building_overlap_m"])
    points_wgs_alt = selected_candidate["altitude_points_wgs_alt"]
    alt_meta = selected_candidate["altitude_meta"]
    profile_samples = selected_candidate["altitude_profile_samples"]

    out_dir = (root / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    base_name = args.name.strip() or "auto_route"
    kml_out = out_dir / f"{base_name}.kml"
    html_out = out_dir / f"{base_name}.html"
    meta_out = out_dir / f"{base_name}_meta.json"
    cand_out = out_dir / f"{base_name}_candidates.json"
    low_risk_landuse_xy, low_risk_counter = load_low_risk_landuse_polygons(summary, fwd)
    candidate_summary = []
    candidate_routes_wgs_for_html: List[Dict[str, Any]] = []
    result_by_id = {str(c["id"]): c for c in candidate_results}
    for spec in candidate_specs:
        cid = str(spec["id"])
        if cid not in result_by_id:
            continue
        cand = result_by_id[cid]
        coords = [(lat, lon) for lon, lat in cand["route_nodes_wgs"]]
        show_flag = (cand["id"] == selected_candidate["id"])
        candidate_routes_wgs_for_html.append(
            {
                "id": cand["id"],
                "label": str(spec["label"]),
                "coords": coords,
                "distance_km": cand["distance_km"],
                "show": show_flag,
            }
        )
        candidate_summary.append(
            {
                "id": cand["id"],
                "label": str(spec["label"]),
                "selected": bool(show_flag),
                "profile_key": cand["profile_key"],
                "distance_km": cand["distance_km"],
                "detour_ratio_vs_direct": cand["detour_ratio"],
                "water_share": cand["water_share"],
                "turns_after_post_smooth": int(cand["turns_after"]),
                "min_turn_angle_deg": float(cand.get("min_turn_angle_deg", 180.0)),
                "vertical_energy_proxy_m": round(float(cand.get("vertical_energy_proxy_m", 0.0)), 2),
                "total_climb_m": round(float(cand.get("total_climb_m", 0.0)), 2),
                "total_descent_m": round(float(cand.get("total_descent_m", 0.0)), 2),
                "max_true_height_m": round(float(cand.get("max_true_height_m", 0.0)), 2),
                "buffer_metrics": cand["buffer_metrics"],
                "route_selection_strategy": cand["strategy"],
            }
        )
    if candidate_failures:
        candidate_summary.append({"failures": candidate_failures})

    write_kml_absolute(kml_out, points_wgs_alt, base_name)
    write_preview_html(
        html_out,
        points_wgs_alt,
        candidate_routes_wgs=candidate_routes_wgs_for_html,
        start_wgs=start_wgs,
        end_wgs=end_wgs,
        nofly_polys_xy=(nofly_hard_polys_xy + nofly_soft_polys_xy),
        route_buffer_xy=route_buffer_xy,
        high_building_polys_xy=high_building_polys_xy,
        crowd_points_xy=crowd_points_xy,
        key_points_xy=key_points_xy,
        infra_geoms_xy=infra_geoms,
        high_road_lines_xy=high_road_lines_xy,
        hsr_lines_xy=hsr_lines_xy,
        line_risk_union_xy=line_risk_union_xy,
        low_risk_landuse_xy=low_risk_landuse_xy,
        school_hard_zones_xy=school_hard_zones_xy,
        school_points_xy=school_points_xy,
        profile_samples=profile_samples,
        inv=inv,
        name=base_name,
    )
    cand_out.write_text(json.dumps(candidate_summary, ensure_ascii=False, indent=2), encoding="utf-8")

    route_len_km = route_line_xy.length / 1000.0
    direct_km = LineString([start_xy, end_xy]).length / 1000.0
    route_pop_avg, route_pop_p90, route_pop_max = _line_population_stats(route_line_xy, pop_sampler, inv)
    meta = {
        "algorithm_version": ALGORITHM_VERSION,
        "city": args.city,
        "name": base_name,
        "profile": selected_candidate["profile_key"],
        "weights": weights,
        "start_wgs84": {"lon": start_wgs[0], "lat": start_wgs[1]},
        "end_wgs84": {"lon": end_wgs[0], "lat": end_wgs[1]},
        "planned_km": round(route_len_km, 3),
        "direct_km": round(direct_km, 3),
        "detour_ratio_vs_direct": round(route_len_km / max(1e-6, direct_km), 3),
        "waypoints_xy": len(route_nodes),
        "open_data_no_fly_enabled": bool(args.open_data_no_fly),
        "no_fly_sources": nofly_counter,
        "network_stats": graph_stats,
        "poi_risk_sources": poi_risk_counter,
        "school_kindergarten_sources": school_counter,
        "line_risk_sources": line_risk_counter,
        "low_risk_landuse_sources": low_risk_counter,
        "water_availability": {
            "long_water_available": bool(long_water_available),
            "water_total_len_km": round(water_total_len_m / 1000.0, 3),
            "nearest_start_water_m": round(water_near_start_m, 2) if math.isfinite(water_near_start_m) else None,
            "nearest_end_water_m": round(water_near_end_m, 2) if math.isfinite(water_near_end_m) else None,
        },
        "path_population_stats": {
            "avg": round(route_pop_avg, 2),
            "p90": round(route_pop_p90, 2),
            "max": round(route_pop_max, 2),
        },
        "infrastructure_features": len(infra_geoms),
        "building_features": obstacle_counter.get("building", 0),
        "obstacle_features": obstacle_counter.get("obstacle", 0),
        "route_cost_breakdown": route_cost,
        "route_selection_strategy": selected_strategy,
        "selected_candidate_id": selected_candidate["id"],
        "selected_candidate_label": selected_candidate["label"],
        "turns_before_post_smooth": int(turns_before),
        "turns_after_post_smooth": int(turns_after),
        "buffer_metrics": {
            "buffer_m": ROUTE_BUFFER_M,
            "crowd_points_in_buffer": int(crowd_in_buffer),
            "key_facility_points_in_buffer": int(key_in_buffer),
            "critical_infra_geoms_in_buffer": int(infra_in_buffer),
            "line_risk_overlap_m": round(line_overlap_m, 2),
            "high_building_overlap_m": round(high_build_overlap_m, 2),
        },
        "route_cost_base": route_cost_base,
        "route_cost_water_priority": route_cost_water,
        "candidate_options": candidate_summary,
        "airframe": {
            "speed_ms": args.speed_ms,
            "climb_ms": args.climb_ms,
            "descend_ms": args.descend_ms,
            "turn_radius_m": args.turn_radius_m,
            "clearance_m": args.clearance_m,
            "endpoint_true_height_m": args.endpoint_true_height_m,
            "min_true_height_m": args.min_true_height_m,
            "max_true_height_m": args.max_true_height_m,
            "preferred_cruise_max_m": args.preferred_cruise_max_m,
            "hard_ceiling_m": args.hard_ceiling_m,
        },
        "route_postprocess": {
            "min_turn_keep_deg": float(args.min_turn_keep_deg),
            "min_turn_angle_deg": float(args.min_turn_angle_deg),
            "turn_prune_passes": int(args.turn_prune_passes),
            "max_waypoints": int(args.max_waypoints),
        },
        "vertical_tradeoff": {
            "enabled": bool(args.vertical_tradeoff),
            "detour_limit_ratio": float(args.vertical_detour_limit_ratio),
            "improve_ratio": float(args.vertical_improve_ratio),
            "energy_weight": float(args.vertical_energy_weight),
            "descend_energy_factor": float(VERTICAL_DESCEND_ENERGY_FACTOR),
            "selected_vertical_energy_proxy_m": round(float(selected_candidate.get("vertical_energy_proxy_m", 0.0)), 2),
            "selected_total_climb_m": round(float(selected_candidate.get("total_climb_m", 0.0)), 2),
            "selected_total_descent_m": round(float(selected_candidate.get("total_descent_m", 0.0)), 2),
            "selected_max_true_height_m": round(float(selected_candidate.get("max_true_height_m", 0.0)), 2),
        },
        "altitude_profile": alt_meta,
        "altitude_profile_samples": profile_samples,
        "outputs": {
            "kml": str(kml_out),
            "preview_html": str(html_out),
            "candidates_json": str(cand_out),
        },
    }
    meta_out.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    review_html = ""
    if args.run_workflow_v2:
        run_cmd(
            [
                "python3",
                str(root / "skills" / "full_assessment_workflow" / "run_workflow_v2.py"),
                "--kml",
                str(kml_out),
                "--out-dir",
                args.workflow_out_dir,
                "--xlsx",
                args.workflow_xlsx,
            ]
        )
        review_html = str((root / args.workflow_out_dir).resolve() / base_name / f"{base_name}_map.html")

    print("DONE")
    print(f"KML: {kml_out}")
    print(f"Preview HTML: {html_out}")
    print(f"Meta: {meta_out}")
    print(f"Candidates: {cand_out}")
    if review_html:
        print(f"Layered HTML: {review_html}")


if __name__ == "__main__":
    main()
