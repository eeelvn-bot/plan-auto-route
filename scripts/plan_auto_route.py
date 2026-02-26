#!/usr/bin/env python3
"""Automatic urban UAV route planner with open-data constraints and altitude profiling."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
from folium.plugins import HeatMap
import pyproj
from osgeo import gdal
from shapely.geometry import LineString, Point, Polygon, shape
from shapely.ops import transform, unary_union
from shapely.strtree import STRtree

try:
    from planner_evidence import build_evidence_pack, write_evidence_pack
except Exception:  # pragma: no cover - optional helper fallback
    build_evidence_pack = None
    write_evidence_pack = None

try:
    from planner_snapshot import write_run_snapshot
except Exception:  # pragma: no cover - optional helper fallback
    write_run_snapshot = None

DEFAULT_OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://lz4.overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]

DEFAULT_OPENTOPO_ENDPOINT = "https://api.opentopodata.org/v1/srtm90m"
ALGORITHM_VERSION = "plan-auto-route-latest-air-corridor-v4-refactor-landuse-softnofly-astarfix"
OVERPASS_CACHE_DIR = Path(__file__).resolve().parents[3] / "output" / "overpass_cache"
DEFAULT_PARETO_POLICY_FILE = Path(__file__).resolve().parents[1] / "config" / "pareto_policies.json"
DEFAULT_CIVIL_AIRPORT_NO_FLY_GEOJSON = Path(__file__).resolve().parents[1] / "config" / "civil_airport_no_fly.geojson"

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
HIGH_VOLTAGE_LINE_MIN_V = 220000.0
HIGH_VOLTAGE_LINE_BUFFER_M = 45.0
WATER_PREF_MIN_FACTOR = 0.18
WATER_PREF_MAX_FACTOR = 1.0
SAFETY_SENSITIVE_HARD_BUFFER_M = 100.0
SAFETY_INFRA_HARD_BUFFER_M = 100.0
CRITICAL_INFRA_MIN_SEVERITY = 2.0
SCHOOL_HARD_BUFFER_M = 100.0
SCHOOL_ENDPOINT_RELIEF_M = 55.0
NO_FLY_SOFT_HELIPORT_BUFFER_M = 700.0
NO_FLY_SOFT_HELIPAD_BUFFER_M = 350.0
NO_FLY_SOFT_MILITARY_BUFFER_M = 600.0
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
        "reference_deviation": 0.0,
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
        "reference_deviation": 0.0,
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
        "reference_deviation": 0.0,
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
    "amenity:square",
    "amenity:cinema",
    "amenity:theatre",
    "amenity:ferry_terminal",
    "tourism:attraction",
    "tourism:museum",
    "tourism:viewpoint",
    "leisure:park",
    "leisure:garden",
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
    "amenity:barracks",
    "landuse:military",
    "military:barracks",
    "military:base",
    "military:airfield",
    "military:naval_base",
}


@dataclass(frozen=True)
class CandidateSpec:
    id: str
    label: str
    profile_key: str
    enable_water_connectors: bool
    water_pref_factor: float
    allow_water_choice: bool
    water_detour_limit: float
    min_water_share: float
    weight_scale: Dict[str, float]
    school_penalty_air: float
    school_penalty_ground: float
    enable_sensitive_hard_constraint: bool = False
    enable_infra_hard_constraint: bool = False


def _normalize_pareto_policy(raw: Dict[str, Any]) -> Dict[str, float]:
    out = {
        "detour_limit_ratio": 1.25,
        "distance_weight": 1.0,
        "population_weight": 1.0,
        "energy_weight": 1.25,
    }
    for key in out:
        val = raw.get(key, out[key])
        try:
            out[key] = float(val)
        except Exception:
            continue
    out["detour_limit_ratio"] = max(1.0, out["detour_limit_ratio"])
    out["distance_weight"] = max(0.05, out["distance_weight"])
    out["population_weight"] = max(0.05, out["population_weight"])
    out["energy_weight"] = max(0.05, out["energy_weight"])
    return out


def _merge_pareto_policy(base: Dict[str, float], override: Dict[str, Any]) -> Dict[str, float]:
    merged: Dict[str, Any] = dict(base)
    for key in ["detour_limit_ratio", "distance_weight", "population_weight", "energy_weight"]:
        if key in override:
            merged[key] = override[key]
    return _normalize_pareto_policy(merged)


def _load_json_file(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return obj if isinstance(obj, dict) else {}


def resolve_pareto_policy(
    *,
    policy_file: str,
    policy_name: str,
    city: str,
    profile: str,
    business_line: str,
    cli_detour_limit_ratio: Optional[float],
    cli_distance_weight: Optional[float],
    cli_population_weight: Optional[float],
    cli_energy_weight: Optional[float],
    fallback_energy_weight: float,
) -> Tuple[Dict[str, float], Dict[str, Any]]:
    active = _normalize_pareto_policy({"energy_weight": fallback_energy_weight})
    trace: Dict[str, Any] = {
        "policy_file": "",
        "policy_name": policy_name,
        "business_line": business_line,
        "applied_layers": ["builtin_default"],
    }
    policy_path = Path(policy_file).resolve() if policy_file else DEFAULT_PARETO_POLICY_FILE
    cfg = _load_json_file(policy_path)
    if cfg:
        trace["policy_file"] = str(policy_path)
        policies = cfg.get("policies", {}) if isinstance(cfg.get("policies", {}), dict) else {}
        if policy_name in policies and isinstance(policies[policy_name], dict):
            active = _merge_pareto_policy(active, policies[policy_name])
            trace["applied_layers"].append(f"policy:{policy_name}")

        def _apply_group(group_name: str, key: str) -> None:
            nonlocal active
            group = cfg.get(group_name, {})
            if (not isinstance(group, dict)) or (key not in group):
                return
            rule = group.get(key, {})
            if not isinstance(rule, dict):
                return
            base_policy = str(rule.get("base_policy", "")).strip()
            if base_policy and base_policy in policies and isinstance(policies[base_policy], dict):
                active = _merge_pareto_policy(active, policies[base_policy])
                trace["applied_layers"].append(f"{group_name}:{key}:base_policy:{base_policy}")
            weights = rule.get("weights", {})
            if isinstance(weights, dict):
                active = _merge_pareto_policy(active, weights)
                trace["applied_layers"].append(f"{group_name}:{key}:weights")
            else:
                active = _merge_pareto_policy(active, rule)
                trace["applied_layers"].append(f"{group_name}:{key}")

        if business_line:
            _apply_group("business_overrides", business_line)
        if city:
            _apply_group("city_overrides", city)
        if profile:
            _apply_group("profile_overrides", profile)

    cli_override: Dict[str, Any] = {}
    if cli_detour_limit_ratio is not None:
        cli_override["detour_limit_ratio"] = cli_detour_limit_ratio
    if cli_distance_weight is not None:
        cli_override["distance_weight"] = cli_distance_weight
    if cli_population_weight is not None:
        cli_override["population_weight"] = cli_population_weight
    if cli_energy_weight is not None:
        cli_override["energy_weight"] = cli_energy_weight
    if cli_override:
        active = _merge_pareto_policy(active, cli_override)
        trace["applied_layers"].append("cli_override")

    return active, trace


def _blend_weight_scales(
    left: Dict[str, float],
    right: Dict[str, float],
    t: float,
) -> Dict[str, float]:
    ratio = max(0.0, min(1.0, float(t)))
    keys = set(left.keys()) | set(right.keys()) | set(WEIGHT_PROFILES["balanced"].keys())
    out: Dict[str, float] = {}
    for key in keys:
        lv = float(left.get(key, 1.0))
        rv = float(right.get(key, 1.0))
        out[key] = max(0.05, lv * (1.0 - ratio) + rv * ratio)
    return out


def build_sweep_candidate_specs(
    levels: int,
    *,
    profile_key: str,
    long_water_available: bool,
    safety_scale: Dict[str, float],
    efficiency_scale: Dict[str, float],
    safety_school_air: float,
    safety_school_ground: float,
    efficiency_school_air: float,
    efficiency_school_ground: float,
) -> List[CandidateSpec]:
    count = max(0, int(levels))
    if count <= 0:
        return []
    out: List[CandidateSpec] = []
    for idx in range(1, count + 1):
        t = float(idx) / float(count + 1)
        scale = _blend_weight_scales(safety_scale, efficiency_scale, t)
        out.append(
            CandidateSpec(
                id=f"sweep_{idx:02d}",
                label=f"Sweep {idx}/{count}",
                profile_key=profile_key,
                enable_water_connectors=bool(long_water_available and t < 0.6),
                water_pref_factor=max(0.6, min(1.0, 0.72 + 0.28 * t)),
                allow_water_choice=bool(t < 0.75),
                water_detour_limit=1.18 + 0.17 * t,
                min_water_share=max(0.0, 0.08 * (1.0 - t)),
                weight_scale=scale,
                school_penalty_air=safety_school_air * (1.0 - t) + efficiency_school_air * t,
                school_penalty_ground=safety_school_ground * (1.0 - t) + efficiency_school_ground * t,
            )
        )
    return out


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def pareto_front_candidates(
    candidates: List[Dict[str, Any]],
    *,
    metric_keys: List[str],
) -> List[Dict[str, Any]]:
    if not candidates:
        return []
    eps = 1e-9
    front: List[Dict[str, Any]] = []
    for i, cand in enumerate(candidates):
        dominated = False
        for j, other in enumerate(candidates):
            if i == j:
                continue
            all_le = True
            any_lt = False
            for key in metric_keys:
                v_other = _safe_float(other.get(key, float("inf")), float("inf"))
                v_cand = _safe_float(cand.get(key, float("inf")), float("inf"))
                if v_other > v_cand + eps:
                    all_le = False
                    break
                if v_other + eps < v_cand:
                    any_lt = True
            if all_le and any_lt:
                dominated = True
                break
        if not dominated:
            front.append(cand)
    return front


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


def wgs_to_xy(fwd, lon: float, lat: float) -> Tuple[float, float]:
    x, y = fwd(float(lon), float(lat))
    return float(x), float(y)


def xy_to_wgs(inv, x: float, y: float) -> Tuple[float, float]:
    lon, lat = inv(float(x), float(y))
    return float(lon), float(lat)


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


def _max_voltage_v(raw_value: Any) -> Optional[float]:
    if raw_value is None:
        return None
    text = str(raw_value).strip().lower()
    if not text:
        return None
    nums = re.findall(r"[-+]?\d+(?:\.\d+)?", text)
    values: List[float] = []
    for token in nums:
        try:
            values.append(float(token))
        except Exception:
            continue
    if not values:
        return None
    max_v = max(values)
    if "kv" in text and max_v < 10000.0:
        max_v *= 1000.0
    if "mv" in text and max_v < 1000.0:
        max_v *= 1_000_000.0
    return max_v


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


def _poi_name_prefer_zh(props: Dict[str, Any], tags: Dict[str, Any]) -> str:
    candidates: List[str] = []
    for key in ("name:zh", "name:zh-Hans", "name:zh_CN", "name:zh-Hant", "name"):
        val = tags.get(key)
        if val is not None:
            text = str(val).strip()
            if text:
                candidates.append(text)
    for key in ("name_zh", "name"):
        val = props.get(key)
        if val is not None:
            text = str(val).strip()
            if text:
                candidates.append(text)
    return candidates[0] if candidates else "未命名POI"


def _poi_type_label(poi_type: str) -> str:
    token = str(poi_type or "").strip().lower()
    labels = {
        "amenity:school": "学校",
        "amenity:kindergarten": "幼儿园",
        "amenity:college": "学院",
        "amenity:university": "大学",
        "amenity:hospital": "医院",
        "amenity:clinic": "诊所",
        "amenity:bus_station": "公交枢纽",
        "amenity:marketplace": "集市",
        "amenity:place_of_worship": "宗教场所",
        "amenity:square": "广场",
        "amenity:cinema": "影院",
        "amenity:theatre": "剧院",
        "amenity:townhall": "政府机构",
        "amenity:courthouse": "法院",
        "amenity:police": "警务设施",
        "amenity:fire_station": "消防设施",
        "office:government": "政府办公机构",
        "shop:mall": "商业中心",
        "shop:supermarket": "商超",
        "leisure:park": "公园",
        "leisure:garden": "园林",
        "landuse:military": "军事用地",
    }
    if token in labels:
        return labels[token]
    if token.startswith("military:"):
        return "军事设施"
    if token.startswith("amenity:"):
        return "公共服务设施"
    if token.startswith("office:"):
        return "办公设施"
    if token.startswith("shop:"):
        return "商业设施"
    if token.startswith("leisure:"):
        return "休闲设施"
    if token.startswith("landuse:"):
        return "用地设施"
    return token or "未分类"


def _build_poi_tooltip_lookup(items: List[Dict[str, Any]], snap_m: float = 8.0) -> Dict[Tuple[float, float], str]:
    grouped: Dict[Tuple[float, float], List[str]] = defaultdict(list)
    for item in items:
        pt = item.get("point_xy")
        if pt is None or getattr(pt, "is_empty", True):
            continue
        try:
            key = _round_key((float(pt.x), float(pt.y)), snap_m=snap_m)
        except Exception:
            continue
        name = str(item.get("name", "")).strip() or "未命名POI"
        type_text = str(item.get("type", "")).strip() or "未分类"
        source = str(item.get("source", "")).strip()
        text = f"名称: {name} | 类型: {type_text}"
        if source:
            text += f" | 来源: {source}"
        grouped[key].append(text)

    out: Dict[Tuple[float, float], str] = {}
    for key, values in grouped.items():
        uniq: List[str] = []
        for val in values:
            if val not in uniq:
                uniq.append(val)
        if not uniq:
            continue
        if len(uniq) == 1:
            out[key] = uniq[0]
        else:
            out[key] = f"{uniq[0]} | 等{len(uniq)}项"
    return out


def build_poi_risk_indices(
    summary: Dict[str, Any],
    fwd,
) -> Tuple[
    List[Any],
    Optional[STRtree],
    Dict[int, int],
    List[Any],
    Optional[STRtree],
    Dict[int, int],
    Dict[str, int],
    List[Dict[str, Any]],
    List[Dict[str, Any]],
]:
    outputs = summary.get("outputs", {})
    poi_geojson = Path(outputs.get("poi", {}).get("geojson", ""))
    crowd_points_xy: List[Any] = []
    key_points_xy: List[Any] = []
    crowd_poi_items: List[Dict[str, Any]] = []
    key_poi_items: List[Dict[str, Any]] = []
    counter = {"crowd": 0, "sensitive_facility": 0, "key_facility": 0}
    for feat in load_geojson_features(poi_geojson):
        props = feat.get("properties") or {}
        poi_type = str(props.get("poi_type", "")).strip().lower()
        raw_tags = _parse_raw_tags(props.get("raw_tags"))
        if not poi_type:
            candidates = []
            for k in ("amenity", "office", "shop", "tourism", "leisure", "place", "landuse"):
                if raw_tags.get(k):
                    candidates.append(f"{k}:{str(raw_tags.get(k)).strip().lower()}")
            if candidates:
                poi_type = candidates[0]
        is_crowd = poi_type in CROWD_POI_TYPES
        is_key = poi_type in KEY_FACILITY_POI_TYPES
        if (not is_key) and poi_type.startswith("military:"):
            is_key = True
        if (not is_crowd) and (not is_key):
            office = str(raw_tags.get("office", "")).strip().lower()
            amenity = str(raw_tags.get("amenity", "")).strip().lower()
            military = str(raw_tags.get("military", "")).strip().lower()
            landuse = str(raw_tags.get("landuse", "")).strip().lower()
            leisure = str(raw_tags.get("leisure", "")).strip().lower()
            place = str(raw_tags.get("place", "")).strip().lower()
            shop = str(raw_tags.get("shop", "")).strip().lower()
            if office == "government":
                is_key = True
            if military or landuse == "military" or amenity == "barracks":
                is_key = True
            if amenity in {"hospital", "school", "kindergarten", "bus_station", "college", "university", "square"}:
                is_crowd = True
            if leisure in {"park", "garden"}:
                is_crowd = True
            if place in {"square"}:
                is_crowd = True
            if shop in {"mall", "supermarket"}:
                is_crowd = True
        if not is_crowd and not is_key:
            continue
        pt_wgs = _point_from_geojson_geometry(feat.get("geometry") or {})
        if pt_wgs is None:
            continue
        pt_xy = _to_xy_geometry(pt_wgs, fwd)
        if pt_xy is None:
            continue
        poi_name = _poi_name_prefer_zh(props, raw_tags)
        poi_type_label = _poi_type_label(poi_type)
        poi_id = str(props.get("id", "")).strip()
        if is_crowd:
            crowd_points_xy.append(pt_xy)
            crowd_poi_items.append(
                {
                    "point_xy": pt_xy,
                    "name": poi_name,
                    "type": poi_type_label,
                    "source": "POI",
                    "id": poi_id,
                }
            )
            counter["crowd"] += 1
        if is_key:
            key_points_xy.append(pt_xy)
            key_poi_items.append(
                {
                    "point_xy": pt_xy,
                    "name": poi_name,
                    "type": poi_type_label,
                    "source": "POI",
                    "id": poi_id,
                }
            )
            counter["sensitive_facility"] += 1
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
        crowd_poi_items,
        key_poi_items,
    )


def build_line_risk_geometries(
    summary: Dict[str, Any],
    route_bbox: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], Any, List[Any], List[Any], Dict[str, int]]:
    outputs = summary.get("outputs", {})
    roads_geojson = Path(outputs.get("transport", {}).get("roads_geojson", ""))
    hsr_geojson = Path(outputs.get("transport", {}).get("hsr_geojson", ""))
    road_lines_xy: List[Any] = []
    hsr_lines_xy: List[Any] = []
    hv_power_lines_xy: List[Any] = []
    c_road = 0
    c_hsr = 0
    c_hv_power = 0
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
  way({south},{west},{north},{east})[power~"^(line|minor_line)$"][voltage];
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
        power = str(tags.get("power", "")).strip().lower()
        if hwy in high_road_types:
            road_lines_xy.append(g_xy)
            c_road += 1
        elif railway == "rail" and highspeed in {"yes", "designated"}:
            hsr_lines_xy.append(g_xy)
            c_hsr += 1
        elif power in {"line", "minor_line"}:
            voltage_v = _max_voltage_v(tags.get("voltage"))
            if voltage_v is not None and voltage_v >= HIGH_VOLTAGE_LINE_MIN_V:
                hv_power_lines_xy.append(g_xy)
                c_hv_power += 1

    line_risk_geoms = (
        [g.buffer(35.0) for g in road_lines_xy]
        + [g.buffer(30.0) for g in hsr_lines_xy]
        + [g.buffer(HIGH_VOLTAGE_LINE_BUFFER_M) for g in hv_power_lines_xy]
    )
    line_risk_union = unary_union(line_risk_geoms) if line_risk_geoms else None
    return road_lines_xy, line_risk_union, hsr_lines_xy, hv_power_lines_xy, {
        "highway": c_road,
        "hsr": c_hsr,
        "high_voltage_power_line": c_hv_power,
    }


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


def build_population_density_samples(
    pop_sampler: PopulationSampler,
    bbox_wgs: Tuple[float, float, float, float],
    max_points: int = 1200,
) -> List[Dict[str, float]]:
    if pop_sampler.data is None or pop_sampler.gt is None or pop_sampler.width <= 0 or pop_sampler.height <= 0:
        return []
    south, north, west, east = bbox_wgs
    gt = pop_sampler.gt
    px1 = (west - gt[0]) / gt[1]
    px2 = (east - gt[0]) / gt[1]
    py1 = (south - gt[3]) / gt[5]
    py2 = (north - gt[3]) / gt[5]
    min_px = max(0, int(math.floor(min(px1, px2))))
    max_px = min(pop_sampler.width - 1, int(math.ceil(max(px1, px2))))
    min_py = max(0, int(math.floor(min(py1, py2))))
    max_py = min(pop_sampler.height - 1, int(math.ceil(max(py1, py2))))
    if min_px > max_px or min_py > max_py:
        return []
    span_w = max_px - min_px + 1
    span_h = max_py - min_py + 1
    target = max(1, int(max_points))
    step = max(1, int(math.sqrt((span_w * span_h) / max(1, target))))
    out: List[Dict[str, float]] = []
    for py in range(min_py, max_py + 1, step):
        lat = gt[3] + (py + 0.5) * gt[5]
        for px in range(min_px, max_px + 1, step):
            val = float(pop_sampler.data[py][px])
            if pop_sampler.nodata is not None and val == pop_sampler.nodata:
                continue
            if (not math.isfinite(val)) or val <= 0.0:
                continue
            lon = gt[0] + (px + 0.5) * gt[1]
            out.append({"lon": float(lon), "lat": float(lat), "value": float(val)})
    if len(out) > target:
        stride = max(1, int(math.ceil(len(out) / target)))
        out = out[::stride]
    return out[:target]


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


def _dedupe_point_geometries(points_xy: List[Any], snap_m: float = 8.0) -> List[Any]:
    out: List[Any] = []
    seen: set[Tuple[float, float]] = set()
    for pt in points_xy:
        if pt is None or getattr(pt, "is_empty", True):
            continue
        try:
            x = float(pt.x)
            y = float(pt.y)
        except Exception:
            continue
        key = _round_key((x, y), snap_m=snap_m)
        if key in seen:
            continue
        seen.add(key)
        out.append(Point(x, y))
    return out


def _buffer_union_from_points(points_xy: List[Any], buffer_m: float) -> Optional[Any]:
    if buffer_m <= 0.0 or not points_xy:
        return None
    geoms: List[Any] = []
    for pt in points_xy:
        try:
            geoms.append(pt.buffer(float(buffer_m)))
        except Exception:
            continue
    if not geoms:
        return None
    union_geom = unary_union(geoms)
    if union_geom is None or union_geom.is_empty:
        return None
    return union_geom


def _item_to_index(item: Any, geoms: List[Any], geom_id_map: Dict[int, int]) -> Optional[int]:
    if isinstance(item, numbers.Integral):
        idx = int(item)
        if 0 <= idx < len(geoms):
            return idx
        return None
    idx = geom_id_map.get(id(item))
    if idx is not None:
        return idx
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
    point_distance_only: bool = False,
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
    if point_distance_only:
        try:
            return idx, math.hypot(float(geom.x) - float(geoms[idx].x), float(geom.y) - float(geoms[idx].y))
        except Exception:
            pass
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
    aeroway = str(tags.get("aeroway", "")).strip().lower()
    military = str(tags.get("military", "")).strip().lower()
    if hard:
        if aeroway in {"aerodrome", "airport"}:
            return 2000.0
        if military in {"airfield", "air_base", "airbase", "naval_air_station"}:
            return 1600.0
        return 0.0
    if aeroway == "heliport":
        return NO_FLY_SOFT_HELIPORT_BUFFER_M
    if aeroway == "helipad":
        return NO_FLY_SOFT_HELIPAD_BUFFER_M
    if military:
        return NO_FLY_SOFT_MILITARY_BUFFER_M
    return 0.0


def _is_hard_no_fly(tags: Dict[str, Any]) -> bool:
    aeroway = str(tags.get("aeroway", "")).strip().lower()
    military = str(tags.get("military", "")).strip().lower()
    if aeroway in {"aerodrome", "airport"}:
        return True
    if military in {"airfield", "air_base", "airbase", "naval_air_station"}:
        return True
    return False


def load_civil_airport_no_fly_zones(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
    dataset_geojson_path: str,
) -> Tuple[List[Any], Dict[str, Any]]:
    out: List[Any] = []
    stats: Dict[str, Any] = {
        "dataset_path": "",
        "dataset_features_loaded": 0,
        "dataset_features_intersected": 0,
        "dataset_airports_intersected": 0,
    }
    path = Path(dataset_geojson_path).resolve() if dataset_geojson_path else DEFAULT_CIVIL_AIRPORT_NO_FLY_GEOJSON
    stats["dataset_path"] = str(path)
    if not path.exists():
        return out, stats
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return out, stats
    features = obj.get("features", []) if isinstance(obj, dict) else []
    if not isinstance(features, list):
        return out, stats
    stats["dataset_features_loaded"] = len(features)
    south, north, west, east = bbox_wgs
    bbox_poly_wgs = Polygon([(west, south), (east, south), (east, north), (west, north), (west, south)])
    by_airport: Dict[str, List[Any]] = defaultdict(list)
    feature_hits = 0
    for idx, feat in enumerate(features):
        if not isinstance(feat, dict):
            continue
        geom_obj = feat.get("geometry") or {}
        props = feat.get("properties") or {}
        airport_code = str(props.get("icao", "")).strip().upper() or f"feature_{idx + 1}"
        try:
            g_wgs = shape(geom_obj)
        except Exception:
            continue
        if g_wgs.is_empty:
            continue
        try:
            if not g_wgs.is_valid:
                g_wgs = g_wgs.buffer(0)
        except Exception:
            continue
        if g_wgs.is_empty or (not g_wgs.intersects(bbox_poly_wgs)):
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        if g_xy.geom_type not in {"Polygon", "MultiPolygon"}:
            continue
        by_airport[airport_code].append(g_xy)
        feature_hits += 1
    for airport_code, geoms in by_airport.items():
        try:
            merged = unary_union(geoms)
        except Exception:
            continue
        if merged.is_empty:
            continue
        if merged.geom_type in {"Polygon", "MultiPolygon"}:
            out.append(merged)
    stats["dataset_features_intersected"] = feature_hits
    stats["dataset_airports_intersected"] = len(out)
    return out, stats


def _normalize_city_token(name: str) -> str:
    token = str(name or "").strip()
    if not token:
        return ""
    token = re.split(r"[_/\\s]", token, maxsplit=1)[0].strip()
    for suffix in ("市", "地区", "自治州", "盟", "州"):
        if token.endswith(suffix) and len(token) > len(suffix):
            token = token[: -len(suffix)]
            break
    return token


def _extract_airport_city_tokens(airport_name: str) -> List[str]:
    name = str(airport_name or "").strip()
    if not name:
        return []
    raw_tokens = [x.strip() for x in re.split(r"[\\/、,，\\-\\s]+", name) if x.strip()]
    out: List[str] = []
    for t in raw_tokens:
        nt = _normalize_city_token(t)
        if nt:
            out.append(nt)
    return out


def load_civil_airport_no_fly_zones_by_city(
    city_name: str,
    fwd,
    dataset_geojson_path: str,
) -> Tuple[List[Any], Dict[str, Any]]:
    out: List[Any] = []
    stats: Dict[str, Any] = {
        "dataset_path": "",
        "dataset_features_loaded": 0,
        "dataset_features_matched_city": 0,
        "dataset_airports_matched_city": 0,
        "city_name_input": city_name,
        "city_token": "",
    }
    city_token = _normalize_city_token(city_name)
    stats["city_token"] = city_token
    path = Path(dataset_geojson_path).resolve() if dataset_geojson_path else DEFAULT_CIVIL_AIRPORT_NO_FLY_GEOJSON
    stats["dataset_path"] = str(path)
    if not city_token or (not path.exists()):
        return out, stats
    try:
        obj = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return out, stats
    features = obj.get("features", []) if isinstance(obj, dict) else []
    if not isinstance(features, list):
        return out, stats
    stats["dataset_features_loaded"] = len(features)
    by_airport: Dict[str, List[Any]] = defaultdict(list)
    feature_hits = 0
    for idx, feat in enumerate(features):
        if not isinstance(feat, dict):
            continue
        geom_obj = feat.get("geometry") or {}
        props = feat.get("properties") or {}
        airport_name = str(props.get("airport_name", "")).strip()
        airport_tokens = _extract_airport_city_tokens(airport_name)
        if not airport_tokens:
            continue
        if not any((city_token in t) or (t in city_token) for t in airport_tokens):
            continue
        airport_code = str(props.get("icao", "")).strip().upper() or f"feature_{idx + 1}"
        try:
            g_wgs = shape(geom_obj)
        except Exception:
            continue
        if g_wgs.is_empty:
            continue
        try:
            if not g_wgs.is_valid:
                g_wgs = g_wgs.buffer(0)
        except Exception:
            continue
        if g_wgs.is_empty:
            continue
        g_xy = _to_xy_geometry(g_wgs, fwd)
        if g_xy is None:
            continue
        if g_xy.geom_type not in {"Polygon", "MultiPolygon"}:
            continue
        by_airport[airport_code].append(g_xy)
        feature_hits += 1
    for _, geoms in by_airport.items():
        try:
            merged = unary_union(geoms)
        except Exception:
            continue
        if merged.is_empty:
            continue
        if merged.geom_type in {"Polygon", "MultiPolygon"}:
            out.append(merged)
    stats["dataset_features_matched_city"] = feature_hits
    stats["dataset_airports_matched_city"] = len(out)
    return out, stats


def fetch_open_data_no_fly_zones(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
    civil_airport_geojson: str,
) -> Tuple[List[Any], List[Any], Dict[str, Any], List[Any], List[Any], List[Any]]:
    south, north, west, east = bbox_wgs
    civil_hard_polygons_xy, civil_stats = load_civil_airport_no_fly_zones(
        bbox_wgs,
        fwd,
        dataset_geojson_path=civil_airport_geojson,
    )
    hard_polygons_xy = list(civil_hard_polygons_xy)
    query = f"""
[out:json][timeout:120];
(
  nwr({south},{west},{north},{east})[aeroway~"^(heliport|helipad)$"];
  nwr({south},{west},{north},{east})[military];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        return hard_polygons_xy, [], {
            "civil_airport": int(civil_stats.get("dataset_airports_intersected", 0)),
            "military_airport": 0,
            "hard": len(hard_polygons_xy),
            "soft": 0,
            "civil_source": "xls_dataset",
            "civil_dataset": civil_stats,
        }, civil_hard_polygons_xy, [], []
    soft_polygons_xy: List[Any] = []
    military_hard_polygons_xy: List[Any] = []
    heli_soft_polygons_xy: List[Any] = []
    counter: Dict[str, Any] = {
        "civil_airport": int(civil_stats.get("dataset_airports_intersected", 0)),
        "military_airport": 0,
        "hard": len(hard_polygons_xy),
        "soft": 0,
        "civil_source": "xls_dataset",
        "civil_dataset": civil_stats,
    }
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
        military = str(tags.get("military", "")).strip().lower()
        if military in {"airfield", "air_base", "airbase", "naval_air_station"}:
            counter["military_airport"] += 1
        try:
            minx, miny, maxx, maxy = g_xy.bounds
            if (maxx - minx) > max_span_m or (maxy - miny) > max_span_m:
                continue
            is_hard = _is_hard_no_fly(tags)
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
                military_hard_polygons_xy.append(zone)
                counter["hard"] += 1
            else:
                soft_polygons_xy.append(zone)
                aeroway = str(tags.get("aeroway", "")).strip().lower()
                if aeroway in {"heliport", "helipad"}:
                    heli_soft_polygons_xy.append(zone)
                counter["soft"] += 1
        except Exception:
            continue
    counter["heli_soft"] = len(heli_soft_polygons_xy)
    return hard_polygons_xy, soft_polygons_xy, counter, civil_hard_polygons_xy, military_hard_polygons_xy, heli_soft_polygons_xy


def fetch_school_kindergarten_zones(
    bbox_wgs: Tuple[float, float, float, float],
    fwd,
) -> Tuple[List[Any], List[Any], Dict[str, int], List[Dict[str, Any]]]:
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
        return [], [], {"school": 0, "kindergarten": 0}, []
    hard_zones_xy: List[Any] = []
    points_xy: List[Any] = []
    school_poi_items: List[Dict[str, Any]] = []
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
                pt = g_xy
            elif g_xy.geom_type in {"Polygon", "MultiPolygon"}:
                zone = g_xy.buffer(18.0)
                pt = g_xy.representative_point()
            else:
                zone = g_xy.buffer(26.0)
                pt = g_xy.representative_point()
            points_xy.append(pt)
            hard_zones_xy.append(zone)
            school_poi_items.append(
                {
                    "point_xy": pt,
                    "name": _poi_name_prefer_zh({}, tags),
                    "type": "学校" if amenity == "school" else "幼儿园",
                    "source": "学校避让",
                }
            )
        except Exception:
            continue
    return hard_zones_xy, points_xy, counter, school_poi_items


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
                x, y = wgs_to_xy(fwd, float(lon), float(lat))
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
  nwr({south},{west},{north},{east})[man_made~"^(storage_tank|works|pipeline|water_works|wastewater_plant|pumping_station)$"];
  nwr({south},{west},{north},{east})[waterway=dam];
);
out geom tags;
""".strip()
    try:
        data = run_overpass(query, timeout=120)
    except Exception:
        data = {"elements": []}

    def _hazard_hint(tags: Dict[str, Any]) -> bool:
        # P1 hazard proxy from common OSM fields around industrial energy assets.
        text = " ".join(
            [
                str(tags.get("content", "")).lower(),
                str(tags.get("substance", "")).lower(),
                str(tags.get("product", "")).lower(),
                str(tags.get("industrial", "")).lower(),
                str(tags.get("hazard", "")).lower(),
                str(tags.get("hazard_type", "")).lower(),
            ]
        )
        if not text.strip():
            return False
        keys = (
            "fuel",
            "gas",
            "lng",
            "lpg",
            "oil",
            "petrol",
            "petroleum",
            "diesel",
            "gasoline",
            "kerosene",
            "chemical",
            "chemicals",
            "petrochemical",
            "refinery",
            "ammonia",
            "hydrogen",
            "toxic",
            "hazardous",
        )
        return any(k in text for k in keys)

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
        waterway = str(tags.get("waterway", "")).lower()
        hazard_hint = _hazard_hint(tags)
        sev = 1.3
        point_buffer_m = 20.0
        line_buffer_m = 8.0
        # P1 critical infrastructure categories.
        if man_made == "storage_tank":
            sev = 2.9 if hazard_hint else 2.4
            point_buffer_m = 26.0
            line_buffer_m = 14.0
        elif man_made == "pipeline":
            sev = 2.7 if hazard_hint else 2.2
            point_buffer_m = 24.0
            line_buffer_m = 16.0
        elif man_made == "works":
            sev = 2.6 if hazard_hint else 2.2
            point_buffer_m = 24.0
            line_buffer_m = 14.0
        # P2 critical infrastructure categories.
        elif man_made in {"water_works", "wastewater_plant", "pumping_station"}:
            sev = 2.0
            point_buffer_m = 24.0
            line_buffer_m = 12.0
        elif waterway == "dam":
            sev = 2.2
            point_buffer_m = 28.0
            line_buffer_m = 18.0
        if power in {"plant", "substation"}:
            sev = max(sev, 2.2)
            point_buffer_m = max(point_buffer_m, 24.0)
            line_buffer_m = max(line_buffer_m, 12.0)
        elif power in {"tower", "line"}:
            sev = max(sev, 1.8)
        elif man_made in {"tower", "communications_tower", "chimney"}:
            sev = max(sev, 1.8)
        if g_xy.geom_type == "Point":
            g2 = g_xy.buffer(point_buffer_m)
        elif g_xy.geom_type in {"Polygon", "MultiPolygon"}:
            g2 = g_xy
        else:
            g2 = g_xy.buffer(line_buffer_m)
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
    point_geom: Optional[Any] = None,
) -> float:
    if tree is None:
        return 1.0
    point = point_geom if point_geom is not None else Point(pt_xy)
    candidates = _tree_candidate_indices(tree, point, geoms, geom_id_map)
    best: Optional[float] = None
    for idx in candidates:
        g = geoms[idx]
        if not g.intersects(point):
            continue
        if best is None:
            best = costs[idx]
        else:
            best = min(best, costs[idx])
    return 1.0 if best is None else float(best)


def infrastructure_penalty_at_point(
    pt_xy: Tuple[float, float],
    geoms: List[Any],
    severities: List[float],
    tree: Optional[STRtree],
    geom_id_map: Dict[int, int],
    point_geom: Optional[Any] = None,
    nearest_cache: Optional[Dict[Tuple[float, float], Tuple[Optional[int], float]]] = None,
    nearest_cache_snap_m: float = 1.0,
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = point_geom if point_geom is not None else Point(pt_xy)
    idx: Optional[int] = None
    dist = float("inf")
    cache_key: Optional[Tuple[float, float]] = None
    if nearest_cache is not None:
        cache_key = _round_key(pt_xy, snap_m=nearest_cache_snap_m)
        cached = nearest_cache.get(cache_key)
        if cached is not None:
            idx, dist = cached
    if idx is None and not math.isfinite(dist):
        idx, dist = _nearest_index_and_distance(tree, point, geoms, geom_id_map)
        if nearest_cache is not None and cache_key is not None:
            nearest_cache[cache_key] = (idx, dist)
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
    point_geom: Optional[Any] = None,
) -> float:
    if tree is None:
        return 0.0
    point = point_geom if point_geom is not None else Point(pt_xy)
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
    point_geom: Optional[Any] = None,
    nearest_cache: Optional[Dict[Tuple[float, float], Tuple[Optional[int], float]]] = None,
    nearest_cache_snap_m: float = 1.0,
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = point_geom if point_geom is not None else Point(pt_xy)
    idx: Optional[int] = None
    dist = float("inf")
    cache_key: Optional[Tuple[float, float]] = None
    if nearest_cache is not None:
        cache_key = _round_key(pt_xy, snap_m=nearest_cache_snap_m)
        cached = nearest_cache.get(cache_key)
        if cached is not None:
            idx, dist = cached
    if idx is None and not math.isfinite(dist):
        idx, dist = _nearest_index_and_distance(tree, point, geoms, geom_id_map)
        if nearest_cache is not None and cache_key is not None:
            nearest_cache[cache_key] = (idx, dist)
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
    point_geom: Optional[Any] = None,
    nearest_cache: Optional[Dict[Tuple[float, float], Tuple[Optional[int], float]]] = None,
    nearest_cache_snap_m: float = 1.0,
) -> float:
    if tree is None or not geoms:
        return 0.0
    point = point_geom if point_geom is not None else Point(pt_xy)
    idx: Optional[int] = None
    dist = float("inf")
    cache_key: Optional[Tuple[float, float]] = None
    if nearest_cache is not None:
        cache_key = _round_key(pt_xy, snap_m=nearest_cache_snap_m)
        cached = nearest_cache.get(cache_key)
        if cached is not None:
            idx, dist = cached
    if idx is None and not math.isfinite(dist):
        idx, dist = _nearest_index_and_distance(
            tree,
            point,
            geoms,
            geom_id_map,
            point_distance_only=True,
        )
        if nearest_cache is not None and cache_key is not None:
            nearest_cache[cache_key] = (idx, dist)
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


def _turn_penalty_for_vectors(
    prev_vec: Tuple[float, float],
    next_vec: Tuple[float, float],
    edge_meta: Dict[str, float],
    turn_weight: float,
    turn_radius_m: float,
    max_turn_deflection_deg: Optional[float] = None,
) -> Tuple[Optional[float], bool]:
    ang = _vector_angle(prev_vec, next_vec)
    if max_turn_deflection_deg is not None and ang > max_turn_deflection_deg:
        return None, False
    if ang <= TURN_IGNORE_DEG:
        return 0.0, False
    turn_severity = (ang - TURN_IGNORE_DEG) / max(1e-6, 180.0 - TURN_IGNORE_DEG)
    turn_severity = max(0.0, min(1.0, turn_severity)) ** 1.7
    bend_radius_proxy = edge_meta["dist"] / max(0.1, math.radians(max(5.0, ang)))
    radius_pen = max(0.0, (turn_radius_m - bend_radius_proxy) / max(1.0, turn_radius_m))
    ntype = edge_meta.get("network_type", "road")
    road_turn_boost = 1.75 if ntype == "road" else 1.0
    sharp_bonus = 0.85 if ang >= TURN_SHARP_DEG else 0.0
    fixed_pen = turn_weight * TURN_FIXED_SCALE * (1.2 if ntype == "road" else 0.65)
    penalty = fixed_pen + turn_weight * road_turn_boost * (turn_severity + 0.65 * radius_pen + sharp_bonus)
    return penalty, True


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
        lon0, lat0 = xy_to_wgs(inv, x0, y0)
        v = pop_sampler.sample(lon0, lat0)
        return v, v, v
    vals: List[float] = []
    for r in _sample_ratios_for_segment(seg_xy.length):
        p = seg_xy.interpolate(seg_xy.length * r)
        px, py = p.coords[0]
        lon, lat = xy_to_wgs(inv, px, py)
        vals.append(pop_sampler.sample(lon, lat))
    if not vals:
        return 0.0, 0.0, 0.0
    return sum(vals) / len(vals), _percentile(vals, 0.9), max(vals)


def _line_offset_stats(seg_xy: LineString, reference_line_xy: LineString) -> Tuple[float, float, float]:
    if seg_xy.is_empty or reference_line_xy.is_empty:
        return 0.0, 0.0, 0.0
    vals: List[float] = []
    if seg_xy.length < 1.0:
        p = Point(seg_xy.coords[0])
        d = float(p.distance(reference_line_xy))
        return d, d, d
    for r in _sample_ratios_for_segment(seg_xy.length):
        p = seg_xy.interpolate(seg_xy.length * r)
        vals.append(float(p.distance(reference_line_xy)))
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
    school_hard_union_xy,
    sensitive_hard_union_xy,
    school_penalty_air: float,
    school_penalty_ground: float,
    key_points_xy: List[Any],
    key_tree: Optional[STRtree],
    key_id_map: Dict[int, int],
    line_risk_union_xy,
    high_building_union_xy,
    enable_water_endpoint_connectors: bool = True,
    edge_feature_cache: Optional[Dict[Tuple[Tuple[float, float], Tuple[float, float], str], Dict[str, float]]] = None,
    reference_line_xy: Optional[LineString] = None,
    reference_corridor_m: float = 300.0,
    reference_deviation_weight: float = 0.0,
) -> Tuple[
    Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]],
    Dict[str, int],
]:
    graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]] = defaultdict(list)
    node_set: Dict[Tuple[float, float], Tuple[float, float]] = {}
    nofly_hard = no_fly_hard_union_xy if no_fly_hard_union_xy is not None and (not no_fly_hard_union_xy.is_empty) else None
    nofly_soft = no_fly_soft_union_xy if no_fly_soft_union_xy is not None and (not no_fly_soft_union_xy.is_empty) else None
    infra_hard = infra_hard_union_xy if infra_hard_union_xy is not None and (not infra_hard_union_xy.is_empty) else None
    school_hard = school_hard_union_xy if school_hard_union_xy is not None and (not school_hard_union_xy.is_empty) else None
    sensitive_hard = (
        sensitive_hard_union_xy
        if sensitive_hard_union_xy is not None and (not sensitive_hard_union_xy.is_empty)
        else None
    )
    line_risk_union = line_risk_union_xy if line_risk_union_xy is not None and (not line_risk_union_xy.is_empty) else None
    high_building_union = (
        high_building_union_xy if high_building_union_xy is not None and (not high_building_union_xy.is_empty) else None
    )
    reference_line = (
        reference_line_xy
        if reference_line_xy is not None and (not reference_line_xy.is_empty)
        else None
    )
    ref_corridor = max(1.0, float(reference_corridor_m))
    ref_dev_weight = max(0.0, float(reference_deviation_weight))
    skipped_nofly = 0
    skipped_infra_hard = 0
    skipped_school_hard = 0
    skipped_sensitive_hard = 0
    kept_edges = 0
    air_edges = 0
    min_base_per_m = float("inf")
    lattice_nodes: set[Tuple[float, float]] = set()
    water_nodes: set[Tuple[float, float]] = set()
    edge_seen: set[Tuple[Tuple[float, float], Tuple[float, float], str]] = set()
    feature_cache = edge_feature_cache if edge_feature_cache is not None else {}
    infra_nearest_cache: Dict[Tuple[float, float], Tuple[Optional[int], float]] = {}
    obstacle_nearest_cache: Dict[Tuple[float, float], Tuple[Optional[int], float]] = {}
    crowd_nearest_cache: Dict[Tuple[float, float], Tuple[Optional[int], float]] = {}
    key_nearest_cache: Dict[Tuple[float, float], Tuple[Optional[int], float]] = {}

    def canonical(p: Tuple[float, float]) -> Tuple[float, float]:
        k = _round_key(p, GRAPH_NODE_SNAP_M)
        if k not in node_set:
            node_set[k] = k
        return node_set[k]

    def _edge_features_for_segment(seg: LineString, ntype: str) -> Dict[str, float]:
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
            point_xy = (sx, sy)
            point_geom = Point(point_xy)
            lon, lat = xy_to_wgs(inv, sx, sy)
            pop_values.append(pop_sampler.sample(lon, lat))
            land_samples.append(
                landuse_cost_at_point(
                    point_xy,
                    landuse_geoms,
                    landuse_costs,
                    landuse_tree,
                    landuse_id_map,
                    point_geom=point_geom,
                )
            )
            infra_samples.append(
                infrastructure_penalty_at_point(
                    point_xy,
                    infra_geoms,
                    infra_severities,
                    infra_tree,
                    infra_id_map,
                    point_geom=point_geom,
                    nearest_cache=infra_nearest_cache,
                )
            )
            building_h = building_height_at_point(
                point_xy,
                b_geoms,
                b_heights,
                b_tree,
                b_id_map,
                point_geom=point_geom,
            )
            obstacle_h = obstacle_height_near_point(
                point_xy,
                o_geoms,
                o_heights,
                o_tree,
                o_id_map,
                point_geom=point_geom,
                nearest_cache=obstacle_nearest_cache,
            )
            height_samples.append(max(building_h, obstacle_h) / 50.0)
            crowd_samples.append(
                point_risk_penalty_at_point(
                    point_xy,
                    crowd_points_xy,
                    crowd_tree,
                    crowd_id_map,
                    inner_m=50.0,
                    point_geom=point_geom,
                    nearest_cache=crowd_nearest_cache,
                )
            )
            key_samples.append(
                point_risk_penalty_at_point(
                    point_xy,
                    key_points_xy,
                    key_tree,
                    key_id_map,
                    inner_m=70.0,
                    point_geom=point_geom,
                    nearest_cache=key_nearest_cache,
                )
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
        school_overlap = _segment_overlap_ratio(seg, school_hard)
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
        reference_dist_vals: List[float] = []
        if reference_line is not None and ref_dev_weight > 0:
            for ratio in ratios:
                sx = a[0] * (1.0 - ratio) + b[0] * ratio
                sy = a[1] * (1.0 - ratio) + b[1] * ratio
                reference_dist_vals.append(float(Point(sx, sy).distance(reference_line)))
        ref_mean = sum(reference_dist_vals) / len(reference_dist_vals) if reference_dist_vals else 0.0
        ref_p90 = _percentile(reference_dist_vals, 0.9) if reference_dist_vals else 0.0
        ref_norm = min(4.0, ref_mean / ref_corridor) if reference_dist_vals else 0.0
        return {
            "dist": dist,
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
            "low_risk_land_ratio": low_risk_land_ratio,
            "reference_mean_offset_m": ref_mean,
            "reference_p90_offset_m": ref_p90,
            "reference_dev_norm": ref_norm,
            "network_type": ntype,
        }

    def _score_edge_features(features: Dict[str, float]) -> Dict[str, float]:
        ntype = str(features.get("network_type", "road"))
        if ntype == "water":
            ground_mult = 0.52
            network_mult = 0.8
        elif ntype == "road":
            ground_mult = 1.14
            network_mult = 1.15
        else:
            ground_mult = 0.72
            network_mult = 0.92
        pop_p90 = float(features.get("pop_p90", 0.0))
        low_risk_land_ratio = float(features.get("low_risk_land_ratio", 0.0))
        water_like_ratio = float(features.get("water_like_ratio", 0.0))
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
            + weights["population"] * float(features.get("pop", 0.0))
            + weights["landuse"] * float(features.get("land", 1.0)) * ground_mult
            + weights["infrastructure"] * float(features.get("infra", 0.0))
            + weights["altitude"] * float(features.get("height_proxy", 0.0))
            + weights["soft_no_fly"] * float(features.get("soft_no_fly", 0.0))
            + weights["crowd"] * float(features.get("crowd_pen", 0.0))
            + weights["key_facility"] * float(features.get("key_pen", 0.0))
            + weights["line_cross"] * float(features.get("line_overlap", 0.0))
            + weights.get("reference_deviation", 0.0)
            * ref_dev_weight
            * float(features.get("reference_dev_norm", 0.0))
            + (school_penalty_air if ntype == "air" else school_penalty_ground)
            * float(features.get("school_overlap", 0.0))
        ) * network_mult * context_mult
        dist = max(1e-6, float(features.get("dist", 1.0)))
        scored = dict(features)
        scored["base"] = dist * max(0.01, per_m)
        scored["air_edge"] = 1.0 if ntype == "air" else 0.0
        scored["water_edge"] = 1.0 if ntype == "water" else 0.0
        return scored

    def try_add_edge(a: Tuple[float, float], b: Tuple[float, float], ntype: str) -> bool:
        nonlocal kept_edges, skipped_nofly, skipped_infra_hard, skipped_school_hard, skipped_sensitive_hard, air_edges, min_base_per_m
        if a == b:
            return False
        edge_key = (a, b, ntype) if a <= b else (b, a, ntype)
        if edge_key in edge_seen:
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
        if school_hard is not None and seg.intersects(school_hard):
            skipped_school_hard += 1
            return False
        if sensitive_hard is not None and seg.intersects(sensitive_hard):
            skipped_sensitive_hard += 1
            return False
        meta = feature_cache.get(edge_key)
        if meta is None:
            meta = _edge_features_for_segment(seg, ntype=ntype)
            feature_cache[edge_key] = meta
        meta = _score_edge_features(meta)
        dist_m = max(1e-6, float(meta.get("dist", 1.0)))
        min_base_per_m = min(min_base_per_m, float(meta.get("base", dist_m)) / dist_m)
        vector_ab = (b[0] - a[0], b[1] - a[1])
        vector_ba = (a[0] - b[0], a[1] - b[1])
        graph[a].append((b, meta, vector_ab))
        graph[b].append((a, meta, vector_ba))
        edge_seen.add(edge_key)
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
            "skipped_school_hard": skipped_school_hard,
            "skipped_sensitive_hard": skipped_sensitive_hard,
            "skipped_crowd_hard": skipped_school_hard + skipped_sensitive_hard,
            "min_base_per_m": round(min_base_per_m, 6) if math.isfinite(min_base_per_m) else 1.0,
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
        "skipped_school_hard": skipped_school_hard,
        "skipped_sensitive_hard": skipped_sensitive_hard,
        "skipped_crowd_hard": skipped_school_hard + skipped_sensitive_hard,
        "min_base_per_m": round(min_base_per_m, 6) if math.isfinite(min_base_per_m) else 1.0,
    }


def astar_with_turn_penalty(
    graph: Dict[Tuple[float, float], List[Tuple[Tuple[float, float], Dict[str, float], Tuple[float, float]]]],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    turn_weight: float,
    turn_radius_m: float,
    water_pref_factor: float = 1.0,
    max_turn_deflection_deg: Optional[float] = None,
    min_per_m_hint: Optional[float] = None,
) -> Tuple[Optional[List[Tuple[float, float]]], Dict[str, float]]:
    if start not in graph or goal not in graph:
        return None, {}
    min_per_m = float(min_per_m_hint) if min_per_m_hint is not None else float("inf")
    if (not math.isfinite(min_per_m)) or min_per_m <= 0:
        min_per_m = float("inf")
        for neighbors in graph.values():
            for _, meta, _ in neighbors:
                dist = max(1e-6, meta.get("dist", 1.0))
                min_per_m = min(min_per_m, meta.get("base", dist) / dist)
    if (not math.isfinite(min_per_m)) or min_per_m <= 0:
        min_per_m = 1.0
    start_state = (None, start)
    pq: List[Tuple[float, float, Tuple[Optional[Tuple[float, float]], Tuple[float, float]]]] = []
    heapq.heappush(pq, (0.0, 0.0, start_state))
    g_score: Dict[Tuple[Optional[Tuple[float, float]], Tuple[float, float]], float] = {start_state: 0.0}
    parent: Dict[
        Tuple[Optional[Tuple[float, float]], Tuple[float, float]],
        Tuple[Optional[Tuple[float, float]], Tuple[float, float]],
    ] = {}
    best_goal_state = None
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
        if g_cur > g_score.get(state, float("inf")) + 1e-9:
            continue
        prev_node, cur_node = state
        if cur_node == goal:
            best_goal_state = state
            break
        for next_node, edge_meta, vec in graph.get(cur_node, []):
            turn_penalty = 0.0
            if prev_node is not None:
                pv = (cur_node[0] - prev_node[0], cur_node[1] - prev_node[1])
                turn_penalty_maybe, _ = _turn_penalty_for_vectors(
                    prev_vec=pv,
                    next_vec=vec,
                    edge_meta=edge_meta,
                    turn_weight=turn_weight,
                    turn_radius_m=turn_radius_m,
                    max_turn_deflection_deg=max_turn_deflection_deg,
                )
                if turn_penalty_maybe is None:
                    continue
                turn_penalty = turn_penalty_maybe
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
                turn_penalty_maybe, has_turn = _turn_penalty_for_vectors(
                    prev_vec=pv,
                    next_vec=vec,
                    edge_meta=meta,
                    turn_weight=turn_weight,
                    turn_radius_m=turn_radius_m,
                )
                if has_turn:
                    turn_count += 1
                    total_turn += float(turn_penalty_maybe or 0.0)
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
        lon, lat = xy_to_wgs(inv, x, y)
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


def build_dynamic_grb_geometry(route_line_xy: LineString, profile_samples: List[Dict[str, float]]) -> Optional[Any]:
    if route_line_xy is None or route_line_xy.is_empty or not profile_samples:
        return None
    geoms: List[Any] = []
    samples = _downsample_profile_samples(profile_samples, max_points=420)
    route_len = max(1e-6, float(route_line_xy.length))
    for sample in samples:
        dist_m = max(0.0, min(route_len, _safe_float(sample.get("distance_m", 0.0), 0.0)))
        agl_m = max(1.0, _safe_float(sample.get("true_height_m", 0.0), 0.0))
        try:
            p = route_line_xy.interpolate(dist_m)
            geoms.append(p.buffer(agl_m))
        except Exception:
            continue
    if not geoms:
        return None
    dyn = unary_union(geoms)
    if dyn is None or dyn.is_empty:
        return None
    return dyn


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


def _build_profile_panel_html(
    name: str,
    profile_variants: Dict[str, Dict[str, Any]],
    default_profile_variant_id: str,
) -> str:
    if not profile_variants:
        return ""
    width = 1100.0
    height = 280.0
    left = 64.0
    right = 18.0
    top = 20.0
    bottom = 36.0
    plot_w = width - left - right
    plot_h = height - top - bottom

    variants_html: List[str] = []
    options_html: List[str] = []
    variant_ids = list(profile_variants.keys())
    default_id = str(default_profile_variant_id or "")
    if default_id not in variant_ids:
        default_id = variant_ids[0]
    for vid in variant_ids:
        item = profile_variants.get(vid) or {}
        samples = _downsample_profile_samples(item.get("samples") or [], max_points=260)
        if len(samples) < 2:
            continue
        label = str(item.get("label", vid))
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
        selected = " selected" if vid == default_id else ""
        hidden_cls = "" if vid == default_id else " is-hidden"
        options_html.append(f'<option value="{html.escape(vid)}"{selected}>{html.escape(label)}</option>')
        variants_html.append(
            f"""
    <div class="route-profile-panel__variant{hidden_cls}" data-role="profile-variant" data-profile-id="{html.escape(vid)}">
      <div class="route-profile-panel__meta">
        {html.escape(name)} | {html.escape(label)} | 距离 {dist_max/1000.0:.2f} km | 真高最小/均值 {min_true:.1f}/{mean_true:.1f} m
      </div>
      <svg viewBox="0 0 {width:.0f} {height:.0f}" class="route-profile-panel__chart">
        {''.join(grid_parts)}
        <polyline fill="none" stroke="#94a9b8" stroke-width="2" points="{terrain_path}"/>
        <polyline fill="none" stroke="#8d6e63" stroke-width="2.5" points="{surface_path}"/>
        <polyline fill="none" stroke="#1e5ea8" stroke-width="2.8" points="{route_path}"/>
        {''.join(label_parts)}
      </svg>
    </div>
"""
        )
    if not variants_html:
        return ""
    panel = f"""
<div class="route-profile-panel">
  <div class="route-profile-panel__head">
    <div>
      <div class="route-profile-panel__title">航线垂直剖面</div>
      <div class="route-profile-panel__head-controls">
        <label>剖面航线
          <select data-role="profile-variant-select">
            {''.join(options_html)}
          </select>
        </label>
      </div>
    </div>
    <button type="button" class="route-profile-panel__toggle" data-role="profile-toggle">收起剖面</button>
  </div>
  <div class="route-profile-panel__body">
    {''.join(variants_html)}
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

  .leaflet-control-layers.route-native-hidden {
    display: none !important;
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
    z-index: 1160;
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

  .route-map-toolbar__label-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    margin-bottom: 6px;
  }

  .route-map-toolbar__collapse {
    border: 1px solid #c5d6e7;
    background: #f2f7fe;
    color: #325375;
    border-radius: 999px;
    padding: 3px 9px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.03em;
    cursor: pointer;
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
    transition: background-color 0.16s ease, border-color 0.16s ease, color 0.16s ease, box-shadow 0.16s ease, transform 0.16s ease;
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

  .route-map-toolbar.is-layers-collapsed .route-map-toolbar__layers {
    display: none;
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

  .route-editor-panel {
    position: absolute;
    top: 74px;
    left: 14px;
    bottom: 220px;
    z-index: 1140;
    width: min(350px, calc(100vw - 28px));
    border: 1px solid var(--panel-border);
    border-radius: 16px;
    background: var(--panel-bg);
    box-shadow: var(--panel-shadow);
    backdrop-filter: blur(10px);
    color: var(--text-main);
    overflow: auto;
    display: flex;
    flex-direction: column;
    max-height: min(62vh, calc(100vh - 300px));
    overscroll-behavior: contain;
  }

  .route-editor-panel__head {
    padding: 10px 12px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    border-bottom: 1px solid #d9e6f3;
    background: linear-gradient(180deg, #fbfdff 0%, #f2f8ff 100%);
  }

  .route-editor-panel__head-actions {
    display: flex;
    align-items: center;
    gap: 6px;
  }

  .route-editor-panel__title {
    font-size: 15px;
    line-height: 1.2;
    font-weight: 700;
    letter-spacing: 0.01em;
  }

  .route-editor-panel__head button {
    border: 1px solid #c2d5e8;
    background: #f5f9ff;
    color: #274465;
    border-radius: 10px;
    padding: 5px 10px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
  }

  .route-editor-panel__body {
    padding: 11px 12px 12px;
    display: grid;
    gap: 9px;
    overflow: auto;
    overscroll-behavior: contain;
  }

  .route-editor-panel__row {
    display: grid;
    gap: 8px;
  }

  .route-editor-panel__hint {
    font-size: 12px;
    line-height: 1.45;
    color: #4a6886;
    background: #f3f8ff;
    border: 1px solid #d9e7f4;
    border-radius: 10px;
    padding: 7px 9px;
  }

  .route-editor-panel__row--inline {
    display: grid;
    grid-template-columns: 1fr auto auto;
    gap: 7px;
    align-items: center;
  }

  .route-editor-panel__row--pair {
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 8px;
    align-items: center;
  }

  .route-editor-panel__advanced-wrap {
    display: grid;
    gap: 8px;
    background: #f7fbff;
    border: 1px solid #dce9f5;
    border-radius: 10px;
    padding: 8px;
  }

  .route-editor-panel__advanced-wrap.is-collapsed {
    display: none;
  }

  .route-editor-panel__text {
    font-size: 12px;
    color: #4b6986;
    line-height: 1.45;
  }

  .route-editor-panel__field-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }

  .route-editor-panel label {
    display: grid;
    gap: 4px;
    font-size: 12px;
    color: #355474;
    font-weight: 600;
  }

  .route-editor-panel input {
    width: 100%;
    border: 1px solid #c8d9ea;
    border-radius: 10px;
    background: #fbfdff;
    color: #18314d;
    padding: 7px 9px;
    font-size: 13px;
    line-height: 1.2;
  }

  .route-editor-panel select {
    width: 100%;
    border: 1px solid #c8d9ea;
    border-radius: 10px;
    background: #fbfdff;
    color: #18314d;
    padding: 7px 9px;
    font-size: 13px;
    line-height: 1.2;
  }

  .route-editor-panel__alt-nudges {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 7px;
  }

  .route-editor-panel__alt-nudges button {
    border: 1px solid #bcd0e5;
    background: #f5faff;
    color: #20405f;
    border-radius: 10px;
    padding: 7px 8px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
  }

  .route-editor-panel__actions {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 7px;
  }

  .route-editor-panel__history {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 7px;
  }

  .route-editor-panel__actions button,
  .route-editor-panel__history button,
  .route-editor-panel__primary {
    border: 1px solid #b7cde2;
    background: #f4f9ff;
    color: #1f3d5c;
    border-radius: 10px;
    padding: 7px 9px;
    font-size: 13px;
    font-weight: 600;
    cursor: pointer;
    touch-action: manipulation;
    -webkit-tap-highlight-color: rgba(30, 94, 168, 0.14);
  }

  .route-editor-panel__actions button:disabled,
  .route-editor-panel__history button:disabled,
  .route-editor-panel__primary:disabled {
    opacity: 0.45;
    cursor: not-allowed;
  }

  .route-editor-panel__primary {
    background: #1f62ad;
    color: #ffffff;
    border-color: #1f62ad;
  }

  .route-editor-panel__status {
    min-height: 18px;
    font-size: 12px;
    line-height: 1.4;
    color: #496784;
  }

  .route-editor-panel.is-collapsed .route-editor-panel__body {
    display: none;
  }

  .route-waypoint-marker {
    width: 28px;
    height: 28px;
    margin-left: -14px;
    margin-top: -14px;
    border-radius: 999px;
    display: grid;
    place-items: center;
    background: #ffffff;
    border: 2px solid #1f5fa8;
    color: #1f5fa8;
    font-size: 12px;
    font-weight: 700;
    line-height: 1;
    box-shadow: 0 2px 10px rgba(24, 64, 109, 0.24), 0 0 0 2px rgba(255, 255, 255, 0.84);
    transition: transform 0.12s ease, border-color 0.12s ease, color 0.12s ease;
    cursor: grab;
    touch-action: none;
  }

  .route-waypoint-marker.is-dirty {
    border-color: #cb7a1f;
    color: #cb7a1f;
    background: #fff8ec;
  }

  .route-waypoint-marker.is-active {
    background: #fffbf1;
    border-color: #d26c10;
    color: #ad5200;
    transform: scale(1.14);
    box-shadow: 0 0 0 3px rgba(255, 228, 194, 0.95), 0 6px 12px rgba(173, 82, 0, 0.22);
    cursor: grabbing;
  }

  .route-midpoint-handle {
    width: 18px;
    height: 18px;
    margin-left: -9px;
    margin-top: -9px;
    border-radius: 999px;
    display: grid;
    place-items: center;
    background: #f3f8ff;
    border: 1.5px dashed #5583b3;
    color: #3d6b99;
    font-size: 12px;
    font-weight: 700;
    line-height: 1;
    box-shadow: 0 2px 8px rgba(53, 94, 138, 0.22);
  }

  .route-midpoint-handle.is-active {
    background: #fff3df;
    border-style: solid;
    border-color: #cb7a1f;
    color: #cb7a1f;
  }

  body.route-dragging,
  body.route-dragging * {
    user-select: none !important;
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
    transition: background-color 0.16s ease, border-color 0.16s ease, color 0.16s ease;
    touch-action: manipulation;
    -webkit-tap-highlight-color: rgba(30, 94, 168, 0.14);
  }

  .route-editor-panel button:focus-visible,
  .route-editor-panel input:focus-visible,
  .route-editor-panel select:focus-visible,
  .route-profile-panel button:focus-visible {
    outline: 2px solid #1e5ea8;
    outline-offset: 2px;
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

  .route-profile-panel__head-controls {
    margin-top: 5px;
  }

  .route-profile-panel__head-controls label {
    display: inline-grid;
    gap: 4px;
    font-size: 12px;
    color: #355474;
    font-weight: 600;
  }

  .route-profile-panel__head-controls select {
    border: 1px solid #c8d9ea;
    border-radius: 10px;
    background: #fbfdff;
    color: #18314d;
    padding: 6px 9px;
    font-size: 13px;
    line-height: 1.2;
  }

  .route-profile-panel__meta {
    margin-top: 3px;
    font-size: 13px;
    color: var(--text-sub);
    line-height: 1.45;
  }

  .route-profile-panel__variant.is-hidden {
    display: none;
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

    .route-editor-panel {
      width: min(92vw, 336px);
      max-height: min(58vh, calc(100vh - 280px));
    }
  }

  @media (max-width: 768px) {
    .route-map-toolbar {
      top: 10px;
      bottom: auto;
      left: 10px;
      right: auto;
      width: min(80vw, 312px);
      max-height: 38vh;
      overflow: auto;
    }

    .route-profile-panel {
      left: 8px;
      right: 8px;
      bottom: 8px;
      padding: 10px;
    }

    .route-editor-panel {
      top: auto;
      left: 10px;
      right: 10px;
      bottom: 112px;
      width: auto;
      max-height: 46vh;
      overflow: auto;
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


def _build_preview_toolbar_script_html(
    name: str,
    route_variants: Dict[str, Dict[str, Any]],
    default_route_variant_id: str,
) -> str:
    route_variants_json = json.dumps(route_variants, ensure_ascii=False)
    default_variant_json = json.dumps(str(default_route_variant_id or "safety_default"), ensure_ascii=False)
    route_name_json = json.dumps(str(name), ensure_ascii=False)
    template = """
<script>
(function () {
  const routeEditorVariants = __ROUTE_VARIANTS_JSON__;
  const routeEditorDefaultId = __ROUTE_DEFAULT_ID_JSON__;
  const routeEditorName = __ROUTE_NAME_JSON__;

  function hasAny(text, keywords) {
    const source = String(text || "");
    return keywords.some(function (k) {
      return source.indexOf(k) >= 0;
    });
  }

  function groupName(layerName) {
    if (hasAny(layerName, ["候选航线", "Candidate:", "主航线", "Route (3D", "安全优先", "效率优先"])) return "航线方案";
    if (hasAny(layerName, ["禁飞", "No-fly", "缓冲", "Buffer", "GRB", "起点", "Start/End"])) return "安全边界";
    if (hasAny(layerName, ["高层", "High Buildings", "建筑", "学校", "School", "人群", "Crowd", "敏感设施", "关键基础设施", "Line Risks", "低风险", "人口密度", "土地利用"])) return "风险参考";
    return "其他";
  }

  function clampNumber(value, minVal, maxVal) {
    const v = Number(value);
    const lo = Number(minVal);
    const hi = Number(maxVal);
    if (!Number.isFinite(v)) return lo;
    if (!Number.isFinite(lo)) return v;
    if (!Number.isFinite(hi)) return Math.max(v, lo);
    return Math.min(Math.max(v, lo), hi);
  }

  function isElementVisible(el) {
    if (!el) return false;
    const style = window.getComputedStyle(el);
    if (!style) return false;
    return style.display !== "none" && style.visibility !== "hidden" && Number(style.opacity || "1") > 0;
  }

  function rectsIntersect(a, b) {
    if (!a || !b) return false;
    return a.left < b.right && a.right > b.left && a.top < b.bottom && a.bottom > b.top;
  }

  function triggerPanelLayout() {
    if (typeof window.__routeSchedulePanelLayout === "function") {
      window.__routeSchedulePanelLayout();
    }
  }

  function setupPanelLayout(map) {
    if (!map || map._routePanelLayoutReady) return;
    const container = map.getContainer();
    if (!container) return;

    function applyLayout() {
      const editor = container.querySelector(".route-editor-panel");
      if (!editor) return;
      const containerRect = container.getBoundingClientRect();
      if (!containerRect || containerRect.height <= 0) return;

      const mobile = window.matchMedia("(max-width: 768px)").matches;
      const gap = mobile ? 8 : 10;
      const minHeight = mobile ? 140 : 120;
      const defaultBottom = mobile ? 112 : 220;
      const editorCollapsed = editor.classList.contains("is-collapsed");
      const nativeLayers = container.querySelector(".leaflet-control-layers");
      if (nativeLayers) {
        nativeLayers.classList.remove("route-native-hidden");
      }
      let bottomPx = defaultBottom;
      const profile = document.querySelector(".route-profile-panel");
      if (profile) {
        if (mobile || profile.classList.contains("is-collapsed")) {
          profile.style.left = "";
          profile.style.right = "";
          profile.style.maxWidth = "";
          profile.style.margin = "";
        } else {
          let leftInset = 12;
          let rightInset = 12;
          if (isElementVisible(editor)) {
            const editorRect = editor.getBoundingClientRect();
            leftInset = Math.max(leftInset, Math.ceil(editorRect.right + gap));
          }
          if (isElementVisible(nativeLayers)) {
            const nativeRect = nativeLayers.getBoundingClientRect();
            rightInset = Math.max(rightInset, Math.ceil(window.innerWidth - nativeRect.left + gap));
          }
          const minProfileWidth = 420;
          const availWidth = window.innerWidth - leftInset - rightInset;
          if (availWidth < minProfileWidth) {
            leftInset = 12;
            rightInset = 12;
          }
          profile.style.left = leftInset + "px";
          profile.style.right = rightInset + "px";
          profile.style.maxWidth = "none";
          profile.style.margin = "0";
        }
      }
      if (isElementVisible(profile)) {
        const profileRect = profile.getBoundingClientRect();
        const overlap = containerRect.bottom - profileRect.top;
        if (Number.isFinite(overlap) && overlap > 0) {
          bottomPx = Math.max(bottomPx, Math.ceil(overlap + gap));
        }
      }

      if (editorCollapsed) {
        if (mobile) {
          editor.style.left = "";
          editor.style.right = "";
          editor.style.top = "10px";
        } else {
          editor.style.left = "14px";
          editor.style.right = "auto";
          editor.style.top = "74px";
        }
        editor.style.bottom = "auto";
        editor.style.maxHeight = "none";
        editor.style.overflow = "hidden";
        return;
      }
      editor.style.overflow = "auto";

      if (mobile) {
        const bottomMin = 8;
        const bottomMax = Math.max(bottomMin, containerRect.height - minHeight - 8);
        bottomPx = clampNumber(bottomPx, bottomMin, bottomMax);
        const available = Math.max(minHeight, Math.floor(containerRect.height - bottomPx - 16));
        editor.style.left = "";
        editor.style.right = "";
        editor.style.top = "auto";
        editor.style.bottom = bottomPx + "px";
        editor.style.maxHeight = available + "px";
        return;
      }

      const topPx = 74;

      const topMin = 14;
      const bottomMin = 16;
      const topMax = Math.max(topMin, containerRect.height - bottomPx - minHeight);
      const finalTopPx = clampNumber(topPx, topMin, topMax);
      const bottomMax = Math.max(bottomMin, containerRect.height - finalTopPx - minHeight);
      bottomPx = clampNumber(bottomPx, bottomMin, bottomMax);
      const available = Math.max(minHeight, Math.floor(containerRect.height - finalTopPx - bottomPx));

      editor.style.left = "14px";
      editor.style.right = "auto";
      editor.style.top = finalTopPx + "px";
      editor.style.bottom = bottomPx + "px";
      editor.style.maxHeight = available + "px";
    }

    function scheduleLayout() {
      if (map._routePanelLayoutRaf) return;
      map._routePanelLayoutRaf = window.requestAnimationFrame(function () {
        map._routePanelLayoutRaf = 0;
        applyLayout();
      });
    }

    map._routeSchedulePanelLayout = scheduleLayout;
    window.__routeSchedulePanelLayout = scheduleLayout;
    window.addEventListener("resize", scheduleLayout);
    window.addEventListener("orientationchange", scheduleLayout);
    map.on("resize", scheduleLayout);
    map._routePanelLayoutReady = true;
    scheduleLayout();
  }

  function setupProfilePanel() {
    const panel = document.querySelector(".route-profile-panel");
    if (!panel || panel.dataset.ready === "1") return;
    const toggle = panel.querySelector('[data-role="profile-toggle"]');
    const variantSelect = panel.querySelector('[data-role="profile-variant-select"]');
    const variants = Array.from(panel.querySelectorAll('[data-role="profile-variant"]'));
    function switchProfileVariant(profileId) {
      const targetId = String(profileId || "");
      variants.forEach(function (node) {
        const nid = String(node && node.getAttribute("data-profile-id") || "");
        node.classList.toggle("is-hidden", nid !== targetId);
      });
      triggerPanelLayout();
    }
    if (!toggle) return;
    if (variantSelect && variants.length) {
      const initId = String(variantSelect.value || (variants[0] && variants[0].getAttribute("data-profile-id")) || "");
      switchProfileVariant(initId);
      variantSelect.addEventListener("change", function () {
        switchProfileVariant(variantSelect.value);
      });
    }
    panel.classList.add("is-collapsed");
    toggle.textContent = "展开剖面";
    toggle.addEventListener("click", function () {
      const collapsed = panel.classList.toggle("is-collapsed");
      toggle.textContent = collapsed ? "展开剖面" : "收起剖面";
      triggerPanelLayout();
    });
    panel.dataset.ready = "1";
  }

  function setOverlayVisible(map, entry, visible) {
    if (!entry) return;
    if (entry.input) {
      const next = !!visible;
      if (!!entry.input.checked !== next) entry.input.click();
      return;
    }
    if (!entry.layer) return;
    if (visible) {
      if (!map.hasLayer(entry.layer)) map.addLayer(entry.layer);
    } else {
      if (map.hasLayer(entry.layer)) map.removeLayer(entry.layer);
    }
  }

  function normalizeLayerName(name) {
    return String(name || "").replace(/\\s+/g, " ").trim();
  }

  function readNativeLayerEntries() {
    const root = document.querySelector(".leaflet-control-layers");
    if (!root) return { base: [], overlays: [] };
    function collect(selector, overlay) {
      return Array.from(root.querySelectorAll(selector + " label"))
        .map(function (label) {
          const input = label.querySelector("input.leaflet-control-layers-selector");
          if (!input) return null;
          const name = normalizeLayerName(label.textContent);
          return {
            name: name || (overlay ? "图层" : "底图"),
            layer: null,
            overlay: !!overlay,
            input: input,
          };
        })
        .filter(Boolean);
    }
    return {
      base: collect(".leaflet-control-layers-base", false),
      overlays: collect(".leaflet-control-layers-overlays", true),
    };
  }

  function mergeEntriesWithNative(entries, nativeEntries) {
    const output = Array.isArray(entries) ? entries.slice() : [];
    const normalized = new Map();
    output.forEach(function (entry, idx) {
      normalized.set(normalizeLayerName(entry && entry.name), { entry: entry, idx: idx });
    });
    (Array.isArray(nativeEntries) ? nativeEntries : []).forEach(function (nativeEntry) {
      const key = normalizeLayerName(nativeEntry && nativeEntry.name);
      const existing = normalized.get(key);
      if (existing && existing.entry) {
        existing.entry.input = nativeEntry.input || existing.entry.input;
        if (!existing.entry.layer && nativeEntry.layer) existing.entry.layer = nativeEntry.layer;
      } else {
        output.push(nativeEntry);
      }
    });
    return output;
  }

  function findLayerCatalog() {
    const keys = Object.keys(window).filter(function (key) {
      return /^layer_control_.*_layers$/.test(key);
    });
    for (let i = keys.length - 1; i >= 0; i -= 1) {
      const value = window[keys[i]];
      if (value && value.base_layers && value.overlays) return value;
    }
    return null;
  }

  function hideNativeLayerControl() {
    const el = document.querySelector(".leaflet-control-layers");
    if (el) el.classList.add("route-native-hidden");
  }

  function setupToolbar(map) {
    if (!map || map._routeToolbarReady) return true;
    const catalog = findLayerCatalog();
    const nativeEntries = readNativeLayerEntries();
    if (!catalog && !nativeEntries.base.length) return false;

    const container = map.getContainer();
    if (!container || container.querySelector(".route-map-toolbar")) return true;

    let baseEntries = catalog
      ? Object.entries(catalog.base_layers || {}).map(function (entry) {
          return { name: String(entry[0] || "底图"), layer: entry[1], overlay: false };
        })
      : [];
    let overlayEntries = catalog
      ? Object.entries(catalog.overlays || {}).map(function (entry) {
          return { name: String(entry[0] || "图层"), layer: entry[1], overlay: true };
        })
      : [];
    baseEntries = mergeEntriesWithNative(baseEntries, nativeEntries.base);
    overlayEntries = mergeEntriesWithNative(overlayEntries, nativeEntries.overlays);
    const dynamicOverlays = Array.isArray(map._routeDynamicOverlays) ? map._routeDynamicOverlays : [];
    dynamicOverlays.forEach(function (item) {
      if (item && item.layer) {
        overlayEntries.push({ name: String(item.name || "动态图层"), layer: item.layer, overlay: true });
      }
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
      '<div class="route-map-toolbar__label-row">' +
      '<div class="route-map-toolbar__label">图层</div>' +
      '<button type="button" class="route-map-toolbar__collapse" data-role="layers-toggle">收起</button>' +
      '</div>' +
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
      if (target && target.input) {
        if (!target.input.checked) target.input.click();
        if (satLabels) {
          const useSat = !!target && hasAny(target.name, ["卫星"]);
          setOverlayVisible(map, satLabels, useSat);
        }
        syncState();
        return;
      }
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
        const active = item.entry.input ? !!item.entry.input.checked : !!item.entry.layer && map.hasLayer(item.entry.layer);
        item.button.classList.toggle("is-active", active);
      });
      overlayInputs.forEach(function (item) {
        item.input.checked = item.entry.input
          ? !!item.entry.input.checked
          : !!item.entry.layer && map.hasLayer(item.entry.layer);
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
      if (entry.input) {
        entry.input.addEventListener("change", function () {
          syncState();
        });
      }
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

    if (!grouped.length) {
      const empty = document.createElement("div");
      empty.className = "route-editor-panel__text";
      empty.textContent = "未读取到可切换图层，请刷新页面后重试。";
      layerWrap.appendChild(empty);
    } else {
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
        input.checked = item.entry.input
          ? !!item.entry.input.checked
          : !!item.entry.layer && map.hasLayer(item.entry.layer);
        input.addEventListener("change", function () {
          setOverlayVisible(map, item.entry, input.checked);
        });
        if (item.entry.input) {
          item.entry.input.addEventListener("change", function () {
            syncState();
          });
        }
        const text = document.createElement("span");
        text.textContent = item.label;
        row.appendChild(input);
        row.appendChild(text);
        layerWrap.appendChild(row);
        overlayInputs.push({ entry: item.entry, input: input });
      });
    }

    function applyPreset(mode) {
      overlayEntries.forEach(function (entry) {
        if (satLabels && entry === satLabels) return;
        const lname = String(entry.name || "");
        let visible = false;
        if (mode === "clear") {
          visible = hasAny(lname, ["主航线", "Route (3D", "起点/终点", "Start/End", "禁飞", "No-fly", "缓冲", "Buffer", "GRB", "安全优先", "效率优先"]);
        } else if (mode === "risk") {
          visible = hasAny(lname, ["主航线", "Route (3D", "起点/终点", "Start/End", "禁飞", "No-fly", "高层", "High Buildings", "建筑", "学校", "School", "人群", "Crowd", "敏感设施", "Sensitive Facility", "关键基础设施", "Critical Infrastructure", "线性风险", "Line Risks"]);
        }
        setOverlayVisible(map, entry, visible);
      });
      syncState();
    }

    const presetClear = toolbar.querySelector('[data-role="preset-clear"]');
    const presetRisk = toolbar.querySelector('[data-role="preset-risk"]');
    const layersToggle = toolbar.querySelector('[data-role="layers-toggle"]');
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
    if (layersToggle) {
      layersToggle.addEventListener("click", function () {
        const collapsed = toolbar.classList.toggle("is-layers-collapsed");
        layersToggle.textContent = collapsed ? "展开" : "收起";
        triggerPanelLayout();
      });
    }
    map.on("layeradd layerremove baselayerchange", syncState);
    map._routeToolbarReady = true;
    syncState();
    triggerPanelLayout();
    return true;
  }

  function setupRouteEditor(map) {
    if (!map || map._routeEditorReady) return true;
    const variantsObj = routeEditorVariants && typeof routeEditorVariants === "object" ? routeEditorVariants : {};
    const variantIds = Object.keys(variantsObj).filter(function (rid) {
      const item = variantsObj[rid] || {};
      return Array.isArray(item.points) && item.points.length >= 2;
    });
    if (!variantIds.length) {
      map._routeEditorReady = true;
      return true;
    }
    const defaultRouteId = variantIds.indexOf(routeEditorDefaultId) >= 0 ? routeEditorDefaultId : variantIds[0];
    const activeSeed = variantsObj[defaultRouteId].points;
    const container = map.getContainer();
    if (!container) return false;

    const panel = document.createElement("div");
    panel.className = "route-editor-panel";
    panel.innerHTML =
      '<div class="route-editor-panel__head">' +
      '<div class="route-editor-panel__title">航点编辑器</div>' +
      '<div class="route-editor-panel__head-actions">' +
      '<button type="button" data-role="editor-panel-toggle">收起</button>' +
      "</div>" +
      '</div>' +
      '<div class="route-editor-panel__body">' +
      '<div class="route-editor-panel__row">' +
      '<label>编辑基线航线<select name="route_base" data-role="route-base-select"></select></label>' +
      "</div>" +
      '<div class="route-editor-panel__row">' +
      '<button type="button" class="route-editor-panel__primary" data-role="edit-mode-toggle">开启拖拽编辑</button>' +
      '<div class="route-editor-panel__text" data-role="summary">航点 0 / 0</div>' +
      "</div>" +
      '<div class="route-editor-panel__hint" data-role="edit-hint">开启后可拖动航点；点击蓝色航线可直接插入新航点，航线实时跟随。</div>' +
      '<div class="route-editor-panel__history">' +
      '<button type="button" data-role="undo-change">撤销上一步</button>' +
      '<button type="button" data-role="redo-change">重做</button>' +
      "</div>" +
      '<div class="route-editor-panel__row">' +
      '<label>快速选择航点<select name="point_select" data-role="point-select"></select></label>' +
      "</div>" +
      '<div class="route-editor-panel__row route-editor-panel__row--inline">' +
      '<input type="number" min="1" step="1" name="point_index" autocomplete="off" inputmode="numeric" data-role="point-index" />' +
      '<button type="button" data-role="point-prev">上一点</button>' +
      '<button type="button" data-role="point-next">下一点</button>' +
      "</div>" +
      '<div class="route-editor-panel__row route-editor-panel__row--pair">' +
      '<button type="button" data-role="advanced-toggle">展开高级调整</button>' +
      '<div class="route-editor-panel__text">经纬度与高度数值微调</div>' +
      "</div>" +
      '<div class="route-editor-panel__advanced-wrap is-collapsed" data-role="advanced-wrap">' +
      '<div class="route-editor-panel__field-grid">' +
      '<label>经度<input type="number" step="0.000001" name="point_lon" autocomplete="off" inputmode="decimal" data-role="point-lon" /></label>' +
      '<label>纬度<input type="number" step="0.000001" name="point_lat" autocomplete="off" inputmode="decimal" data-role="point-lat" /></label>' +
      '<label style="grid-column: 1 / span 2;">高度(m)<input type="number" step="0.1" name="point_alt" autocomplete="off" inputmode="decimal" data-role="point-alt" /></label>' +
      "</div>" +
      '<div class="route-editor-panel__alt-nudges">' +
      '<button type="button" data-role="alt-minus-5">高度 -5m</button>' +
      '<button type="button" data-role="alt-plus-5">高度 +5m</button>' +
      '<button type="button" data-role="alt-plus-20">高度 +20m</button>' +
      "</div>" +
      "</div>" +
      '<div class="route-editor-panel__actions">' +
      '<button type="button" data-role="restore-point">还原本点(初始)</button>' +
      '<button type="button" data-role="restore-all">还原全部航点</button>' +
      '<button type="button" data-role="locate-point">定位本点</button>' +
      "</div>" +
      '<div class="route-editor-panel__row">' +
      '<label>KML文件名<input type="text" name="kml_name" autocomplete="off" data-role="kml-name" /></label>' +
      '<button type="button" class="route-editor-panel__primary" data-role="save-kml">另存为新KML</button>' +
      "</div>" +
      '<div class="route-editor-panel__status" data-role="editor-status" aria-live="polite"></div>' +
      "</div>";
    container.appendChild(panel);
    ["click", "dblclick", "mousedown", "mouseup", "touchstart", "touchend", "pointerdown"].forEach(function (evt) {
      panel.addEventListener(evt, function (e) {
        e.stopPropagation();
      });
    });
    panel.addEventListener(
      "wheel",
      function (e) {
        e.stopPropagation();
      },
      { passive: true }
    );

    const state = {
      routeVariants: variantsObj,
      activeRouteId: defaultRouteId,
      points: activeSeed.map(function (p, idx) {
        return { lon: Number(p.lon), lat: Number(p.lat), alt: Number(p.alt), originIdx: idx };
      }),
      original: activeSeed.map(function (p, idx) {
        return { lon: Number(p.lon), lat: Number(p.lat), alt: Number(p.alt), originIdx: idx };
      }),
      selectedIndex: 0,
      editEnabled: false,
      markersReady: false,
      markers: [],
      updateScheduled: false,
      undoStack: [],
      redoStack: [],
      maxHistory: 60,
      draggingIndex: -1,
      routeLayers: {},
      routeLayerWasVisible: {},
    };

    const catalog = findLayerCatalog();
    if (catalog && catalog.overlays) {
      Object.entries(catalog.overlays).forEach(function (entry) {
        const layerName = String(entry[0] || "");
        const layer = entry[1];
        if (!layer) return;
        if (hasAny(layerName, ["安全优先（3D高度）"])) {
          state.routeLayers.safety_default = layer;
          state.routeLayerWasVisible.safety_default = map.hasLayer(layer);
        } else if (hasAny(layerName, ["效率优先（3D高度）"])) {
          state.routeLayers.efficiency = layer;
          state.routeLayerWasVisible.efficiency = map.hasLayer(layer);
        }
      });
    }

    const editLayer = L.featureGroup();
    const markerLayer = L.layerGroup();
    const editLine = L.polyline([], {
      color: "#0f62ad",
      weight: 4,
      opacity: 0.92,
      dashArray: "9 7",
    }).addTo(editLayer);
    editLayer.addTo(map);
    if (!Array.isArray(map._routeDynamicOverlays)) {
      map._routeDynamicOverlays = [];
    }
    map._routeDynamicOverlays = map._routeDynamicOverlays.filter(function (item) {
      return !(item && item.name === "手动编辑结果");
    });
    map._routeDynamicOverlays.push({ name: "手动编辑结果", layer: editLayer });

    const ui = {
      panel: panel,
      panelToggle: panel.querySelector('[data-role="editor-panel-toggle"]'),
      routeBaseSelect: panel.querySelector('[data-role="route-base-select"]'),
      editToggle: panel.querySelector('[data-role="edit-mode-toggle"]'),
      summary: panel.querySelector('[data-role="summary"]'),
      hint: panel.querySelector('[data-role="edit-hint"]'),
      undoChange: panel.querySelector('[data-role="undo-change"]'),
      redoChange: panel.querySelector('[data-role="redo-change"]'),
      pointSelect: panel.querySelector('[data-role="point-select"]'),
      index: panel.querySelector('[data-role="point-index"]'),
      prev: panel.querySelector('[data-role="point-prev"]'),
      next: panel.querySelector('[data-role="point-next"]'),
      advancedToggle: panel.querySelector('[data-role="advanced-toggle"]'),
      advancedWrap: panel.querySelector('[data-role="advanced-wrap"]'),
      lon: panel.querySelector('[data-role="point-lon"]'),
      lat: panel.querySelector('[data-role="point-lat"]'),
      alt: panel.querySelector('[data-role="point-alt"]'),
      altMinus5: panel.querySelector('[data-role="alt-minus-5"]'),
      altPlus5: panel.querySelector('[data-role="alt-plus-5"]'),
      altPlus20: panel.querySelector('[data-role="alt-plus-20"]'),
      restorePoint: panel.querySelector('[data-role="restore-point"]'),
      restoreAll: panel.querySelector('[data-role="restore-all"]'),
      locate: panel.querySelector('[data-role="locate-point"]'),
      kmlName: panel.querySelector('[data-role="kml-name"]'),
      saveKml: panel.querySelector('[data-role="save-kml"]'),
      status: panel.querySelector('[data-role="editor-status"]'),
    };

    const defaultKmlName = String(routeEditorName || "edited_route")
      .trim()
      .replace(/\\s+/g, "_")
      .replace(/[\\\\/:*?"<>|]/g, "_");
    if (ui.kmlName) ui.kmlName.value = (defaultKmlName || "edited_route") + "_manual.kml";

    function setPanelCollapsed(collapsed) {
      const next = !!collapsed;
      panel.classList.toggle("is-collapsed", next);
      if (ui.panelToggle) ui.panelToggle.textContent = next ? "展开" : "收起";
      triggerPanelLayout();
    }

    function setAdvancedCollapsed(collapsed) {
      if (!ui.advancedWrap || !ui.advancedToggle) return;
      const next = !!collapsed;
      ui.advancedWrap.classList.toggle("is-collapsed", next);
      ui.advancedToggle.textContent = next ? "展开高级调整" : "收起高级调整";
    }

    setAdvancedCollapsed(true);
    if (window.matchMedia("(max-width: 768px)").matches) {
      setPanelCollapsed(true);
    } else {
      setPanelCollapsed(false);
    }

    function clonePoint(p) {
      const originIdx = p && Number.isInteger(p.originIdx) ? Number(p.originIdx) : null;
      return { lon: Number(p.lon), lat: Number(p.lat), alt: Number(p.alt), originIdx: originIdx };
    }

    function updateRouteBaseOptions() {
      if (!ui.routeBaseSelect) return;
      ui.routeBaseSelect.innerHTML = "";
      variantIds.forEach(function (rid) {
        const item = state.routeVariants[rid] || {};
        const option = document.createElement("option");
        option.value = String(rid);
        option.textContent = String(item.label || rid);
        ui.routeBaseSelect.appendChild(option);
      });
      ui.routeBaseSelect.value = String(state.activeRouteId || defaultRouteId);
    }

    function pointsEqual(a, b) {
      if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
      for (let i = 0; i < a.length; i += 1) {
        const p1 = a[i];
        const p2 = b[i];
        if (
          !p1 ||
          !p2 ||
          Math.abs(Number(p1.lon) - Number(p2.lon)) > 1e-9 ||
          Math.abs(Number(p1.lat) - Number(p2.lat)) > 1e-9 ||
          Math.abs(Number(p1.alt) - Number(p2.alt)) > 1e-9 ||
          Number(p1.originIdx ?? -1) !== Number(p2.originIdx ?? -1)
        ) {
          return false;
        }
      }
      return true;
    }

    function snapshotPoints() {
      return state.points.map(function (p) {
        return clonePoint(p);
      });
    }

    function pointChanged(index) {
      const cur = state.points[index];
      if (!cur) return false;
      const originIdx = Number.isInteger(cur.originIdx) ? Number(cur.originIdx) : -1;
      if (originIdx < 0 || originIdx >= state.original.length) return true;
      const ori = state.original[originIdx];
      if (!ori) return true;
      return (
        Math.abs(cur.lon - ori.lon) > 1e-9 ||
        Math.abs(cur.lat - ori.lat) > 1e-9 ||
        Math.abs(cur.alt - ori.alt) > 1e-9
      );
    }

    function updateHistoryButtons() {
      if (ui.undoChange) ui.undoChange.disabled = state.undoStack.length === 0;
      if (ui.redoChange) ui.redoChange.disabled = state.redoStack.length === 0;
    }

    function pushHistory() {
      const snap = snapshotPoints();
      const prev = state.undoStack[state.undoStack.length - 1];
      if (prev && pointsEqual(prev, snap)) return;
      state.undoStack.push(snap);
      if (state.undoStack.length > state.maxHistory) {
        state.undoStack.shift();
      }
      state.redoStack = [];
      updateHistoryButtons();
    }

    function applySnapshot(snapshot) {
      if (!Array.isArray(snapshot) || !snapshot.length) return;
      state.points = snapshot.map(function (p) {
        return clonePoint(p);
      });
      state.selectedIndex = Math.max(0, Math.min(state.points.length - 1, state.selectedIndex));
      rebuildEditableLayers();
      scheduleLineRefresh();
      refreshSelectedFields();
    }

    function switchRouteBase(routeId) {
      const rid = String(routeId || "");
      if (variantIds.indexOf(rid) < 0) return;
      if (rid === state.activeRouteId) return;
      const seed = state.routeVariants[rid] && Array.isArray(state.routeVariants[rid].points)
        ? state.routeVariants[rid].points
        : null;
      if (!seed || seed.length < 2) return;
      state.activeRouteId = rid;
      state.points = seed.map(function (p, idx) {
        return { lon: Number(p.lon), lat: Number(p.lat), alt: Number(p.alt), originIdx: idx };
      });
      state.original = seed.map(function (p, idx) {
        return { lon: Number(p.lon), lat: Number(p.lat), alt: Number(p.alt), originIdx: idx };
      });
      state.selectedIndex = 0;
      state.undoStack = [];
      state.redoStack = [];
      rebuildEditableLayers();
      scheduleLineRefresh();
      refreshSelectedFields();
      updateHistoryButtons();
      updateRouteBaseOptions();
      state.routeLayerWasVisible[rid] = !!(state.routeLayers[rid] && map.hasLayer(state.routeLayers[rid]));
      updateStatus("已切换编辑基线: " + String((state.routeVariants[rid] || {}).label || rid));
    }

    function markerIcon(index, active) {
      const changed = pointChanged(index);
      let cls = "route-waypoint-marker";
      if (changed) cls += " is-dirty";
      if (active) cls += " is-active";
      return L.divIcon({
        className: cls,
        html: String(index + 1),
        iconSize: [28, 28],
        iconAnchor: [14, 14],
      });
    }

    function changedCount() {
      let count = 0;
      const seenOriginal = new Set();
      state.points.forEach(function (p) {
        const originIdx = Number.isInteger(p.originIdx) ? Number(p.originIdx) : -1;
        if (originIdx < 0 || originIdx >= state.original.length) {
          count += 1;
          return;
        }
        seenOriginal.add(originIdx);
        const b = state.original[originIdx];
        if (!b) {
          count += 1;
          return;
        }
        if (
          Math.abs(p.lon - b.lon) > 1e-9 ||
          Math.abs(p.lat - b.lat) > 1e-9 ||
          Math.abs(p.alt - b.alt) > 1e-9
        ) {
          count += 1;
        }
      });
      for (let i = 0; i < state.original.length; i += 1) {
        if (!seenOriginal.has(i)) count += 1;
      }
      return count;
    }

    function updateStatus(text) {
      if (ui.status) ui.status.textContent = String(text || "");
    }

    function refreshPointSelectOptions() {
      if (!ui.pointSelect) return;
      ui.pointSelect.innerHTML = "";
      state.points.forEach(function (p, idx) {
        const option = document.createElement("option");
        option.value = String(idx);
        const isInserted = !Number.isInteger(p.originIdx);
        option.textContent =
          "航点 " +
          (idx + 1) +
          (isInserted ? " (新增)" : "") +
          " | " +
          p.lat.toFixed(4) +
          ", " +
          p.lon.toFixed(4) +
          " | " +
          p.alt.toFixed(1) +
          "m";
        ui.pointSelect.appendChild(option);
      });
      ui.pointSelect.value = String(state.selectedIndex);
    }

    function syncLine() {
      const latlngs = state.points.map(function (p) {
        return [p.lat, p.lon];
      });
      editLine.setLatLngs(latlngs);
    }

    function scheduleLineRefresh() {
      if (state.updateScheduled) return;
      state.updateScheduled = true;
      window.requestAnimationFrame(function () {
        state.updateScheduled = false;
        syncLine();
        refreshSummary();
      });
    }

    function refreshMarkerStyles() {
      if (!state.markersReady) return;
      state.markers.forEach(function (mk, idx) {
        mk.setIcon(markerIcon(idx, idx === state.selectedIndex));
      });
    }

    function ensureMarkers() {
      if (state.markersReady) return;
      state.markers = state.points.map(function (p, idx) {
        const marker = L.marker([p.lat, p.lon], {
          draggable: true,
          icon: markerIcon(idx, false),
          keyboard: false,
          autoPan: true,
        });
        marker.on("dragstart", function () {
          state.draggingIndex = idx;
          document.body.classList.add("route-dragging");
          pushHistory();
        });
        marker.on("drag", function () {
          const ll = marker.getLatLng();
          state.points[idx].lon = ll.lng;
          state.points[idx].lat = ll.lat;
          if (idx === state.selectedIndex) {
            refreshSelectedFields();
          }
          scheduleLineRefresh();
        });
        marker.on("dragend", function () {
          const ll = marker.getLatLng();
          state.points[idx].lon = ll.lng;
          state.points[idx].lat = ll.lat;
          state.draggingIndex = -1;
          document.body.classList.remove("route-dragging");
          selectPoint(idx, false);
          updateStatus("已拖动航点 " + (idx + 1) + "，航线已实时更新");
        });
        marker.on("click", function () {
          selectPoint(idx, false);
        });
        marker.bindTooltip("航点 " + (idx + 1), { direction: "top", offset: [0, -6] });
        marker.addTo(markerLayer);
        return marker;
      });
      state.markersReady = true;
      refreshMarkerStyles();
    }

    function nearestSegmentInfo(targetLatLng) {
      if (!targetLatLng || state.points.length < 2) {
        return { index: 0, distPx: Infinity };
      }
      const tp = map.latLngToLayerPoint(targetLatLng);
      let bestIdx = 0;
      let bestDistSq = Infinity;
      for (let i = 0; i < state.points.length - 1; i += 1) {
        const a = map.latLngToLayerPoint([state.points[i].lat, state.points[i].lon]);
        const b = map.latLngToLayerPoint([state.points[i + 1].lat, state.points[i + 1].lon]);
        const abx = b.x - a.x;
        const aby = b.y - a.y;
        const lenSq = abx * abx + aby * aby;
        let t = 0.0;
        if (lenSq > 1e-9) {
          t = ((tp.x - a.x) * abx + (tp.y - a.y) * aby) / lenSq;
          t = Math.max(0.0, Math.min(1.0, t));
        }
        const px = a.x + t * abx;
        const py = a.y + t * aby;
        const dx = tp.x - px;
        const dy = tp.y - py;
        const distSq = dx * dx + dy * dy;
        if (distSq < bestDistSq) {
          bestDistSq = distSq;
          bestIdx = i;
        }
      }
      return { index: bestIdx, distPx: Math.sqrt(bestDistSq) };
    }

    function insertWaypointAtSegment(segIdx, ll, reasonText) {
      const safeSeg = Math.max(0, Math.min(state.points.length - 2, Number(segIdx) || 0));
      const left = state.points[safeSeg];
      const right = state.points[safeSeg + 1];
      const insertAlt = left && right ? (Number(left.alt) + Number(right.alt)) * 0.5 : 60.0;
      const insertIdx = safeSeg + 1;
      state.points.splice(insertIdx, 0, { lon: ll.lng, lat: ll.lat, alt: insertAlt, originIdx: null });
      rebuildEditableLayers();
      scheduleLineRefresh();
      selectPoint(insertIdx, false);
      updateStatus("已" + String(reasonText || "插入") + "新航点 " + (insertIdx + 1));
    }

    function rebuildEditableLayers() {
      markerLayer.clearLayers();
      state.markers = [];
      state.markersReady = false;
      ensureMarkers();
      if (state.editEnabled) {
        if (!map.hasLayer(markerLayer)) map.addLayer(markerLayer);
      }
    }

    map.on("click", function (event) {
      if (!state.editEnabled || !event || !event.latlng) return;
      if (state.draggingIndex >= 0) return;
      const targetEl = event.originalEvent && event.originalEvent.target;
      if (targetEl && typeof targetEl.closest === "function" && targetEl.closest(".route-waypoint-marker")) {
        return;
      }
      const nearest = nearestSegmentInfo(event.latlng);
      if (!nearest || !Number.isFinite(nearest.distPx) || nearest.distPx > 18) return;
      pushHistory();
      insertWaypointAtSegment(nearest.index, event.latlng, "点击航线新增");
    });

    function refreshSummary() {
      if (!ui.summary) return;
      const changed = changedCount();
      ui.summary.textContent =
        "航点 " +
        (state.selectedIndex + 1) +
        " / " +
        state.points.length +
        " | 已修改 " +
        changed +
        " 个";
    }

    function refreshSelectedFields() {
      const point = state.points[state.selectedIndex];
      if (!point) return;
      if (ui.index) ui.index.value = String(state.selectedIndex + 1);
      if (ui.lon) ui.lon.value = point.lon.toFixed(6);
      if (ui.lat) ui.lat.value = point.lat.toFixed(6);
      if (ui.alt) ui.alt.value = point.alt.toFixed(1);
      refreshPointSelectOptions();
      refreshSummary();
      refreshMarkerStyles();
    }

    function selectPoint(index, panToPoint) {
      const idx = Math.max(0, Math.min(state.points.length - 1, Number(index) || 0));
      state.selectedIndex = idx;
      refreshSelectedFields();
      if (panToPoint) {
        const p = state.points[idx];
        map.panTo([p.lat, p.lon], { animate: true, duration: 0.35 });
      }
    }

    function syncMarkerPosition(index) {
      if (!state.markersReady) return;
      const marker = state.markers[index];
      const point = state.points[index];
      if (marker && point) marker.setLatLng([point.lat, point.lon]);
    }

    function applyCurrentPointFromInputs() {
      const idx = state.selectedIndex;
      const point = state.points[idx];
      if (!point) return;
      const lon = Number(ui.lon && ui.lon.value);
      const lat = Number(ui.lat && ui.lat.value);
      const alt = Number(ui.alt && ui.alt.value);
      if (!Number.isFinite(lon) || !Number.isFinite(lat) || !Number.isFinite(alt)) {
        updateStatus("经纬度或高度格式无效，请输入数值");
        return;
      }
      const unchanged =
        Math.abs(point.lon - lon) < 1e-9 &&
        Math.abs(point.lat - lat) < 1e-9 &&
        Math.abs(point.alt - alt) < 1e-9;
      if (unchanged) return;
      pushHistory();
      point.lon = lon;
      point.lat = lat;
      point.alt = alt;
      syncMarkerPosition(idx);
      scheduleLineRefresh();
      refreshSelectedFields();
      updateStatus("已应用航点 " + (idx + 1) + " 的经纬度和高度");
    }

    function nudgeCurrentAltitude(delta) {
      const idx = state.selectedIndex;
      const point = state.points[idx];
      if (!point) return;
      if (!Number.isFinite(Number(delta)) || Number(delta) === 0) return;
      pushHistory();
      point.alt = Math.max(0.0, Number(point.alt) + Number(delta || 0));
      if (ui.alt) ui.alt.value = point.alt.toFixed(1);
      scheduleLineRefresh();
      refreshSelectedFields();
      updateStatus("已调整航点 " + (idx + 1) + " 高度 " + (delta >= 0 ? "+" : "") + Number(delta).toFixed(1) + "m");
    }

    function undoChange() {
      if (!state.undoStack.length) return;
      const current = snapshotPoints();
      const prev = state.undoStack.pop();
      state.redoStack.push(current);
      applySnapshot(prev);
      updateHistoryButtons();
      updateStatus("已撤销上一步修改");
    }

    function redoChange() {
      if (!state.redoStack.length) return;
      const current = snapshotPoints();
      const next = state.redoStack.pop();
      state.undoStack.push(current);
      applySnapshot(next);
      updateHistoryButtons();
      updateStatus("已重做上一步修改");
    }

    function restoreCurrentPoint() {
      const idx = state.selectedIndex;
      const point = state.points[idx];
      if (!point) return;
      const originIdx = Number.isInteger(point.originIdx) ? Number(point.originIdx) : -1;
      const origin = originIdx >= 0 && originIdx < state.original.length ? state.original[originIdx] : null;
      if (!origin) {
        if (state.points.length <= 2) return;
        pushHistory();
        state.points.splice(idx, 1);
        const targetIdx = Math.max(0, Math.min(state.points.length - 1, idx - 1));
        rebuildEditableLayers();
        scheduleLineRefresh();
        selectPoint(targetIdx, false);
        updateStatus("已删除新增航点，回到初始航线结构");
        return;
      }
      pushHistory();
      state.points[idx] = clonePoint(origin);
      syncMarkerPosition(idx);
      scheduleLineRefresh();
      refreshSelectedFields();
      updateStatus("已还原航点 " + (idx + 1));
    }

    function restoreAllPoints() {
      pushHistory();
      state.points = state.original.map(function (p) {
        return clonePoint(p);
      });
      rebuildEditableLayers();
      scheduleLineRefresh();
      refreshSelectedFields();
      updateStatus("已还原全部航点");
    }

    function setEditMode(enabled) {
      const next = !!enabled;
      if (state.editEnabled === next) return;
      state.editEnabled = next;
      if (next) {
        ensureMarkers();
        if (!map.hasLayer(editLayer)) map.addLayer(editLayer);
        if (!map.hasLayer(markerLayer)) map.addLayer(markerLayer);
        editLine.setStyle({ weight: 5, opacity: 0.95, dashArray: null });
        Object.keys(state.routeLayers).forEach(function (rid) {
          const layer = state.routeLayers[rid];
          if (!layer) return;
          state.routeLayerWasVisible[rid] = map.hasLayer(layer);
          if (map.hasLayer(layer)) map.removeLayer(layer);
        });
        if (ui.editToggle) ui.editToggle.textContent = "关闭拖拽编辑";
        if (ui.hint) ui.hint.textContent = "拖动航点可改位置；点击蓝色航线可新增航点；支持撤销/重做。";
        updateStatus("编辑已开启：拖动航点，或点击航线插入新航点");
      } else {
        if (map.hasLayer(markerLayer)) map.removeLayer(markerLayer);
        editLine.setStyle({ weight: 4, opacity: 0.88, dashArray: "9 7" });
        Object.keys(state.routeLayers).forEach(function (rid) {
          const layer = state.routeLayers[rid];
          if (!layer) return;
          if (state.routeLayerWasVisible[rid] && !map.hasLayer(layer)) {
            map.addLayer(layer);
          }
        });
        if (ui.editToggle) ui.editToggle.textContent = "开启拖拽编辑";
        if (ui.hint) ui.hint.textContent = "开启后可拖动编号航点，航线会实时跟随更新。";
      }
    }

    function exportKml() {
      const rawName = (ui.kmlName && ui.kmlName.value) || "";
      const fileName = String(rawName || "edited_route.kml")
        .trim()
        .replace(/[\\\\/:*?"<>|]/g, "_")
        .replace(/\\s+/g, "_");
      const safeName = fileName.toLowerCase().endsWith(".kml") ? fileName : fileName + ".kml";
      const routeName = safeName.replace(/\\.kml$/i, "");
      const coords = state.points
        .map(function (p) {
          return p.lon.toFixed(6) + "," + p.lat.toFixed(6) + "," + p.alt.toFixed(2);
        })
        .join(" ");
      const kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        "<Document>",
        "<name>" + routeName + "</name>",
        "<Placemark>",
        "<name>" + routeName + "</name>",
        "<Style><LineStyle><color>ff2a6df4</color><width>4</width></LineStyle></Style>",
        "<LineString>",
        "<extrude>1</extrude>",
        "<tessellate>1</tessellate>",
        "<altitudeMode>absolute</altitudeMode>",
        "<coordinates>" + coords + "</coordinates>",
        "</LineString>",
        "</Placemark>",
        "</Document>",
        "</kml>",
      ].join("");
      const blob = new Blob([kml], { type: "application/vnd.google-earth.kml+xml;charset=utf-8" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = safeName;
      document.body.appendChild(a);
      a.click();
      window.setTimeout(function () {
        URL.revokeObjectURL(url);
        a.remove();
      }, 0);
      updateStatus("已导出新KML: " + safeName);
    }

    if (ui.panelToggle) {
      ui.panelToggle.addEventListener("click", function (event) {
        event.preventDefault();
        event.stopPropagation();
        const collapsed = !panel.classList.contains("is-collapsed");
        setPanelCollapsed(collapsed);
      });
    }
    if (ui.editToggle) {
      ui.editToggle.addEventListener("click", function () {
        setEditMode(!state.editEnabled);
      });
    }
    if (ui.prev) {
      ui.prev.addEventListener("click", function () {
        selectPoint(state.selectedIndex - 1, true);
      });
    }
    if (ui.next) {
      ui.next.addEventListener("click", function () {
        selectPoint(state.selectedIndex + 1, true);
      });
    }
    if (ui.index) {
      ui.index.addEventListener("change", function () {
        const val = Number(ui.index.value);
        if (!Number.isFinite(val)) return;
        selectPoint(val - 1, true);
      });
    }
    if (ui.pointSelect) {
      ui.pointSelect.addEventListener("change", function () {
        const val = Number(ui.pointSelect.value);
        if (!Number.isFinite(val)) return;
        selectPoint(val, true);
      });
    }
    if (ui.routeBaseSelect) {
      ui.routeBaseSelect.addEventListener("change", function () {
        switchRouteBase(ui.routeBaseSelect.value);
      });
    }
    if (ui.advancedToggle) {
      ui.advancedToggle.addEventListener("click", function () {
        if (!ui.advancedWrap) return;
        const nextCollapsed = !ui.advancedWrap.classList.contains("is-collapsed");
        setAdvancedCollapsed(nextCollapsed);
      });
    }
    if (ui.undoChange) {
      ui.undoChange.addEventListener("click", function () {
        undoChange();
      });
    }
    if (ui.redoChange) {
      ui.redoChange.addEventListener("click", function () {
        redoChange();
      });
    }
    if (ui.altMinus5) {
      ui.altMinus5.addEventListener("click", function () {
        nudgeCurrentAltitude(-5.0);
      });
    }
    if (ui.altPlus5) {
      ui.altPlus5.addEventListener("click", function () {
        nudgeCurrentAltitude(5.0);
      });
    }
    if (ui.altPlus20) {
      ui.altPlus20.addEventListener("click", function () {
        nudgeCurrentAltitude(20.0);
      });
    }
    [ui.lon, ui.lat, ui.alt].forEach(function (input) {
      if (!input) return;
      input.addEventListener("change", function () {
        applyCurrentPointFromInputs();
      });
      input.addEventListener("keydown", function (event) {
        if (event.key === "Enter") {
          event.preventDefault();
          applyCurrentPointFromInputs();
        }
      });
    });
    if (ui.restorePoint) {
      ui.restorePoint.addEventListener("click", function () {
        restoreCurrentPoint();
      });
    }
    if (ui.restoreAll) {
      ui.restoreAll.addEventListener("click", function () {
        restoreAllPoints();
      });
    }
    if (ui.locate) {
      ui.locate.addEventListener("click", function () {
        selectPoint(state.selectedIndex, true);
      });
    }
    if (ui.saveKml) {
      ui.saveKml.addEventListener("click", function () {
        exportKml();
      });
    }

    syncLine();
    updateRouteBaseOptions();
    rebuildEditableLayers();
    selectPoint(0, false);
    updateHistoryButtons();
    setEditMode(false);
    updateStatus("可先开启拖拽编辑，再拖动航点或点击航线插入新航点");
    map._routeEditorReady = true;
    triggerPanelLayout();
    return true;
  }

  function boot() {
    setupProfilePanel();
    if (typeof L === "undefined") return false;
    const map = Object.values(window).find(function (value) {
      return value && value instanceof L.Map;
    });
    if (!map) return false;
    setupPanelLayout(map);
    const readyEditor = setupRouteEditor(map);
    triggerPanelLayout();
    return !!readyEditor;
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
    return (
        template.replace("__ROUTE_VARIANTS_JSON__", route_variants_json)
        .replace("__ROUTE_DEFAULT_ID_JSON__", default_variant_json)
        .replace("__ROUTE_NAME_JSON__", route_name_json)
    )


def write_preview_html(
    path: Path,
    route_points: List[Tuple[float, float, float]],
    candidate_routes_wgs: List[Dict[str, Any]],
    start_wgs: Tuple[float, float],
    end_wgs: Tuple[float, float],
    civil_airport_polys_xy: List[Any],
    military_hard_nofly_polys_xy: List[Any],
    heli_soft_nofly_polys_xy: List[Any],
    route_buffer_xy,
    dynamic_grb_xy,
    population_points_wgs: List[Dict[str, float]],
    population_tiles_template: str,
    population_tiles_min_zoom: int,
    population_tiles_max_native_zoom: int,
    landuse_geoms_xy: List[Any],
    landuse_costs: List[float],
    all_building_polys_xy: List[Any],
    high_building_polys_xy: List[Any],
    crowd_points_xy: List[Any],
    key_points_xy: List[Any],
    infra_geoms_xy: List[Any],
    high_road_lines_xy: List[Any],
    hsr_lines_xy: List[Any],
    hv_power_lines_xy: List[Any],
    line_risk_union_xy,
    school_hard_zones_xy: List[Any],
    school_points_xy: List[Any],
    school_point_tooltips_xy: Dict[Tuple[float, float], str],
    crowd_point_tooltips_xy: Dict[Tuple[float, float], str],
    key_point_tooltips_xy: Dict[Tuple[float, float], str],
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

    def _point_tooltip_from_lookup(pt_xy, lookup: Dict[Tuple[float, float], str], fallback: str) -> str:
        try:
            key = _round_key((float(pt_xy.x), float(pt_xy.y)), snap_m=8.0)
            if key in lookup:
                return str(lookup[key])
        except Exception:
            pass
        return fallback

    def _landuse_color(cost: float) -> str:
        c = float(cost)
        if c <= 0.25:
            return "#1e88e5"
        if c <= 0.6:
            return "#2e7d32"
        if c <= 1.2:
            return "#8bc34a"
        if c <= 1.6:
            return "#ffb300"
        return "#e53935"

    def _add_landuse_layer(
        layer: folium.FeatureGroup,
        geoms_xy: List[Any],
        costs: List[float],
    ) -> None:
        count = min(len(geoms_xy), len(costs))
        for i in range(count):
            geom_xy = geoms_xy[i]
            color = _landuse_color(costs[i])
            try:
                geom_wgs = transform(inv, geom_xy)
                if geom_wgs.geom_type == "Polygon":
                    folium.Polygon(
                        _poly_to_latlon(geom_wgs),
                        color=color,
                        weight=1,
                        fill=True,
                        fill_opacity=0.08,
                    ).add_to(layer)
                elif geom_wgs.geom_type == "MultiPolygon":
                    for p in geom_wgs.geoms:
                        folium.Polygon(
                            _poly_to_latlon(p),
                            color=color,
                            weight=1,
                            fill=True,
                            fill_opacity=0.08,
                        ).add_to(layer)
            except Exception:
                continue

    center_lat = (start_wgs[1] + end_wgs[1]) / 2.0
    center_lon = (start_wgs[0] + end_wgs[0]) / 2.0
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=12,
        control_scale=True,
        tiles=None,
        prefer_canvas=True,
        max_zoom=22,
    )

    folium.TileLayer(
        tiles="OpenStreetMap",
        name="普通地图",
        overlay=False,
        control=True,
        show=True,
        max_native_zoom=19,
        max_zoom=22,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png",
        attr='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors '
        '&copy; <a href="https://carto.com/">CARTO</a>',
        name="浅色底图",
        overlay=False,
        control=True,
        show=False,
        max_native_zoom=20,
        max_zoom=20,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Tiles &copy; Esri",
        name="卫星影像",
        overlay=False,
        control=True,
        show=False,
        max_native_zoom=19,
        max_zoom=22,
    ).add_to(m)
    folium.TileLayer(
        tiles="https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}",
        attr="Labels &copy; Esri",
        name="卫星注记",
        overlay=True,
        control=True,
        show=False,
        opacity=0.9,
        max_native_zoom=19,
        max_zoom=22,
    ).add_to(m)

    route_style = {
        "safety_default": {
            "line_color": "#1565c0",
            "marker_color": "#0d47a1",
            "layer_name": "安全优先（3D高度）",
            "buffer_name": f"安全优先 缓冲区 {int(ROUTE_BUFFER_M)}m",
            "buffer_color": "#1e88e5",
            "fallback_label": "安全优先",
        },
        "efficiency": {
            "line_color": "#ef6c00",
            "marker_color": "#e65100",
            "layer_name": "效率优先（3D高度）",
            "buffer_name": f"效率优先 缓冲区 {int(ROUTE_BUFFER_M)}m",
            "buffer_color": "#fb8c00",
            "fallback_label": "效率优先",
        },
    }
    route_variants_for_editor: Dict[str, Dict[str, Any]] = {}
    profile_variants_for_panel: Dict[str, Dict[str, Any]] = {}
    default_variant_id = "safety_default"
    for cand in candidate_routes_wgs:
        cid = str(cand.get("id", "candidate"))
        style = route_style.get(cid)
        if not style:
            continue
        alt_points = cand.get("alt_points") or []
        points_for_editor: List[Dict[str, float]] = []
        coords_2d: List[Tuple[float, float]] = []
        alts: List[float] = []
        for pt in alt_points:
            try:
                lon = float(pt[0])
                lat = float(pt[1])
                alt = float(pt[2])
            except Exception:
                continue
            points_for_editor.append({"lon": lon, "lat": lat, "alt": alt})
            coords_2d.append((lat, lon))
            alts.append(alt)
        if len(points_for_editor) < 2:
            coords = cand.get("coords") or []
            for lat, lon in coords:
                try:
                    points_for_editor.append({"lon": float(lon), "lat": float(lat), "alt": 60.0})
                except Exception:
                    continue
            coords_2d = [(float(p["lat"]), float(p["lon"])) for p in points_for_editor]
            alts = [float(p["alt"]) for p in points_for_editor]
        if len(points_for_editor) < 2:
            continue
        if bool(cand.get("show", False)):
            default_variant_id = cid
        label = str(cand.get("label", style["fallback_label"]))
        route_variants_for_editor[cid] = {"label": label, "points": points_for_editor}
        profile_samples_variant = cand.get("profile_samples") or []
        if isinstance(profile_samples_variant, list) and len(profile_samples_variant) >= 2:
            profile_variants_for_panel[cid] = {"label": label, "samples": profile_samples_variant}

        route_layer = folium.FeatureGroup(name=style["layer_name"], show=True)
        distance_km = _safe_float(cand.get("distance_km", 0.0), 0.0)
        tip = f"{label} | {distance_km:.2f} km | 高度最小/最大 {min(alts):.1f}/{max(alts):.1f} m"
        folium.PolyLine(coords_2d, color=style["line_color"], weight=4, opacity=0.92, tooltip=tip).add_to(route_layer)
        step = max(1, len(points_for_editor) // 120)
        for i in range(0, len(points_for_editor), step):
            p = points_for_editor[i]
            folium.CircleMarker(
                [p["lat"], p["lon"]],
                radius=2,
                color=style["marker_color"],
                fill=True,
                fill_opacity=0.8,
                tooltip=f"高度 {p['alt']:.1f} m",
            ).add_to(route_layer)
        route_layer.add_to(m)

        buf_layer = folium.FeatureGroup(name=style["buffer_name"], show=True)
        route_buffer_variant = cand.get("route_buffer_xy")
        if route_buffer_variant is not None and (not route_buffer_variant.is_empty):
            _add_polygon_layer(
                buf_layer,
                [route_buffer_variant],
                color=style["buffer_color"],
                fill_opacity=0.08,
                max_items=1,
            )
        buf_layer.add_to(m)

    if not route_variants_for_editor and len(route_points) >= 2:
        fallback_points = [
            {"lon": float(lon), "lat": float(lat), "alt": float(alt)}
            for lon, lat, alt in route_points
        ]
        route_variants_for_editor["safety_default"] = {"label": "安全优先", "points": fallback_points}
        fallback_route = folium.FeatureGroup(name="安全优先（3D高度）", show=True)
        folium.PolyLine(
            [(float(lat), float(lon)) for lon, lat, _ in route_points],
            color="#1565c0",
            weight=4,
            opacity=0.92,
            tooltip=f"安全优先 | 高度最小/最大 {min(alt for _, _, alt in route_points):.1f}/{max(alt for _, _, alt in route_points):.1f} m",
        ).add_to(fallback_route)
        fallback_route.add_to(m)
        fallback_buf = folium.FeatureGroup(name=f"安全优先 缓冲区 {int(ROUTE_BUFFER_M)}m", show=True)
        if route_buffer_xy is not None and (not route_buffer_xy.is_empty):
            _add_polygon_layer(fallback_buf, [route_buffer_xy], color="#1e88e5", fill_opacity=0.08, max_items=1)
        fallback_buf.add_to(m)
    dyn_grb_layer = folium.FeatureGroup(name="动态 GRB（AGL 1:1）", show=False)
    if dynamic_grb_xy is not None and (not dynamic_grb_xy.is_empty):
        _add_polygon_layer(dyn_grb_layer, [dynamic_grb_xy], color="#26a69a", fill_opacity=0.1, max_items=1)
    dyn_grb_layer.add_to(m)

    if population_tiles_template:
        folium.TileLayer(
            tiles=population_tiles_template,
            name="人口密度",
            attr="Population Density Heatmap",
            overlay=True,
            control=True,
            show=False,
            opacity=0.7,
            min_zoom=max(0, int(population_tiles_min_zoom)),
            max_native_zoom=max(0, int(population_tiles_max_native_zoom)),
            max_zoom=22,
            tms=True,
        ).add_to(m)
    else:
        pop_layer = folium.FeatureGroup(name="人口密度", show=False)
        pop_vals = [float(p.get("value", 0.0)) for p in population_points_wgs if _safe_float(p.get("value", 0.0), 0.0) > 0.0]
        norm_base = _percentile(pop_vals, 0.95) if pop_vals else 0.0
        if norm_base <= 0.0 and pop_vals:
            norm_base = max(pop_vals)
        heat_points: List[List[float]] = []
        for p in population_points_wgs:
            lat = _safe_float(p.get("lat", 0.0), 0.0)
            lon = _safe_float(p.get("lon", 0.0), 0.0)
            val = _safe_float(p.get("value", 0.0), 0.0)
            if val <= 0.0:
                continue
            weight = min(1.0, val / max(1.0, norm_base))
            heat_points.append([lat, lon, max(0.05, weight)])
        if heat_points:
            HeatMap(
                heat_points,
                min_opacity=0.28,
                radius=16,
                blur=14,
                max_zoom=18,
                gradient={0.2: "#64b5f6", 0.4: "#4fc3f7", 0.6: "#ffeb3b", 0.8: "#ff9800", 1.0: "#d32f2f"},
            ).add_to(pop_layer)
        pop_layer.add_to(m)

    civil_airport_layer = folium.FeatureGroup(name="民航机场禁飞区（CAAC）", show=True)
    _add_polygon_layer(civil_airport_layer, civil_airport_polys_xy, color="#7b1fa2", fill_opacity=0.2, max_items=260)
    civil_airport_layer.add_to(m)

    military_nofly_layer = folium.FeatureGroup(name="军用机场禁飞区（硬约束）", show=True)
    _add_polygon_layer(military_nofly_layer, military_hard_nofly_polys_xy, color="#d32f2f", fill_opacity=0.22, max_items=220)
    military_nofly_layer.add_to(m)

    heli_soft_layer = folium.FeatureGroup(name="直升机场避让区（软约束）", show=True)
    _add_polygon_layer(heli_soft_layer, heli_soft_nofly_polys_xy, color="#ef6c00", fill_opacity=0.16, max_items=260)
    heli_soft_layer.add_to(m)

    landuse_layer = folium.FeatureGroup(name="土地利用", show=False)
    _add_landuse_layer(landuse_layer, landuse_geoms_xy, landuse_costs)
    landuse_layer.add_to(m)

    all_build_layer = folium.FeatureGroup(name="全部建筑物", show=False)
    _add_polygon_layer(all_build_layer, all_building_polys_xy, color="#8d6e63", fill_opacity=0.08, max_items=1200)
    all_build_layer.add_to(m)

    high_build_layer = folium.FeatureGroup(name=f"高层建筑（>={int(HIGH_BUILDING_THRESHOLD_M)}m）", show=False)
    _add_polygon_layer(high_build_layer, high_building_polys_xy, color="#ef6c00", fill_opacity=0.22, max_items=700)
    high_build_layer.add_to(m)

    school_layer = folium.FeatureGroup(name="学校/幼儿园避让区", show=False)
    _add_polygon_layer(school_layer, school_hard_zones_xy, color="#c62828", fill_opacity=0.16, max_items=600)
    for pxy in school_points_xy[:800]:
        try:
            pwgs = transform(inv, pxy)
            tip = _point_tooltip_from_lookup(pxy, school_point_tooltips_xy, "名称: 未命名POI | 类型: 学校/幼儿园")
            folium.CircleMarker(
                [pwgs.y, pwgs.x],
                radius=2,
                color="#b71c1c",
                fill=True,
                fill_opacity=0.85,
                tooltip=tip,
            ).add_to(school_layer)
        except Exception:
            continue
    school_layer.add_to(m)

    crowd_layer = folium.FeatureGroup(name="人群聚集 POI", show=False)
    for pxy in crowd_points_xy[:1200]:
        try:
            pwgs = transform(inv, pxy)
            tip = _point_tooltip_from_lookup(pxy, crowd_point_tooltips_xy, "名称: 未命名POI | 类型: 人群聚集点")
            folium.CircleMarker(
                [pwgs.y, pwgs.x],
                radius=2,
                color="#ad1457",
                fill=True,
                fill_opacity=0.7,
                tooltip=tip,
            ).add_to(crowd_layer)
        except Exception:
            continue
    crowd_layer.add_to(m)

    key_layer = folium.FeatureGroup(name="敏感设施 POI", show=False)
    for pxy in key_points_xy[:800]:
        try:
            pwgs = transform(inv, pxy)
            tip = _point_tooltip_from_lookup(pxy, key_point_tooltips_xy, "名称: 未命名POI | 类型: 敏感设施")
            folium.CircleMarker(
                [pwgs.y, pwgs.x],
                radius=3,
                color="#6a1b9a",
                fill=True,
                fill_opacity=0.85,
                tooltip=tip,
            ).add_to(key_layer)
        except Exception:
            continue
    key_layer.add_to(m)

    infra_layer = folium.FeatureGroup(name="关键基础设施", show=False)
    _add_polygon_layer(infra_layer, infra_geoms_xy, color="#455a64", fill_opacity=0.14, max_items=500)
    infra_layer.add_to(m)

    line_layer = folium.FeatureGroup(name="线性风险（高速/高铁/高压电力线）", show=False)
    _add_line_layer(line_layer, high_road_lines_xy, color="#b71c1c", max_items=800)
    _add_line_layer(line_layer, hsr_lines_xy, color="#1b5e20", max_items=500)
    _add_line_layer(line_layer, hv_power_lines_xy, color="#ff8f00", max_items=600)
    if line_risk_union_xy is not None and (not line_risk_union_xy.is_empty):
        _add_polygon_layer(line_layer, [line_risk_union_xy], color="#8d6e63", fill_opacity=0.1, max_items=1)
    line_layer.add_to(m)

    start_end_layer = folium.FeatureGroup(name="起点/终点", show=True)
    folium.Marker([start_wgs[1], start_wgs[0]], tooltip="起点", icon=folium.Icon(color="green")).add_to(start_end_layer)
    folium.Marker([end_wgs[1], end_wgs[0]], tooltip="终点", icon=folium.Icon(color="red")).add_to(start_end_layer)
    start_end_layer.add_to(m)

    folium.LayerControl(collapsed=False).add_to(m)
    html_text = m.get_root().render()
    if "</head>" in html_text:
        html_text = html_text.replace("</head>", _build_preview_theme_head_html() + "\n</head>")
    panel_html = _build_profile_panel_html(
        name=name,
        profile_variants=profile_variants_for_panel,
        default_profile_variant_id=default_variant_id,
    )
    if panel_html and "</body>" in html_text:
        html_text = html_text.replace("</body>", panel_html + "\n</body>")
    if "</html>" in html_text:
        html_text = html_text.replace(
            "</html>",
            _build_preview_toolbar_script_html(
                name=name,
                route_variants=route_variants_for_editor,
                default_route_variant_id=default_variant_id,
            )
            + "\n</html>",
        )
    path.write_text(html_text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Automatic OD route planner for urban UAV logistics.")
    parser.add_argument("--city", required=True, help="City name for cache and context.")
    parser.add_argument("--start-lon", type=float, default=None)
    parser.add_argument("--start-lat", type=float, default=None)
    parser.add_argument("--end-lon", type=float, default=None)
    parser.add_argument("--end-lat", type=float, default=None)
    parser.add_argument("--od-kml", default="", help="Optional KML. Use first and last points as OD.")
    parser.add_argument(
        "--reference-kml",
        default="",
        help="Optional reference route KML for minimal-change replanning bias.",
    )
    parser.add_argument(
        "--reference-corridor-m",
        type=float,
        default=300.0,
        help="Offset normalization corridor (meters) for reference-route deviation penalty.",
    )
    parser.add_argument(
        "--reference-deviation-weight",
        type=float,
        default=1.4,
        help="Global multiplier for reference-route deviation penalty.",
    )
    parser.add_argument(
        "--reference-max-detour-ratio",
        type=float,
        default=1.2,
        help="Hard filter: max candidate distance ratio versus reference route length.",
    )
    parser.add_argument(
        "--reference-max-mean-offset-m",
        type=float,
        default=180.0,
        help="Hard filter: max mean offset distance (meters) versus reference route.",
    )
    parser.add_argument("--name", default="auto_route", help="Output route base name.")
    parser.add_argument("--city-zoom", default="8-14")
    parser.add_argument("--profile", choices=["fastest", "balanced", "safest"], default="balanced")
    parser.add_argument(
        "--select-candidate",
        choices=["safety_default", "efficiency"],
        default="safety_default",
        help="Select which internal candidate to export as main route output.",
    )
    parser.add_argument(
        "--weight-sweep-levels",
        type=int,
        default=0,
        help="Deprecated in 2-candidate mode; accepted for compatibility but ignored.",
    )
    parser.add_argument(
        "--pareto-select",
        dest="pareto_select",
        action="store_true",
        default=False,
        help="Allow selecting final route from Pareto front (distance/population/vertical).",
    )
    parser.add_argument(
        "--pareto-detour-limit-ratio",
        type=float,
        default=None,
        help="Optional override: max distance ratio allowed in Pareto-based selection.",
    )
    parser.add_argument(
        "--pareto-distance-weight",
        type=float,
        default=None,
        help="Optional override: distance weight in Pareto selection score.",
    )
    parser.add_argument(
        "--pareto-pop-weight",
        type=float,
        default=None,
        help="Optional override: population p90 weight in Pareto selection score.",
    )
    parser.add_argument(
        "--pareto-energy-weight",
        type=float,
        default=None,
        help="Optional override: vertical energy weight in Pareto selection score.",
    )
    parser.add_argument(
        "--pareto-max-front",
        type=int,
        default=12,
        help="Max Pareto front items to keep in summary output.",
    )
    parser.add_argument(
        "--pareto-policy-file",
        default="",
        help="Optional Pareto policy JSON path. Defaults to skills/plan-auto-route/config/pareto_policies.json.",
    )
    parser.add_argument(
        "--pareto-policy-name",
        default="default",
        help="Base policy name in policy file.",
    )
    parser.add_argument(
        "--pareto-business-line",
        default="default",
        help="Business-line key for policy overrides (e.g., medical/ecommerce/inspection).",
    )
    parser.add_argument("--top-k", type=int, default=1, help="Candidate routes to evaluate (phase-1 summary).")
    parser.add_argument(
        "--open-data-no-fly",
        dest="open_data_no_fly",
        action="store_true",
        default=True,
        help="Enable open-data no-fly: civil airports from CAAC dataset + military/heli tags from OSM.",
    )
    parser.add_argument(
        "--no-open-data-no-fly",
        dest="open_data_no_fly",
        action="store_false",
        help="Disable open-data no-fly filtering.",
    )
    parser.add_argument(
        "--civil-airport-no-fly-geojson",
        default=str(DEFAULT_CIVIL_AIRPORT_NO_FLY_GEOJSON),
        help="Civil airport no-fly dataset (GeoJSON converted from CAAC XLS).",
    )
    parser.add_argument("--soft-no-fly-scale", type=float, default=1.0, help="Scale factor for soft no-fly penalty.")
    parser.add_argument(
        "--infra-hard-buffer-m",
        type=float,
        default=0.0,
        help="Legacy global hard exclusion around key infrastructure (kept for compatibility).",
    )
    parser.add_argument(
        "--safety-sensitive-hard-buffer-m",
        type=float,
        default=SAFETY_SENSITIVE_HARD_BUFFER_M,
        help="Hard exclusion buffer around sensitive facilities for safety route.",
    )
    parser.add_argument(
        "--safety-infra-hard-buffer-m",
        type=float,
        default=SAFETY_INFRA_HARD_BUFFER_M,
        help="Hard exclusion buffer around critical infrastructure for safety route.",
    )
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
    parser.add_argument(
        "--write-evidence",
        dest="write_evidence",
        action="store_true",
        default=True,
        help="Write structured evidence JSON for audit/review.",
    )
    parser.add_argument(
        "--no-write-evidence",
        dest="write_evidence",
        action="store_false",
        help="Disable evidence JSON output.",
    )
    parser.add_argument(
        "--write-snapshot",
        dest="write_snapshot",
        action="store_true",
        default=True,
        help="Write run snapshot manifest for reproducibility.",
    )
    parser.add_argument(
        "--no-write-snapshot",
        dest="write_snapshot",
        action="store_false",
        help="Disable run snapshot output.",
    )
    parser.add_argument(
        "--snapshot-dir",
        default="",
        help="Optional snapshot output dir; default is <out-dir>/snapshots.",
    )
    args = parser.parse_args()
    effective_clearance_m = max(30.0, float(args.clearance_m))

    root = Path(__file__).resolve().parents[3]
    reference_points_wgs: List[Tuple[float, float]] = []
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
    if args.reference_kml:
        ref_coords = parse_kml_coords(Path(args.reference_kml).resolve())
        reference_points_wgs = [(float(lon), float(lat)) for lon, lat, _ in ref_coords]
        if len(reference_points_wgs) < 2:
            raise ValueError("--reference-kml must contain at least two coordinates.")

    ensure_city_data(root, args.city, args.city_zoom)
    cache_dir = _find_city_cache_dir(root, args.city)
    if cache_dir is None:
        raise FileNotFoundError(f"City cache dir not found for: {args.city}")
    summary_path = cache_dir / "download_summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(f"download_summary.json missing for city: {args.city}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    projector_points = [start_wgs, end_wgs] + reference_points_wgs
    fwd, inv = build_projectors(projector_points)
    start_xy = wgs_to_xy(fwd, start_wgs[0], start_wgs[1])
    end_xy = wgs_to_xy(fwd, end_wgs[0], end_wgs[1])
    bbox_points = reference_points_wgs if len(reference_points_wgs) >= 2 else [start_wgs, end_wgs]
    route_bbox = route_bbox_wgs(bbox_points, margin_m=4500.0)
    route_bbox_building = route_bbox_wgs(bbox_points, margin_m=2800.0)
    reference_line_xy = None
    reference_length_m = 0.0
    if len(reference_points_wgs) >= 2:
        reference_xy = [wgs_to_xy(fwd, lon, lat) for lon, lat in reference_points_wgs]
        reference_line_xy = LineString(reference_xy)
        reference_length_m = float(reference_line_xy.length)
    reference_mode = reference_line_xy is not None and reference_length_m > 0

    pop_tif = Path(summary.get("outputs", {}).get("population", {}).get("clipped_tif", ""))
    pop_sampler = PopulationSampler(pop_tif if pop_tif.exists() else None)
    landuse_geoms, landuse_costs, landuse_tree = build_landuse_index(summary, fwd)
    landuse_id_map = _build_geom_id_map(landuse_geoms)
    infra_geoms, infra_severities, infra_tree = build_infrastructure_index(summary, route_bbox, fwd)
    infra_id_map = _build_geom_id_map(infra_geoms)
    (
        crowd_points_xy,
        crowd_tree,
        crowd_id_map,
        key_points_xy,
        key_tree,
        key_id_map,
        poi_risk_counter,
        crowd_poi_items,
        key_poi_items,
    ) = build_poi_risk_indices(summary, fwd)
    high_road_lines_xy, line_risk_union_xy, hsr_lines_xy, hv_power_lines_xy, line_risk_counter = build_line_risk_geometries(
        summary, route_bbox, fwd
    )
    school_hard_zones_xy, school_points_xy, school_counter, school_poi_items = fetch_school_kindergarten_zones(route_bbox, fwd)
    crowd_display_items: List[Dict[str, Any]] = list(crowd_poi_items)
    if school_points_xy:
        crowd_points_xy.extend(school_points_xy)
        crowd_display_items.extend(school_poi_items)
        poi_risk_counter["crowd"] = int(poi_risk_counter.get("crowd", 0) + len(school_points_xy))
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
    nofly_military_hard_polys_xy: List[Any] = []
    nofly_heli_soft_polys_xy: List[Any] = []
    civil_airport_route_scope_polys_xy: List[Any] = []
    civil_airport_display_polys_xy: List[Any] = []
    nofly_counter: Dict[str, Any] = {"civil_airport": 0, "military_airport": 0, "hard": 0, "soft": 0}
    city_bbox_info = summary.get("bbox", {}) if isinstance(summary.get("bbox", {}), dict) else {}
    city_bbox = (
        _safe_float(city_bbox_info.get("south", route_bbox[0])),
        _safe_float(city_bbox_info.get("north", route_bbox[1])),
        _safe_float(city_bbox_info.get("west", route_bbox[2])),
        _safe_float(city_bbox_info.get("east", route_bbox[3])),
    )
    if args.open_data_no_fly:
        nofly_hard_polys_xy, nofly_soft_polys_xy, nofly_counter, civil_airport_route_scope_polys_xy, nofly_military_hard_polys_xy, nofly_heli_soft_polys_xy = fetch_open_data_no_fly_zones(
            route_bbox,
            fwd,
            civil_airport_geojson=args.civil_airport_no_fly_geojson,
        )
        civil_airport_display_polys_xy, civil_city_stats = load_civil_airport_no_fly_zones(
            city_bbox,
            fwd,
            dataset_geojson_path=args.civil_airport_no_fly_geojson,
        )
        civil_airport_city_all_polys_xy, civil_city_match_stats = load_civil_airport_no_fly_zones_by_city(
            args.city,
            fwd,
            dataset_geojson_path=args.civil_airport_no_fly_geojson,
        )
        if civil_airport_city_all_polys_xy:
            # Standard behavior: always include city-level civil airport no-fly zones
            # even when the route/data bbox is a local corridor crop.
            nofly_hard_polys_xy.extend(civil_airport_city_all_polys_xy)
            civil_airport_display_polys_xy.extend(civil_airport_city_all_polys_xy)
        if not civil_airport_display_polys_xy:
            civil_airport_display_polys_xy = list(civil_airport_route_scope_polys_xy)
        nofly_counter["civil_dataset_city_scope"] = civil_city_stats
        nofly_counter["civil_dataset_city_match"] = civil_city_match_stats
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
    high_rise_points_xy: List[Any] = []
    high_building_polys_xy = [g.buffer(HIGH_BUILDING_AVOID_BUFFER_M) for g, h in zip(b_geoms, b_heights) if h >= HIGH_BUILDING_THRESHOLD_M]
    for g, h in zip(b_geoms, b_heights):
        if h < HIGH_BUILDING_THRESHOLD_M:
            continue
        try:
            high_rise_points_xy.append(g.representative_point())
        except Exception:
            continue
    high_building_union_xy = unary_union(high_building_polys_xy) if high_building_polys_xy else None
    if high_rise_points_xy:
        crowd_points_xy.extend(high_rise_points_xy)
        for pt in high_rise_points_xy:
            crowd_display_items.append(
                {
                    "point_xy": pt,
                    "name": "高层建筑",
                    "type": "高层建筑",
                    "source": "建筑物",
                }
            )
        poi_risk_counter["crowd"] = int(poi_risk_counter.get("crowd", 0) + len(high_rise_points_xy))
        poi_risk_counter["crowd_from_high_rise"] = int(len(high_rise_points_xy))

    crowd_points_xy = _dedupe_point_geometries(crowd_points_xy, snap_m=8.0)
    key_points_xy = _dedupe_point_geometries(key_points_xy, snap_m=8.0)
    school_point_tooltips_xy = _build_poi_tooltip_lookup(school_poi_items, snap_m=8.0)
    crowd_point_tooltips_xy = _build_poi_tooltip_lookup(crowd_display_items, snap_m=8.0)
    key_point_tooltips_xy = _build_poi_tooltip_lookup(key_poi_items, snap_m=8.0)
    crowd_tree = STRtree(crowd_points_xy) if crowd_points_xy else None
    crowd_id_map = _build_geom_id_map(crowd_points_xy)
    key_tree = STRtree(key_points_xy) if key_points_xy else None
    key_id_map = _build_geom_id_map(key_points_xy)

    legacy_infra_hard_union_xy = None
    if args.infra_hard_buffer_m > 0 and infra_geoms:
        hard_geoms = [
            g.buffer(float(args.infra_hard_buffer_m))
            for g, sev in zip(infra_geoms, infra_severities)
            if sev >= CRITICAL_INFRA_MIN_SEVERITY
        ]
        if hard_geoms:
            legacy_infra_hard_union_xy = unary_union(hard_geoms)
    safety_infra_hard_union_xy = None
    if args.safety_infra_hard_buffer_m > 0 and infra_geoms:
        hard_geoms = [
            g.buffer(float(args.safety_infra_hard_buffer_m))
            for g, sev in zip(infra_geoms, infra_severities)
            if sev >= CRITICAL_INFRA_MIN_SEVERITY
        ]
        if hard_geoms:
            safety_infra_hard_union_xy = unary_union(hard_geoms)
    sensitive_hard_union_xy = _buffer_union_from_points(key_points_xy, float(args.safety_sensitive_hard_buffer_m))

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
    edge_feature_cache: Dict[Tuple[Tuple[float, float], Tuple[float, float], str], Dict[str, float]] = {}

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
        enable_sensitive_hard_constraint: bool,
        enable_infra_hard_constraint: bool,
    ) -> Optional[Dict[str, Any]]:
        local_weights = WEIGHT_PROFILES[profile_key].copy()
        min_turn_angle_deg = max(60.0, min(179.0, float(args.min_turn_angle_deg)))
        max_turn_deflection_deg = max(1.0, 180.0 - min_turn_angle_deg)
        local_sensitive_hard_union_xy = sensitive_hard_union_xy if enable_sensitive_hard_constraint else None
        local_infra_hard_union_xy = legacy_infra_hard_union_xy
        if enable_infra_hard_constraint and safety_infra_hard_union_xy is not None:
            if local_infra_hard_union_xy is None:
                local_infra_hard_union_xy = safety_infra_hard_union_xy
            else:
                local_infra_hard_union_xy = unary_union([local_infra_hard_union_xy, safety_infra_hard_union_xy])
        local_crowd_hard_union_xy = school_hard_union_xy
        if local_sensitive_hard_union_xy is not None:
            if local_crowd_hard_union_xy is None:
                local_crowd_hard_union_xy = local_sensitive_hard_union_xy
            else:
                local_crowd_hard_union_xy = unary_union([local_crowd_hard_union_xy, local_sensitive_hard_union_xy])

        def _turn_angle_ok(poly: List[Tuple[float, float]]) -> bool:
            return polyline_min_interior_angle(poly) >= (min_turn_angle_deg - 1e-6)

        for wk, sv in (weight_scale or {}).items():
            if wk in local_weights:
                local_weights[wk] = max(0.01, float(local_weights[wk]) * float(sv))
        local_weights["soft_no_fly"] = max(0.0, local_weights["soft_no_fly"] * float(args.soft_no_fly_scale))
        local_weights["reference_deviation"] = 1.0 if reference_mode else 0.0
        graph_local, graph_stats_local = build_navigation_graph(
            networks_xy,
            start_xy,
            end_xy,
            weights=local_weights,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            no_fly_soft_union_xy=nofly_soft_union_xy,
            infra_hard_union_xy=local_infra_hard_union_xy,
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
            school_hard_union_xy=school_hard_union_xy,
            sensitive_hard_union_xy=local_sensitive_hard_union_xy,
            school_penalty_air=float(school_penalty_air),
            school_penalty_ground=float(school_penalty_ground),
            key_points_xy=key_points_xy,
            key_tree=key_tree,
            key_id_map=key_id_map,
            line_risk_union_xy=line_risk_union_xy,
            high_building_union_xy=high_building_union_xy,
            enable_water_endpoint_connectors=enable_water_connectors,
            edge_feature_cache=edge_feature_cache,
            reference_line_xy=reference_line_xy,
            reference_corridor_m=float(args.reference_corridor_m),
            reference_deviation_weight=float(args.reference_deviation_weight),
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
            min_per_m_hint=graph_stats_local.get("min_base_per_m", None),
        )
        water_nodes, water_cost = astar_with_turn_penalty(
            graph_local,
            start_node,
            end_node,
            turn_weight=local_weights["turn"],
            turn_radius_m=args.turn_radius_m,
            water_pref_factor=water_pref_factor,
            max_turn_deflection_deg=max_turn_deflection_deg,
            min_per_m_hint=graph_stats_local.get("min_base_per_m", None),
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
            infra_hard_union_xy=local_infra_hard_union_xy,
            crowd_hard_union_xy=local_crowd_hard_union_xy,
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
            infra_hard_union_xy=local_infra_hard_union_xy,
            crowd_hard_union_xy=local_crowd_hard_union_xy,
            passes=4,
        )
        if _turn_angle_ok(cand):
            nodes = cand
        cand = prune_low_value_turns(
            nodes,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=local_infra_hard_union_xy,
            crowd_hard_union_xy=local_crowd_hard_union_xy,
            min_turn_keep_deg=float(args.min_turn_keep_deg),
            passes=max(1, int(args.turn_prune_passes)),
        )
        cand = enforce_min_turn_angle(
            cand,
            min_turn_angle_deg=min_turn_angle_deg,
            pop_sampler=pop_sampler,
            inv=inv,
            no_fly_hard_union_xy=nofly_hard_union_xy,
            infra_hard_union_xy=local_infra_hard_union_xy,
            crowd_hard_union_xy=local_crowd_hard_union_xy,
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
                infra_hard_union_xy=local_infra_hard_union_xy,
                crowd_hard_union_xy=local_crowd_hard_union_xy,
            )
            cand = enforce_min_turn_angle(
                cand,
                min_turn_angle_deg=min_turn_angle_deg,
                pop_sampler=pop_sampler,
                inv=inv,
                no_fly_hard_union_xy=nofly_hard_union_xy,
                infra_hard_union_xy=local_infra_hard_union_xy,
                crowd_hard_union_xy=local_crowd_hard_union_xy,
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
            lon, lat = xy_to_wgs(inv, x, y)
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
                "sensitive_facility_points_in_buffer": int(key_in_buf_local),
                "key_facility_points_in_buffer": int(key_in_buf_local),
                "critical_infra_geoms_in_buffer": int(infra_in_buf_local),
                "line_risk_overlap_m": round(line_overlap_local, 2),
                "high_building_overlap_m": round(high_build_overlap_local, 2),
            },
            "graph_stats": graph_stats_local,
        }

    core_candidate_specs = [
        CandidateSpec(
            id="safety_default",
            label="安全优先",
            profile_key="safest",
            enable_water_connectors=False,
            water_pref_factor=0.72,
            allow_water_choice=True,
            water_detour_limit=1.18,
            min_water_share=0.08,
            weight_scale={
                "length": 0.92,
                "population": 1.26,
                "landuse": 1.22,
                "infrastructure": 1.24,
                "altitude": 1.12,
                "turn": 1.08,
                "crowd": 1.28,
                "key_facility": 1.2,
                "line_cross": 1.18,
            },
            school_penalty_air=11.0,
            school_penalty_ground=8.0,
            enable_sensitive_hard_constraint=True,
            enable_infra_hard_constraint=True,
        ),
        CandidateSpec(
            id="efficiency",
            label="效率优先",
            profile_key="fastest",
            enable_water_connectors=False,
            water_pref_factor=1.0,
            allow_water_choice=False,
            water_detour_limit=1.35,
            min_water_share=0.0,
            weight_scale={
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
            },
            school_penalty_air=5.0,
            school_penalty_ground=3.8,
        ),
    ]
    sweep_candidate_specs: List[CandidateSpec] = []
    if int(args.weight_sweep_levels) > 0:
        print("[WARN] weight sweep is disabled in 2-candidate mode; ignoring --weight-sweep-levels.", flush=True)
    candidate_specs = list(core_candidate_specs)

    candidate_results: List[Dict[str, Any]] = []
    candidate_failures: List[Dict[str, Any]] = []
    for spec in candidate_specs:
        failure_reason = ""
        try:
            solved = _solve_scheme(
                scheme_id=spec.id,
                label=spec.label,
                profile_key=spec.profile_key,
                enable_water_connectors=spec.enable_water_connectors,
                water_pref_factor=spec.water_pref_factor,
                allow_water_choice=spec.allow_water_choice,
                water_detour_limit=spec.water_detour_limit,
                min_water_share=spec.min_water_share,
                weight_scale=spec.weight_scale,
                school_penalty_air=spec.school_penalty_air,
                school_penalty_ground=spec.school_penalty_ground,
                enable_sensitive_hard_constraint=bool(spec.enable_sensitive_hard_constraint),
                enable_infra_hard_constraint=bool(spec.enable_infra_hard_constraint),
            )
        except Exception as exc:
            solved = None
            failure_reason = str(exc)
        if solved is None:
            candidate_failures.append({"id": spec.id, "error": failure_reason or "no feasible route"})
            continue
        route_len_m = float(solved["route_line_xy"].length)
        solved["distance_km"] = round(route_len_m / 1000.0, 3)
        solved["detour_ratio"] = round(route_len_m / max(1e-6, direct_dist_m), 3)
        solved["water_share"] = round(
            float(solved["route_cost"].get("water_distance_m", 0.0)) / max(1e-6, float(solved["route_cost"].get("distance_m", 0.0))),
            3,
        )
        if reference_line_xy is not None and reference_length_m > 0:
            off_mean, off_p90, off_max = _line_offset_stats(solved["route_line_xy"], reference_line_xy)
            solved["mean_offset_to_reference_m"] = round(float(off_mean), 2)
            solved["p90_offset_to_reference_m"] = round(float(off_p90), 2)
            solved["max_offset_to_reference_m"] = round(float(off_max), 2)
            solved["detour_ratio_vs_reference"] = round(route_len_m / max(1e-6, reference_length_m), 3)
        pop_avg, pop_p90, pop_max = _line_population_stats(solved["route_line_xy"], pop_sampler, inv)
        solved["path_population_avg"] = round(pop_avg, 2)
        solved["path_population_p90"] = round(pop_p90, 2)
        solved["path_population_max"] = round(pop_max, 2)
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
                clearance_m=float(effective_clearance_m),
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

    if reference_mode:
        max_ref_detour = max(1.0, float(args.reference_max_detour_ratio))
        max_ref_mean_offset = max(0.0, float(args.reference_max_mean_offset_m))
        filtered_ref: List[Dict[str, Any]] = []
        for cand in candidate_results:
            detour_ref = float(cand.get("detour_ratio_vs_reference", cand.get("detour_ratio", 999.0)))
            mean_offset = float(cand.get("mean_offset_to_reference_m", 1e9))
            if detour_ref > max_ref_detour + 1e-9:
                candidate_failures.append(
                    {
                        "id": cand.get("id", "unknown"),
                        "error": (
                            f"reference_detour_exceeded: detour_ratio_vs_reference={detour_ref:.3f} "
                            f"> limit={max_ref_detour:.3f}"
                        ),
                    }
                )
                continue
            if mean_offset > max_ref_mean_offset + 1e-9:
                candidate_failures.append(
                    {
                        "id": cand.get("id", "unknown"),
                        "error": (
                            f"reference_mean_offset_exceeded: mean_offset={mean_offset:.2f}m "
                            f"> limit={max_ref_mean_offset:.2f}m"
                        ),
                    }
                )
                continue
            filtered_ref.append(cand)
        candidate_results = filtered_ref
        if not candidate_results:
            raise RuntimeError(f"No feasible route after reference constraints: {candidate_failures}")

    pareto_metric_keys = ["distance_km", "path_population_p90", "vertical_energy_proxy_m"]
    pareto_front = pareto_front_candidates(candidate_results, metric_keys=pareto_metric_keys)
    pareto_policy, pareto_policy_trace = resolve_pareto_policy(
        policy_file=str(args.pareto_policy_file),
        policy_name=str(args.pareto_policy_name),
        city=str(args.city),
        profile=str(args.profile),
        business_line=str(args.pareto_business_line),
        cli_detour_limit_ratio=args.pareto_detour_limit_ratio,
        cli_distance_weight=args.pareto_distance_weight,
        cli_population_weight=args.pareto_pop_weight,
        cli_energy_weight=args.pareto_energy_weight,
        fallback_energy_weight=float(args.vertical_energy_weight),
    )
    pareto_detour_limit = max(1.0, _safe_float(pareto_policy.get("detour_limit_ratio", 1.25), 1.25))
    pareto_distance_weight = max(0.05, _safe_float(pareto_policy.get("distance_weight", 1.0), 1.0))
    pareto_population_weight = max(0.05, _safe_float(pareto_policy.get("population_weight", 1.0), 1.0))
    pareto_energy_weight = max(0.05, _safe_float(pareto_policy.get("energy_weight", args.vertical_energy_weight), 1.25))

    selected_candidate = None
    if reference_mode:
        candidate_results = sorted(
            candidate_results,
            key=lambda c: (
                float(c.get("mean_offset_to_reference_m", 1e9)),
                float(c.get("detour_ratio_vs_reference", 1e9)),
                float(c.get("path_population_p90", 1e9)),
                float(c.get("vertical_energy_proxy_m", 1e9)),
            ),
        )
        selected_candidate = candidate_results[0]
        selected_candidate["strategy"] = f"{selected_candidate.get('strategy', 'base')}+reference_min_change"
    else:
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

    if (not reference_mode) and args.pareto_select and pareto_front:
        base = selected_candidate
        base_dist = max(1e-6, _safe_float(base.get("distance_km", 0.0), 0.0))
        base_pop = max(1e-6, _safe_float(base.get("path_population_p90", 0.0), 0.0))
        base_energy = max(1e-6, _safe_float(base.get("vertical_energy_proxy_m", 0.0), 0.0))
        best = base
        best_score = float("inf")
        for cand in pareto_front:
            dist_ratio = max(1e-6, _safe_float(cand.get("distance_km", 0.0), 0.0)) / base_dist
            if dist_ratio > pareto_detour_limit:
                continue
            pop_ratio = max(1e-6, _safe_float(cand.get("path_population_p90", 0.0), 0.0)) / base_pop
            energy_ratio = max(1e-6, _safe_float(cand.get("vertical_energy_proxy_m", 0.0), 0.0)) / base_energy
            score = (
                pareto_distance_weight * dist_ratio
                + pareto_population_weight * pop_ratio
                + pareto_energy_weight * energy_ratio
            )
            if score < best_score:
                best = cand
                best_score = score
        if best is not base:
            best["strategy"] = f"{best.get('strategy', 'base')}+pareto_select"
            selected_candidate = best

    if (not reference_mode) and args.vertical_tradeoff and len(candidate_results) > 1:
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
    sensitive_in_buffer = int(
        selected_candidate["buffer_metrics"].get(
            "sensitive_facility_points_in_buffer",
            selected_candidate["buffer_metrics"].get("key_facility_points_in_buffer", 0),
        )
    )
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
    pareto_out = out_dir / f"{base_name}_pareto.json"
    evidence_out = out_dir / f"{base_name}_evidence.json"
    snapshot_root = Path(args.snapshot_dir).resolve() if args.snapshot_dir else (out_dir / "snapshots").resolve()
    snapshot_out = snapshot_root / f"{base_name}_snapshot.json"
    landuse_display_counter = {
        "total": int(len(landuse_geoms)),
        "low_cost": int(sum(1 for c in landuse_costs if c <= 0.6)),
        "medium_cost": int(sum(1 for c in landuse_costs if 0.6 < c <= 1.2)),
        "high_cost": int(sum(1 for c in landuse_costs if c > 1.2)),
    }
    population_points_wgs = build_population_density_samples(pop_sampler, route_bbox, max_points=3600)
    population_tiles_template = ""
    population_tiles_min_zoom = 0
    population_tiles_max_native_zoom = 18
    population_tiles_dir = Path(summary.get("outputs", {}).get("population", {}).get("tiles_dir", ""))
    if population_tiles_dir.exists():
        encoded_tiles_dir = parse.quote(population_tiles_dir.resolve().as_posix(), safe="/:")
        population_tiles_template = f"file://{encoded_tiles_dir}" + "/{z}/{x}/{y}.png"
        zoom_levels: List[int] = []
        for sub in population_tiles_dir.iterdir():
            if sub.is_dir() and str(sub.name).isdigit():
                zoom_levels.append(int(str(sub.name)))
        if zoom_levels:
            population_tiles_min_zoom = min(zoom_levels)
            population_tiles_max_native_zoom = max(zoom_levels)
    dynamic_grb_xy = build_dynamic_grb_geometry(route_line_xy, profile_samples)
    dynamic_grb_area_m2 = 0.0
    if dynamic_grb_xy is not None and (not dynamic_grb_xy.is_empty):
        try:
            dynamic_grb_area_m2 = float(dynamic_grb_xy.area)
        except Exception:
            dynamic_grb_area_m2 = 0.0
    candidate_summary = []
    candidate_routes_wgs_for_html: List[Dict[str, Any]] = []
    result_by_id = {str(c["id"]): c for c in candidate_results}
    candidate_kml_outputs: Dict[str, str] = {}
    for cid, cand in result_by_id.items():
        alt_points = cand.get("altitude_points_wgs_alt") or []
        if not alt_points:
            continue
        cand_kml_out = out_dir / f"{base_name}_{cid}.kml"
        write_kml_absolute(cand_kml_out, alt_points, f"{base_name}_{cid}")
        candidate_kml_outputs[cid] = str(cand_kml_out)
    for spec in core_candidate_specs:
        cid = spec.id
        if cid not in result_by_id:
            continue
        cand = result_by_id[cid]
        altitude_points = cand.get("altitude_points_wgs_alt") or []
        coords = [(lat, lon) for lon, lat, _ in altitude_points] if altitude_points else [(lat, lon) for lon, lat in cand["route_nodes_wgs"]]
        show_flag = (cand["id"] == selected_candidate["id"])
        candidate_routes_wgs_for_html.append(
            {
                "id": cand["id"],
                "label": spec.label,
                "coords": coords,
                "alt_points": [
                    (float(lon), float(lat), float(alt))
                    for lon, lat, alt in altitude_points
                ],
                "profile_samples": cand.get("altitude_profile_samples") or [],
                "distance_km": cand["distance_km"],
                "route_buffer_xy": cand["route_buffer_xy"],
                "show": show_flag,
            }
        )
        candidate_summary.append(
            {
                "id": cand["id"],
                "label": spec.label,
                "selected": bool(show_flag),
                "profile_key": cand["profile_key"],
                "distance_km": cand["distance_km"],
                "detour_ratio_vs_direct": cand["detour_ratio"],
                "detour_ratio_vs_reference": cand.get("detour_ratio_vs_reference", None),
                "mean_offset_to_reference_m": cand.get("mean_offset_to_reference_m", None),
                "p90_offset_to_reference_m": cand.get("p90_offset_to_reference_m", None),
                "max_offset_to_reference_m": cand.get("max_offset_to_reference_m", None),
                "water_share": cand["water_share"],
                "turns_after_post_smooth": int(cand["turns_after"]),
                "min_turn_angle_deg": float(cand.get("min_turn_angle_deg", 180.0)),
                "vertical_energy_proxy_m": round(float(cand.get("vertical_energy_proxy_m", 0.0)), 2),
                "total_climb_m": round(float(cand.get("total_climb_m", 0.0)), 2),
                "total_descent_m": round(float(cand.get("total_descent_m", 0.0)), 2),
                "max_true_height_m": round(float(cand.get("max_true_height_m", 0.0)), 2),
                "kml": candidate_kml_outputs.get(cid, ""),
                "buffer_metrics": cand["buffer_metrics"],
                "route_selection_strategy": cand["strategy"],
            }
        )
    if candidate_failures:
        candidate_summary.append({"failures": candidate_failures})

    pareto_sorted = sorted(
        pareto_front,
        key=lambda c: (
            _safe_float(c.get("distance_km", 0.0)),
            _safe_float(c.get("path_population_p90", 0.0)),
            _safe_float(c.get("vertical_energy_proxy_m", 0.0)),
        ),
    )
    pareto_front_summary: List[Dict[str, Any]] = []
    for cand in pareto_sorted[: max(1, int(args.pareto_max_front))]:
        pareto_front_summary.append(
            {
                "id": str(cand.get("id", "")),
                "label": str(cand.get("label", "")),
                "distance_km": round(_safe_float(cand.get("distance_km", 0.0)), 3),
                "path_population_p90": round(_safe_float(cand.get("path_population_p90", 0.0)), 2),
                "vertical_energy_proxy_m": round(_safe_float(cand.get("vertical_energy_proxy_m", 0.0)), 2),
                "water_share": round(_safe_float(cand.get("water_share", 0.0)), 3),
                "selected": bool(str(cand.get("id", "")) == str(selected_candidate.get("id", ""))),
            }
        )
    pareto_payload = {
        "metrics": pareto_metric_keys,
        "sweep_levels": int(len(sweep_candidate_specs)),
        "evaluated_candidates": len(candidate_results),
        "pareto_front_size": len(pareto_front),
        "pareto_front": pareto_front_summary,
        "selected_candidate_id": str(selected_candidate.get("id", "")),
        "pareto_select_enabled": bool(args.pareto_select),
        "policy": {
            "detour_limit_ratio": round(pareto_detour_limit, 4),
            "distance_weight": round(pareto_distance_weight, 4),
            "population_weight": round(pareto_population_weight, 4),
            "energy_weight": round(pareto_energy_weight, 4),
            "trace": pareto_policy_trace,
        },
    }
    pareto_out.write_text(json.dumps(pareto_payload, ensure_ascii=False, indent=2), encoding="utf-8")

    write_kml_absolute(kml_out, points_wgs_alt, base_name)
    write_preview_html(
        html_out,
        points_wgs_alt,
        candidate_routes_wgs=candidate_routes_wgs_for_html,
        start_wgs=start_wgs,
        end_wgs=end_wgs,
        civil_airport_polys_xy=civil_airport_display_polys_xy,
        military_hard_nofly_polys_xy=nofly_military_hard_polys_xy,
        heli_soft_nofly_polys_xy=nofly_heli_soft_polys_xy,
        route_buffer_xy=route_buffer_xy,
        dynamic_grb_xy=dynamic_grb_xy,
        population_points_wgs=population_points_wgs,
        population_tiles_template=population_tiles_template,
        population_tiles_min_zoom=population_tiles_min_zoom,
        population_tiles_max_native_zoom=population_tiles_max_native_zoom,
        landuse_geoms_xy=landuse_geoms,
        landuse_costs=landuse_costs,
        all_building_polys_xy=b_geoms,
        high_building_polys_xy=high_building_polys_xy,
        crowd_points_xy=crowd_points_xy,
        key_points_xy=key_points_xy,
        infra_geoms_xy=infra_geoms,
        high_road_lines_xy=high_road_lines_xy,
        hsr_lines_xy=hsr_lines_xy,
        hv_power_lines_xy=hv_power_lines_xy,
        line_risk_union_xy=line_risk_union_xy,
        school_hard_zones_xy=school_hard_zones_xy,
        school_points_xy=school_points_xy,
        school_point_tooltips_xy=school_point_tooltips_xy,
        crowd_point_tooltips_xy=crowd_point_tooltips_xy,
        key_point_tooltips_xy=key_point_tooltips_xy,
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
        "reference_mode_enabled": bool(reference_mode),
        "reference_route": {
            "kml": str(Path(args.reference_kml).resolve()) if args.reference_kml else "",
            "length_km": round(reference_length_m / 1000.0, 3) if reference_mode else None,
            "detour_ratio_vs_reference": round(route_line_xy.length / max(1e-6, reference_length_m), 3)
            if reference_mode
            else None,
            "mean_offset_to_reference_m": selected_candidate.get("mean_offset_to_reference_m", None),
            "p90_offset_to_reference_m": selected_candidate.get("p90_offset_to_reference_m", None),
            "max_offset_to_reference_m": selected_candidate.get("max_offset_to_reference_m", None),
            "corridor_m": float(args.reference_corridor_m),
            "deviation_weight": float(args.reference_deviation_weight),
            "max_detour_ratio": float(args.reference_max_detour_ratio),
            "max_mean_offset_m": float(args.reference_max_mean_offset_m),
        },
        "waypoints_xy": len(route_nodes),
        "open_data_no_fly_enabled": bool(args.open_data_no_fly),
        "no_fly_sources": nofly_counter,
        "network_stats": graph_stats,
        "poi_risk_sources": poi_risk_counter,
        "school_kindergarten_sources": school_counter,
        "line_risk_sources": line_risk_counter,
        "landuse_sources": landuse_display_counter,
        "low_risk_landuse_sources": landuse_display_counter,
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
            "sensitive_facility_points_in_buffer": int(sensitive_in_buffer),
            "key_facility_points_in_buffer": int(sensitive_in_buffer),
            "critical_infra_geoms_in_buffer": int(infra_in_buffer),
            "line_risk_overlap_m": round(line_overlap_m, 2),
            "high_building_overlap_m": round(high_build_overlap_m, 2),
            "dynamic_grb_area_m2": round(dynamic_grb_area_m2, 2),
        },
        "route_cost_base": route_cost_base,
        "route_cost_water_priority": route_cost_water,
        "candidate_options": candidate_summary,
        "pareto": pareto_payload,
        "airframe": {
            "speed_ms": args.speed_ms,
            "climb_ms": args.climb_ms,
            "descend_ms": args.descend_ms,
            "turn_radius_m": args.turn_radius_m,
            "clearance_m": float(effective_clearance_m),
            "requested_clearance_m": float(args.clearance_m),
            "endpoint_true_height_m": args.endpoint_true_height_m,
            "min_true_height_m": args.min_true_height_m,
            "max_true_height_m": args.max_true_height_m,
            "preferred_cruise_max_m": args.preferred_cruise_max_m,
            "hard_ceiling_m": args.hard_ceiling_m,
        },
        "safety_hard_constraints": {
            "sensitive_facility_buffer_m": float(args.safety_sensitive_hard_buffer_m),
            "critical_infrastructure_buffer_m": float(args.safety_infra_hard_buffer_m),
            "legacy_global_infra_buffer_m": float(args.infra_hard_buffer_m),
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
            "candidate_kmls": candidate_kml_outputs,
            "preview_html": str(html_out),
            "candidates_json": str(cand_out),
            "pareto_json": str(pareto_out),
            "evidence_json": str(evidence_out) if args.write_evidence else "",
            "snapshot_json": str(snapshot_out) if args.write_snapshot else "",
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

    if args.write_evidence:
        if build_evidence_pack is None or write_evidence_pack is None:
            print("[WARN] evidence helpers unavailable; skip evidence output.", flush=True)
        else:
            try:
                evidence = build_evidence_pack(
                    meta=meta,
                    candidate_summary=candidate_summary,
                    candidate_failures=candidate_failures,
                )
                write_evidence_pack(evidence_out, evidence)
            except Exception as exc:
                print(f"[WARN] failed to write evidence: {exc}", flush=True)

    if args.write_snapshot:
        if write_run_snapshot is None:
            print("[WARN] snapshot helper unavailable; skip snapshot output.", flush=True)
        else:
            try:
                dem_tif = Path(args.dem_tif).resolve() if args.dem_tif else None
                outputs_snapshot = {
                    "kml": str(kml_out),
                    "candidate_kmls": candidate_kml_outputs,
                    "preview_html": str(html_out),
                    "meta_json": str(meta_out),
                    "candidates_json": str(cand_out),
                    "pareto_json": str(pareto_out),
                    "evidence_json": str(evidence_out) if args.write_evidence else "",
                    "layered_html": review_html,
                }
                write_run_snapshot(
                    snapshot_out,
                    algorithm_version=ALGORITHM_VERSION,
                    script_path=Path(__file__).resolve(),
                    args=dict(vars(args)),
                    city=args.city,
                    start_wgs=start_wgs,
                    end_wgs=end_wgs,
                    summary_path=summary_path,
                    pop_tif=pop_tif if pop_tif.exists() else None,
                    dem_tif=dem_tif if dem_tif and dem_tif.exists() else None,
                    candidate_ids=[spec.id for spec in candidate_specs],
                    candidate_failures=candidate_failures,
                    outputs=outputs_snapshot,
                )
            except Exception as exc:
                print(f"[WARN] failed to write snapshot: {exc}", flush=True)

    print("DONE")
    print(f"KML: {kml_out}")
    for cid in sorted(candidate_kml_outputs.keys()):
        print(f"Candidate KML ({cid}): {candidate_kml_outputs[cid]}")
    print(f"Preview HTML: {html_out}")
    print(f"Meta: {meta_out}")
    print(f"Candidates: {cand_out}")
    print(f"Pareto: {pareto_out}")
    if args.write_evidence:
        print(f"Evidence: {evidence_out}")
    if args.write_snapshot:
        print(f"Snapshot: {snapshot_out}")
    if review_html:
        print(f"Layered HTML: {review_html}")


if __name__ == "__main__":
    main()
