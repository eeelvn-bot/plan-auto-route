#!/usr/bin/env python3
"""Evidence pack helpers for plan_auto_route."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def _pick_selected_candidate(candidates: Iterable[Dict[str, Any]], selected_id: str) -> Optional[Dict[str, Any]]:
    for item in candidates:
        if str(item.get("id", "")) == str(selected_id):
            return item
    for item in candidates:
        if bool(item.get("selected", False)):
            return item
    return None


def build_evidence_pack(
    *,
    meta: Dict[str, Any],
    candidate_summary: List[Dict[str, Any]],
    candidate_failures: List[Dict[str, Any]],
) -> Dict[str, Any]:
    selected_id = str(meta.get("selected_candidate_id", ""))
    selected = _pick_selected_candidate(candidate_summary, selected_id)
    route_post = meta.get("route_postprocess", {})
    altitude_profile = meta.get("altitude_profile", {})
    path_pop = meta.get("path_population_stats", {})
    route_cost = meta.get("route_cost_breakdown", {})
    network_stats = meta.get("network_stats", {})
    buffer_metrics = meta.get("buffer_metrics", {})
    no_fly_sources = meta.get("no_fly_sources", {})
    no_fly_enabled = bool(meta.get("open_data_no_fly_enabled", False))
    no_fly_detected = int(no_fly_sources.get("hard", 0)) + int(no_fly_sources.get("soft", 0))

    min_turn_threshold = float(route_post.get("min_turn_angle_deg", 0.0))
    selected_min_turn = float((selected or {}).get("min_turn_angle_deg", 180.0))
    constraints = [
        {
            "id": "hard_no_fly_filter",
            "ok": (not no_fly_enabled) or (no_fly_detected > 0),
            "detail": {
                "open_data_no_fly_enabled": no_fly_enabled,
                "source_counts": no_fly_sources,
                "edges_filtered_by_no_fly": int(network_stats.get("skipped_nofly", 0)),
            },
        },
        {
            "id": "min_turn_angle",
            "ok": selected_min_turn + 1e-6 >= min_turn_threshold,
            "detail": {
                "threshold_deg": min_turn_threshold,
                "selected_min_turn_deg": round(selected_min_turn, 2),
            },
        },
        {
            "id": "altitude_profile_feasible",
            "ok": bool(altitude_profile),
            "detail": {
                "profile_points": int(altitude_profile.get("profile_sample_count", 0)),
                "min_surface_clearance_m": float(altitude_profile.get("min_surface_clearance_m", 0.0)),
                "max_climb_rate_ms": float(altitude_profile.get("max_climb_rate_ms", 0.0)),
                "max_descend_rate_ms": float(altitude_profile.get("max_descend_rate_ms", 0.0)),
            },
        },
    ]

    objectives = {
        "distance_km": float(meta.get("planned_km", 0.0)),
        "detour_ratio_vs_direct": float(meta.get("detour_ratio_vs_direct", 0.0)),
        "population_p90": float(path_pop.get("p90", 0.0)),
        "turns_after_post_smooth": int(meta.get("turns_after_post_smooth", 0)),
        "vertical_energy_proxy_m": float(meta.get("vertical_tradeoff", {}).get("selected_vertical_energy_proxy_m", 0.0)),
        "water_share": float((selected or {}).get("water_share", 0.0)),
    }

    coverage = {
        "infrastructure_features": int(meta.get("infrastructure_features", 0)),
        "building_features": int(meta.get("building_features", 0)),
        "obstacle_features": int(meta.get("obstacle_features", 0)),
        "poi_sources": meta.get("poi_risk_sources", {}),
        "school_sources": meta.get("school_kindergarten_sources", {}),
        "line_risk_sources": meta.get("line_risk_sources", {}),
        "low_risk_landuse_sources": meta.get("low_risk_landuse_sources", {}),
    }

    selection_trace = {
        "selected_candidate_id": selected_id,
        "selected_candidate_label": str(meta.get("selected_candidate_label", "")),
        "strategy": str(meta.get("route_selection_strategy", "")),
        "vertical_tradeoff": meta.get("vertical_tradeoff", {}),
        "pareto": meta.get("pareto", {}),
        "candidate_failures": candidate_failures,
    }

    contribution = {
        "route_cost_breakdown": route_cost,
        "buffer_metrics": buffer_metrics,
    }

    return {
        "algorithm_version": meta.get("algorithm_version", ""),
        "city": meta.get("city", ""),
        "name": meta.get("name", ""),
        "constraints": constraints,
        "objectives": objectives,
        "coverage": coverage,
        "contribution": contribution,
        "selection_trace": selection_trace,
    }


def write_evidence_pack(path: Path, evidence: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
