#!/usr/bin/env python3
"""Run snapshot writer for plan_auto_route."""

from __future__ import annotations

import hashlib
import json
import platform
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple


def _normalize_jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [_normalize_jsonable(v) for v in value]
    if isinstance(value, list):
        return [_normalize_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _normalize_jsonable(v) for k, v in value.items()}
    return value


def write_run_snapshot(
    snapshot_path: Path,
    *,
    algorithm_version: str,
    script_path: Path,
    args: Dict[str, Any],
    city: str,
    start_wgs: Tuple[float, float],
    end_wgs: Tuple[float, float],
    summary_path: Path,
    pop_tif: Optional[Path],
    dem_tif: Optional[Path],
    candidate_ids: Iterable[str],
    candidate_failures: Iterable[Dict[str, Any]],
    outputs: Dict[str, str],
) -> None:
    snapshot_path.parent.mkdir(parents=True, exist_ok=True)
    now_utc = datetime.now(timezone.utc)
    run_seed = {
        "algorithm_version": algorithm_version,
        "city": city,
        "start_wgs": [float(start_wgs[0]), float(start_wgs[1])],
        "end_wgs": [float(end_wgs[0]), float(end_wgs[1])],
        "timestamp_utc": now_utc.isoformat(),
    }
    run_id = hashlib.sha1(json.dumps(run_seed, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()[:16]

    snapshot = {
        "run_id": run_id,
        "timestamp_utc": now_utc.isoformat(),
        "algorithm_version": algorithm_version,
        "script_path": str(script_path),
        "platform": {
            "python": platform.python_version(),
            "system": platform.system(),
            "release": platform.release(),
        },
        "od": {
            "city": city,
            "start_wgs84": {"lon": float(start_wgs[0]), "lat": float(start_wgs[1])},
            "end_wgs84": {"lon": float(end_wgs[0]), "lat": float(end_wgs[1])},
        },
        "inputs": {
            "city_summary_json": str(summary_path),
            "city_summary_exists": bool(summary_path.exists()),
            "population_tif": str(pop_tif) if pop_tif else "",
            "population_tif_exists": bool(pop_tif and pop_tif.exists()),
            "dem_tif": str(dem_tif) if dem_tif else "",
            "dem_tif_exists": bool(dem_tif and dem_tif.exists()),
        },
        "candidates": {
            "requested_ids": list(candidate_ids),
            "failures": list(candidate_failures),
        },
        "outputs": outputs,
        "args": _normalize_jsonable(args),
    }
    snapshot_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2), encoding="utf-8")

