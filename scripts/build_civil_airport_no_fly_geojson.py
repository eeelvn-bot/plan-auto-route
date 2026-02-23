#!/usr/bin/env python3
from __future__ import annotations

import argparse
import io
import json
import re
from pathlib import Path
from typing import Any, Dict, List, Optional


COORD_COLUMNS = ["A1", "A2", "C2", "B2", "B3", "C3", "A3", "A4", "C4", "B4", "B1", "C1"]


def _load_sheet(path: Path, password: str):
    try:
        import xlrd
    except Exception as exc:
        raise RuntimeError("Missing dependency: xlrd") from exc
    try:
        book = xlrd.open_workbook(str(path))
        return book.sheet_by_index(0)
    except Exception as exc:
        message = str(exc).lower()
        if "encrypted" not in message:
            raise
    try:
        import msoffcrypto
    except Exception as exc:
        raise RuntimeError("Encrypted workbook requires msoffcrypto-tool") from exc
    with path.open("rb") as f:
        office = msoffcrypto.OfficeFile(f)
        office.load_key(password=password)
        bio = io.BytesIO()
        office.decrypt(bio)
    book = xlrd.open_workbook(file_contents=bio.getvalue())
    return book.sheet_by_index(0)


def _parse_coord(raw: Any) -> Optional[List[float]]:
    text = str(raw).strip()
    if not text:
        return None
    text = text.replace("”", "").replace("″", "").replace('"', "").replace(" ", "")
    pattern = re.compile(r"([NS])(\d+)[°º](\d+(?:\.\d+)?)[′']([EW])(\d+)[°º](\d+(?:\.\d+)?)")
    match = pattern.search(text)
    if not match:
        return None
    ns, lat_d, lat_m, ew, lon_d, lon_m = match.groups()
    lat = float(lat_d) + float(lat_m) / 60.0
    lon = float(lon_d) + float(lon_m) / 60.0
    if ns == "S":
        lat = -lat
    if ew == "W":
        lon = -lon
    return [round(lon, 8), round(lat, 8)]


def build_geojson(xls_path: Path, password: str) -> Dict[str, Any]:
    sheet = _load_sheet(xls_path, password=password)
    headers = [str(sheet.cell_value(0, c)).strip() for c in range(sheet.ncols)]
    header_idx = {h: i for i, h in enumerate(headers)}
    features: List[Dict[str, Any]] = []
    skipped_rows = 0
    for row in range(1, sheet.nrows):
        airport_name = str(sheet.cell_value(row, header_idx.get("机场名称", 0))).strip()
        icao = str(sheet.cell_value(row, header_idx.get("四字地名代码", 1))).strip()
        runway = str(sheet.cell_value(row, header_idx.get("跑道号码", 3))).strip()
        if not icao:
            continue
        ring: List[List[float]] = []
        for col in COORD_COLUMNS:
            col_idx = header_idx.get(col)
            if col_idx is None:
                continue
            pt = _parse_coord(sheet.cell_value(row, col_idx))
            if pt is not None and (not ring or ring[-1] != pt):
                ring.append(pt)
        if len(ring) < 3:
            skipped_rows += 1
            continue
        if ring[0] != ring[-1]:
            ring.append(ring[0])
        elev = sheet.cell_value(row, header_idx.get("机场标高(m)", 2))
        try:
            elev_val: Optional[float] = float(elev)
        except Exception:
            elev_val = None
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                "properties": {
                    "source": "CAAC XLS",
                    "airport_name": airport_name,
                    "icao": icao,
                    "runway": runway,
                    "airport_elevation_m": elev_val,
                },
            }
        )
    return {
        "type": "FeatureCollection",
        "name": "civil_airport_no_fly_from_xls",
        "metadata": {
            "source_file": str(xls_path),
            "source_sheet": sheet.name,
            "total_rows": sheet.nrows - 1,
            "features": len(features),
            "skipped_rows_without_geometry": skipped_rows,
            "coord_columns_order": COORD_COLUMNS,
            "encrypted_password_used": password,
        },
        "features": features,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Build civil-airport no-fly GeoJSON from CAAC XLS.")
    parser.add_argument("--xls", required=True, help="Input CAAC civil-airport XLS file.")
    parser.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parents[1] / "config" / "civil_airport_no_fly.geojson"),
        help="Output GeoJSON path.",
    )
    parser.add_argument(
        "--password",
        default="VelvetSweatshop",
        help="Workbook password used when XLS is encrypted.",
    )
    args = parser.parse_args()
    xls_path = Path(args.xls).resolve()
    if not xls_path.exists():
        raise FileNotFoundError(f"XLS not found: {xls_path}")
    out_path = Path(args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    geojson = build_geojson(xls_path, password=args.password)
    out_path.write_text(json.dumps(geojson, ensure_ascii=False), encoding="utf-8")
    print(f"Wrote {out_path} | features={len(geojson.get('features', []))}")


if __name__ == "__main__":
    main()
