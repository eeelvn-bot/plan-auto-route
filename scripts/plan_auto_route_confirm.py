#!/usr/bin/env python3
"""Interactive confirmation gateway for plan_auto_route.py."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from urllib import parse, request

USER_AGENT = "plan-auto-route-confirm/1.0"
NOMINATIM_SEARCH_URL = "https://nominatim.openstreetmap.org/search"
NOMINATIM_REVERSE_URL = "https://nominatim.openstreetmap.org/reverse"


@dataclass
class GeocodeCandidate:
    display_name: str
    lat: float
    lon: float
    importance: float
    source_raw: Dict[str, object]


def _http_get_json(url: str, params: Dict[str, object], timeout_sec: int = 12) -> object:
    query = parse.urlencode(params)
    req = request.Request(f"{url}?{query}", headers={"User-Agent": USER_AGENT})
    with request.urlopen(req, timeout=timeout_sec) as resp:
        return json.loads(resp.read().decode("utf-8"))


def forward_geocode(address: str, limit: int, countrycodes: str) -> List[GeocodeCandidate]:
    params: Dict[str, object] = {
        "q": address,
        "format": "jsonv2",
        "addressdetails": 1,
        "limit": max(1, int(limit)),
    }
    if countrycodes.strip():
        params["countrycodes"] = countrycodes.strip()
    rows = _http_get_json(NOMINATIM_SEARCH_URL, params)
    out: List[GeocodeCandidate] = []
    if isinstance(rows, list):
        for item in rows:
            if not isinstance(item, dict):
                continue
            try:
                lat = float(item.get("lat"))  # type: ignore[arg-type]
                lon = float(item.get("lon"))  # type: ignore[arg-type]
            except Exception:
                continue
            out.append(
                GeocodeCandidate(
                    display_name=str(item.get("display_name", address)).strip() or address,
                    lat=lat,
                    lon=lon,
                    importance=float(item.get("importance", 0.0) or 0.0),
                    source_raw=item,
                )
            )
    out.sort(key=lambda x: x.importance, reverse=True)
    return out


def reverse_geocode_city(lat: float, lon: float) -> Optional[str]:
    params = {
        "format": "jsonv2",
        "lat": lat,
        "lon": lon,
        "zoom": 10,
        "addressdetails": 1,
    }
    try:
        data = _http_get_json(NOMINATIM_REVERSE_URL, params)
    except Exception:
        return None
    if not isinstance(data, dict):
        return None
    addr = data.get("address")
    if not isinstance(addr, dict):
        return None
    for key in ["city", "municipality", "county", "state_district", "town", "district"]:
        val = str(addr.get(key, "")).strip()
        if val:
            return val
    return None


def prompt_yes_no(question: str, default: bool) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    while True:
        val = input(f"{question} {suffix} ").strip().lower()
        if not val:
            return default
        if val in {"y", "yes", "1", "true"}:
            return True
        if val in {"n", "no", "0", "false"}:
            return False
        print("请输入 y 或 n。", flush=True)


def select_candidate_interactive(label: str, address: str, candidates: Sequence[GeocodeCandidate]) -> GeocodeCandidate:
    print(f"\n{label}地址: {address}", flush=True)
    for idx, cand in enumerate(candidates, start=1):
        print(
            f"  {idx}. {cand.display_name} | lon={cand.lon:.7f}, lat={cand.lat:.7f}, importance={cand.importance:.3f}",
            flush=True,
        )
    default_idx = 1
    while True:
        val = input(f"请选择{label}候选编号 [默认{default_idx}]: ").strip()
        if not val:
            return candidates[default_idx - 1]
        if val.isdigit():
            n = int(val)
            if 1 <= n <= len(candidates):
                return candidates[n - 1]
        print("输入无效，请输入候选编号。", flush=True)


def _validate_kml_path(path_text: str) -> Path:
    p = Path(path_text).expanduser().resolve()
    if not p.exists():
        raise FileNotFoundError(f"未找到禁飞区文件: {p}")
    if p.suffix.lower() != ".kml":
        raise ValueError(f"仅支持 KML 文件: {p}")
    return p


def _validate_existing_routes_dir(path_text: str) -> Path:
    p = Path(path_text).expanduser().resolve()
    if not p.exists() or not p.is_dir():
        raise FileNotFoundError(f"已有航线库目录不存在: {p}")
    kmls = sorted(p.glob("*.kml"))
    if not kmls:
        raise ValueError(f"已有航线库目录未发现 KML: {p}")
    return p


def build_command(
    planner_script: Path,
    city: str,
    start_lon: float,
    start_lat: float,
    end_lon: float,
    end_lat: float,
    profile: str,
    top_k: int,
    name: str,
    emergency_enable: bool,
    custom_no_fly_kmls: Sequence[Path],
    existing_routes_dir: Optional[Path],
    passthrough: Sequence[str],
) -> List[str]:
    cmd = [
        "python3",
        str(planner_script),
        "--city",
        city,
        "--start-lon",
        f"{start_lon:.7f}",
        "--start-lat",
        f"{start_lat:.7f}",
        "--end-lon",
        f"{end_lon:.7f}",
        "--end-lat",
        f"{end_lat:.7f}",
        "--profile",
        profile,
        "--top-k",
        str(int(top_k)),
    ]
    if name.strip():
        cmd.extend(["--name", name.strip()])
    cmd.append("--emergency-enable" if emergency_enable else "--no-emergency-enable")
    for p in custom_no_fly_kmls:
        cmd.extend(["--custom-no-fly-kml", str(p)])
    if existing_routes_dir is not None:
        cmd.extend(["--existing-routes-dir", str(existing_routes_dir)])
    if passthrough:
        cmd.extend(list(passthrough))
    return cmd


def parse_args() -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description="plan-auto-route 确认入口：先地址解析与用户确认，再调用 plan_auto_route.py。",
    )
    parser.add_argument("--city", default="", help="规划城市。为空时尝试由起点坐标反查。")
    parser.add_argument("--start-address", default="", help="起点地址（未提供坐标时必填）。")
    parser.add_argument("--end-address", default="", help="终点地址（未提供坐标时必填）。")
    parser.add_argument("--start-lon", type=float, default=None)
    parser.add_argument("--start-lat", type=float, default=None)
    parser.add_argument("--end-lon", type=float, default=None)
    parser.add_argument("--end-lat", type=float, default=None)
    parser.add_argument("--profile", choices=["fastest", "balanced", "safest"], default="balanced")
    parser.add_argument("--top-k", type=int, default=1)
    parser.add_argument("--name", default="")
    parser.add_argument("--custom-no-fly-kml", action="append", default=[])
    parser.add_argument("--existing-routes-dir", default="")
    parser.add_argument("--emergency-enable", dest="emergency_enable", action="store_true", default=None)
    parser.add_argument("--no-emergency-enable", dest="emergency_enable", action="store_false")
    parser.add_argument("--geocode-limit", type=int, default=5)
    parser.add_argument("--geocode-countrycodes", default="cn", help="Nominatim countrycodes，默认 cn。")
    parser.add_argument("--planner-script", default=str(Path(__file__).resolve().parent / "plan_auto_route.py"))
    parser.add_argument("--non-interactive", action="store_true", help="非交互模式：采用默认值，不询问确认。")
    parser.add_argument("--yes", action="store_true", help="等价于 non-interactive，直接确认默认参数并继续。")
    parser.add_argument("--dry-run", action="store_true", help="仅打印最终命令，不执行规划。")
    args, passthrough = parser.parse_known_args()
    return args, passthrough


def main() -> None:
    args, passthrough = parse_args()
    non_interactive = bool(args.non_interactive or args.yes)

    planner_script = Path(str(args.planner_script)).expanduser().resolve()
    if not planner_script.exists():
        raise FileNotFoundError(f"planner script not found: {planner_script}")

    start_lon = args.start_lon
    start_lat = args.start_lat
    end_lon = args.end_lon
    end_lat = args.end_lat
    start_label = str(args.start_address).strip()
    end_label = str(args.end_address).strip()

    needs_geocode = any(v is None for v in [start_lon, start_lat, end_lon, end_lat])
    if needs_geocode:
        if not start_label or not end_label:
            raise ValueError("未提供完整坐标时，必须提供 --start-address 和 --end-address。")
        start_candidates = forward_geocode(start_label, args.geocode_limit, args.geocode_countrycodes)
        end_candidates = forward_geocode(end_label, args.geocode_limit, args.geocode_countrycodes)
        if not start_candidates:
            raise RuntimeError(f"起点地址无法解析坐标: {start_label}")
        if not end_candidates:
            raise RuntimeError(f"终点地址无法解析坐标: {end_label}")
        if non_interactive:
            start_pick = start_candidates[0]
            end_pick = end_candidates[0]
        else:
            start_pick = select_candidate_interactive("起点", start_label, start_candidates)
            end_pick = select_candidate_interactive("终点", end_label, end_candidates)
        start_lon, start_lat = start_pick.lon, start_pick.lat
        end_lon, end_lat = end_pick.lon, end_pick.lat
        start_label = start_pick.display_name
        end_label = end_pick.display_name
    else:
        if not start_label:
            start_label = "坐标输入"
        if not end_label:
            end_label = "坐标输入"

    assert start_lon is not None and start_lat is not None and end_lon is not None and end_lat is not None

    city = str(args.city).strip()
    if not city:
        city_guess = reverse_geocode_city(start_lat, start_lon)
        if city_guess:
            city = city_guess
    if not city:
        if non_interactive:
            raise RuntimeError("无法自动推断城市，请显式提供 --city。")
        city = input("未自动识别城市，请输入 --city: ").strip()
    if not city:
        raise RuntimeError("city 不能为空。")

    emergency_enable = True if args.emergency_enable is None else bool(args.emergency_enable)
    custom_no_fly_kmls: List[Path] = []
    for item in list(args.custom_no_fly_kml or []):
        custom_no_fly_kmls.append(_validate_kml_path(str(item)))
    existing_routes_dir: Optional[Path] = None
    if str(args.existing_routes_dir).strip():
        existing_routes_dir = _validate_existing_routes_dir(str(args.existing_routes_dir))

    print("\n请确认规划参数：", flush=True)
    print(f"1) 起点: {start_label} | lon={start_lon:.7f}, lat={start_lat:.7f}", flush=True)
    print(f"   终点: {end_label} | lon={end_lon:.7f}, lat={end_lat:.7f}", flush=True)
    print(f"2) 应急航线: {'开启' if emergency_enable else '关闭'}（默认开启）", flush=True)
    print(
        f"3) 自定义禁飞区: {'已配置' if custom_no_fly_kmls else '未配置'}（默认不加载）",
        flush=True,
    )
    print(
        "   加载方法: 重复传入 --custom-no-fly-kml /绝对路径/*.kml",
        flush=True,
    )
    print(
        f"4) 已有航线避让: {'开启' if existing_routes_dir else '关闭'}"
        "（默认不启用；开启时需提供航线库目录）",
        flush=True,
    )

    if not non_interactive:
        coords_ok = prompt_yes_no("确认起终点地址坐标？", default=True)
        if not coords_ok:
            print("用户未确认坐标，已终止。", flush=True)
            raise SystemExit(2)

        emergency_enable = prompt_yes_no("是否包含应急航线？", default=emergency_enable)

    cmd = build_command(
        planner_script=planner_script,
        city=city,
        start_lon=start_lon,
        start_lat=start_lat,
        end_lon=end_lon,
        end_lat=end_lat,
        profile=args.profile,
        top_k=args.top_k,
        name=str(args.name),
        emergency_enable=emergency_enable,
        custom_no_fly_kmls=custom_no_fly_kmls,
        existing_routes_dir=existing_routes_dir,
        passthrough=passthrough,
    )
    print("\n最终执行命令：", flush=True)
    print(" ".join(cmd), flush=True)

    if args.dry_run:
        print("dry-run 模式：未执行规划。", flush=True)
        return
    if not non_interactive and not prompt_yes_no("确认执行规划？", default=True):
        print("用户取消执行。", flush=True)
        return

    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中断。", flush=True)
        raise SystemExit(130)
