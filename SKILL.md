---
name: plan-auto-route
description: Build automatic urban UAV logistics routes from OD points with open-data no-fly constraints, risk-aware path search, and altitude profile planning (terrain + buildings + obstacles + flight envelope).
---

# Plan Auto Route Skill

Use this skill when you need a direct OD-to-route planner (not only KML post-refinement) for urban logistics UAV missions.
This skill uses only the latest `plan_auto_route.py` algorithm (air-corridor + risk-aware graph search).
Do not substitute older road-aligned refinement flows (for example `v72-route-planner`) when this skill is requested.
For reproducibility comparisons, the frozen `opt_v15` baseline is kept at
`skills/plan-auto-route/scripts/plan_auto_route_v15_legacy.py`.

## Dependency

- Required upstream data skill: `city-data-downloader`
- Repository: https://github.com/eeelvn-bot/city-data-downloader
- Recommended order:
  1) run `city-data-downloader` to prepare city POI/landuse/hydro/population/transport layers
  2) run `plan-auto-route` for OD planning and altitude profile generation

## What it does

1. Builds a route graph from open data (roads + waterways).
   - Includes an oblique free-air lattice so planner can use low-risk straight air corridors instead of being locked to roads.
2. Applies hard constraints:
   - Open-data hard no-fly zones:
     - Civil airports use CAAC dataset (`config/civil_airport_no_fly.geojson`, converted from 民航机场禁飞区.xls).
     - Military airport areas still come from OSM military tags.
   - Optional hard buffer around critical infrastructure.
   - Schools/kindergartens use hard avoidance buffers; universities are excluded from school-sensitive hard constraints.
3. Optimizes a risk-aware objective:
   - Distance
   - Population exposure (air-corridor crossing uses multi-point density sampling)
   - Landuse risk
   - Critical infrastructure proximity
     - Includes P1/P2 categories from OSM:
       - P1: `man_made=storage_tank|works|pipeline` (hazard tags increase severity)
       - P2: `man_made=water_works|wastewater_plant|pumping_station`, `waterway=dam`, `power=substation|plant`
   - Height-pressure proxy (buildings/obstacles)
   - Turning penalty (road-turn suppression with corrected angle model)
   - Waterway priority candidate (accepted when detour is within 10%)
   - Linear risk crossing penalty includes highway / HSR / high-voltage power lines (`power=line|minor_line` with `voltage>=220kV`)
   - Sensitive POI includes government + military facilities in key-facility penalties
   - Post-search corner reduction with population-aware shortcut checks
   - Low-yield bend pruning + optional waypoint budget control
4. Plans altitude profile along the final path:
   - Terrain + building + obstacle top surface
   - Clearance
   - Climb/descend envelope from airframe parameters
   - Cruise-first profile strategy: uses route-top reference to keep long flat segments, then descends near destination when feasible
   - KML waypoint compression: straight/no-turn segments avoid dense point injection; extra points are kept only when needed for altitude/safety constraints
5. Outputs KML/HTML/meta and can optionally run `workflow_v2`.
6. Exports two explicit route layers in HTML (no third "main route" layer):
   - `安全优先（3D高度）` + `安全优先 缓冲区 100m`
   - `效率优先（3D高度）` + `效率优先 缓冲区 100m`
   Both are independently toggleable in LayerControl.
7. Waypoint editor supports selecting the baseline route (`安全优先` / `效率优先`) before drag-editing and KML export.
8. Vertical profile panel supports switching between `安全优先` and `效率优先` profile curves.
9. Basemap high-zoom behavior is hardened:
   - `普通地图` and `卫星影像` support high-zoom overzoom rendering.
   - `卫星注记` follows the same high-zoom cap so labels remain consistent.
10. Supports minimal-change replanning with a reference route:
   - Uses `--reference-kml` as baseline and adds deviation penalty in graph search.
   - Enforces hard filters on detour ratio and mean offset vs reference route.
   - Writes reference comparison metrics to candidate/meta outputs.

## Command

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "合肥市" \
  --start-lon 117.2788 --start-lat 31.8030 \
  --end-lon 117.2110 --end-lat 31.8640 \
  --name "hefei_auto_route_demo" \
  --open-data-no-fly \
  --profile balanced \
  --top-k 1
```

## Inputs

- OD by coordinates:
  - `--start-lon --start-lat --end-lon --end-lat`
- Or OD from KML endpoints:
  - `--od-kml /absolute/path/to/route.kml`
- Optional reference route for minimal-change replanning:
  - `--reference-kml /absolute/path/to/reference_route.kml`

## Key options

- `--profile`: `fastest | balanced | safest`
- `--select-candidate`: `safety_default | efficiency`
  - default is `safety_default` (main output route remains safety-first by default)
- `--top-k`: number of alternatives to export (current phase exports best route + candidate summary)
- `--weight-sweep-levels`: deprecated in 2-candidate mode (accepted for compatibility but ignored)
- `--pareto-select`: allow final selection from Pareto front (`distance_km`, `path_population_p90`, `vertical_energy_proxy_m`)
- `--pareto-detour-limit-ratio`: optional override for max distance ratio allowed in Pareto switching
- `--pareto-distance-weight`: optional override for distance weight in Pareto score
- `--pareto-pop-weight`: optional override for population p90 weight in Pareto score
- `--pareto-energy-weight`: optional override for vertical energy weight in Pareto score
- `--reference-kml`: optional reference KML for minimal-change replanning
- `--reference-corridor-m`: normalization corridor for reference offset penalty
- `--reference-deviation-weight`: global multiplier for reference deviation penalty
- `--reference-max-detour-ratio`: hard filter for max distance ratio vs reference route
- `--reference-max-mean-offset-m`: hard filter for max mean offset vs reference route
- `--pareto-max-front`: max Pareto front entries kept in summary JSON
- `--pareto-policy-file`: optional policy JSON path (default `skills/plan-auto-route/config/pareto_policies.json`)
- `--pareto-policy-name`: base policy name in policy JSON (default `default`)
- `--pareto-business-line`: business-line override key (default `default`)
- `--open-data-no-fly`: enable no-fly filtering (civil airports from CAAC dataset + heliport/helipad/military from OSM/Overpass)
- `--civil-airport-no-fly-geojson`: path to civil-airport no-fly dataset (default `skills/plan-auto-route/config/civil_airport_no_fly.geojson`)
- `--existing-routes-dir`: optional directory containing existing-route KML files; loaded as 3D hard-avoid constraints
- `--existing-route-horizontal-buffer-m`: existing-route horizontal hard-avoid half-width (default `30m`)
- `--existing-route-vertical-buffer-m`: existing-route vertical hard-avoid half-width (default `20m`)
- `--existing-route-endpoint-relief-m`: relax existing-route 3D hard-avoid within this radius around start/end (default `200m`)
- `--soft-no-fly-scale`: adjust soft no-fly penalty strength (e.g., `0.6` more efficiency, `1.5` more conservative)
- `--safety-sensitive-hard-buffer-m`: hard exclusion buffer around sensitive facilities for safety route
- `--safety-infra-hard-buffer-m`: hard exclusion buffer around critical infrastructure for safety route
- `--clearance-m`: minimum clearance above local top surface (terrain/building/obstacle)
- `--endpoint-true-height-m`: start/end minimum true height above terrain (default `60m`; may climb higher for hard obstacle clearance)
- `--min-true-height-m`: route true-height floor above terrain (default `60m`)
- `--max-true-height-m`: route true-height upper target above terrain (default `120m`)
  - endpoint vicinity uses a short clearance ramp so takeoff/landing can keep fixed true height while transitioning to full obstacle-clearance constraints
- `--speed-ms --climb-ms --descend-ms --turn-radius-m`: small multirotor envelope assumptions
- `--min-turn-keep-deg`: keep turns above this threshold; lower-angle bends are pruned when shortcut is safe
- `--min-turn-angle-deg`: hard minimum interior turn angle for final route (default `120`; forbids sharp acute turns)
- `--turn-prune-passes`: number of low-yield turn pruning passes
- `--max-waypoints`: optional hard cap for post-processed horizontal waypoints (`0` = auto/no hard cap)
- `--vertical-tradeoff`: enable detour-vs-climb/descend workload trade-off in candidate selection
- `--vertical-detour-limit-ratio`: max accepted detour ratio for vertical trade-off switching
- `--vertical-improve-ratio`: minimum vertical workload reduction required to switch candidate
- `--dem-tif`: optional DEM GeoTIFF for terrain sampling (preferred); without it, OpenTopoData SRTM is used
- `--run-workflow-v2`: run layered review and RA output
- `--write-evidence / --no-write-evidence`: enable/disable structured evidence JSON output
- `--write-snapshot / --no-write-snapshot`: enable/disable run snapshot manifest output
- `--snapshot-dir`: optional custom output directory for snapshots

## Outputs

- Route KML: `output/auto_routes/<name>.kml`
- Candidate KMLs:
  - `output/auto_routes/<name>_safety_default.kml`
  - `output/auto_routes/<name>_efficiency.kml`
- Preview map: `output/auto_routes/<name>.html`
- Plan meta: `output/auto_routes/<name>_meta.json`
- Candidate summary: `output/auto_routes/<name>_candidates.json`
- Pareto summary: `output/auto_routes/<name>_pareto.json`
- Coverage summary: `output/auto_routes/<name>_coverage.json`
- Evidence pack: `output/auto_routes/<name>_evidence.json`
- Snapshot manifest: `output/auto_routes/snapshots/<name>_snapshot.json`
- Optional layered review HTML:
  `output/full-workflow-v2-auto-route/<name>/<name>_map.html`

When `--reference-kml` is enabled, outputs also include:
- `reference_mode_enabled`
- `reference_route` summary in meta
- candidate-level `detour_ratio_vs_reference`
- candidate-level `mean_offset_to_reference_m` / `p90_offset_to_reference_m` / `max_offset_to_reference_m`

## Notes

- This is phase-1 auto planner: OD automation + hard no-fly + risk-aware path + altitude envelope.
- Civil-airport hard no-fly now depends on CAAC dataset geometry quality; military/heliport still depends on OSM completeness.
- Overpass queries are cached in `output/overpass_cache` to improve reproducibility and reduce API jitter impact.
- GIS city datasets are cache-first: planner checks local `output/city_data/<city>` completeness first, and only downloads missing data.
- To regenerate the civil-airport dataset from XLS: run `python3 skills/plan-auto-route/scripts/build_civil_airport_no_fly_geojson.py --xls /Users/leonzhao/dailywork/民航机场禁飞区.xls`.
- For regulated missions, keep SORA/JARUS compliance checks as a separate gate.
- Current refactor keeps script backup at `skills/plan-auto-route/scripts/plan_auto_route_pre_refactor_backup.py`.
- Built-in Pareto policy file: `skills/plan-auto-route/config/pareto_policies.json`.

## Recommended Pareto Templates

Two built-in templates are provided in `skills/plan-auto-route/config/pareto_policies.json`:

- `urban_dense`
  - For dense urban delivery: stronger population-risk weight, tighter detour limit.
  - Effective defaults: `detour_limit_ratio=1.20`, `distance_weight=0.95`, `population_weight=1.55`, `energy_weight=1.35`.
- `suburban_logistics`
  - For suburban logistics: more distance/efficiency oriented with moderate energy control.
  - Effective defaults: `detour_limit_ratio=1.32`, `distance_weight=1.22`, `population_weight=0.78`, `energy_weight=1.08`.

### Example: Dense Urban

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "深圳市" \
  --start-lon 114.0579 --start-lat 22.5431 \
  --end-lon 114.1392 --end-lat 22.5017 \
  --name "sz_dense_policy" \
  --profile safest \
  --pareto-select \
  --pareto-policy-name urban_dense
```

### Example: Suburban Logistics

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "合肥市" \
  --start-lon 117.2788 --start-lat 31.8030 \
  --end-lon 117.2110 --end-lat 31.8640 \
  --name "hf_suburban_policy" \
  --profile balanced \
  --pareto-select \
  --pareto-policy-name suburban_logistics
```
