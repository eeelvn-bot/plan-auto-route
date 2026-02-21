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
   - Open-data hard no-fly zones (airport and major military areas).
   - Optional hard buffer around critical infrastructure.
   - Minor military tags are treated as soft penalties by default.
3. Optimizes a risk-aware objective:
   - Distance
   - Population exposure (air-corridor crossing uses multi-point density sampling)
   - Landuse risk
   - Critical infrastructure proximity
   - Height-pressure proxy (buildings/obstacles)
   - Turning penalty (road-turn suppression with corrected angle model)
   - Waterway priority candidate (accepted when detour is within 10%)
   - Post-search corner reduction with population-aware shortcut checks
   - Low-yield bend pruning + optional waypoint budget control
4. Plans altitude profile along the final path:
   - Terrain + building + obstacle top surface
   - Clearance
   - Climb/descend envelope from airframe parameters
   - Cruise-first profile strategy: uses route-top reference to keep long flat segments, then descends near destination when feasible
   - KML waypoint compression: straight/no-turn segments avoid dense point injection; extra points are kept only when needed for altitude/safety constraints
5. Outputs KML/HTML/meta and can optionally run `workflow_v2`.
6. Exports three explicit candidate layers in HTML:
   - `1) 安全优先 + 水路偏好`
   - `2) 安全优先（默认主航线）`
   - `3) 效率优先`
   You can toggle each in LayerControl.

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

## Key options

- `--profile`: `fastest | balanced | safest`
- `--select-candidate`: `safety_water | safety_default | efficiency`
  - default is `safety_default` (main output route remains safety-first by default)
- `--top-k`: number of alternatives to export (current phase exports best route + candidate summary)
- `--weight-sweep-levels`: generate extra sweep candidates between safety-default and efficiency for trade-off exploration
- `--pareto-select`: allow final selection from Pareto front (`distance_km`, `path_population_p90`, `vertical_energy_proxy_m`)
- `--pareto-detour-limit-ratio`: optional override for max distance ratio allowed in Pareto switching
- `--pareto-distance-weight`: optional override for distance weight in Pareto score
- `--pareto-pop-weight`: optional override for population p90 weight in Pareto score
- `--pareto-energy-weight`: optional override for vertical energy weight in Pareto score
- `--pareto-max-front`: max Pareto front entries kept in summary JSON
- `--pareto-policy-file`: optional policy JSON path (default `skills/plan-auto-route/config/pareto_policies.json`)
- `--pareto-policy-name`: base policy name in policy JSON (default `default`)
- `--pareto-business-line`: business-line override key (default `default`)
- `--open-data-no-fly`: enable airport/heliport/military no-fly polygons from OSM/Overpass
- `--soft-no-fly-scale`: adjust soft no-fly penalty strength (e.g., `0.6` more efficiency, `1.5` more conservative)
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
- Preview map: `output/auto_routes/<name>.html`
- Plan meta: `output/auto_routes/<name>_meta.json`
- Candidate summary: `output/auto_routes/<name>_candidates.json`
- Pareto summary: `output/auto_routes/<name>_pareto.json`
- Evidence pack: `output/auto_routes/<name>_evidence.json`
- Snapshot manifest: `output/auto_routes/snapshots/<name>_snapshot.json`
- Optional layered review HTML:
  `output/full-workflow-v2-auto-route/<name>/<name>_map.html`

## Notes

- This is phase-1 auto planner: OD automation + hard no-fly + risk-aware path + altitude envelope.
- Open-data no-fly coverage depends on OSM completeness and should be verified before operations.
- Overpass queries are cached in `output/overpass_cache` to improve reproducibility and reduce API jitter impact.
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
  --weight-sweep-levels 4 \
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
  --weight-sweep-levels 3 \
  --pareto-select \
  --pareto-policy-name suburban_logistics
```
