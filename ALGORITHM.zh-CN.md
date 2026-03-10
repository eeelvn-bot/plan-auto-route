# plan-auto-route 算法说明（当前主流程）

> 主入口：`skills/plan-auto-route/scripts/plan_auto_route.py`  
> 版本标识：以脚本内 `ALGORITHM_VERSION` 为准。  
> 当前：`plan-auto-route-latest-air-corridor-v4-refactor-landuse-softnofly-astarfix-existing-route-3d-avoid`

## 1. 适用范围

该算法用于城市 UAV 物流航线的 OD 自动规划，输出 3D 高度航线与风险评估元数据。

当前主流程不再依赖 `city-data-downloader` 作为前置步骤，数据由规划器在运行时按 bbox 自主准备。

## 2. 端到端流程

1. 解析输入 OD（经纬度或 `--od-kml` 首末点）。
2. 生成 route bbox（规划范围）与 building bbox（建筑障碍范围）。
3. 准备运行时数据：
   - 扫描本地 `output/auto_route_bbox_cache` 中同城缓存。
   - 复用与当前 bbox 有重叠的数据层。
   - 对缺口 bbox 执行增量 Overpass 查询。
   - 合并去重并裁切到当前 bbox。
   - 人口栅格使用本地 WorldPop，并裁切到当前 bbox。
4. 构建风险索引与约束层（人口、landuse、设施、线性风险、学校、禁飞、建筑障碍）。
5. 构建导航图（road/water + oblique air lattice）。
6. 按候选策略运行 A* 风险感知搜索。
7. 后处理（转角约束、裁弯、航点约束、候选比较）。
8. 垂向高度规划（DEM 或 OpenTopoData + 建筑/障碍净空约束）。
9. 输出 KML/HTML/meta/candidates/pareto/coverage/evidence/snapshot。

## 3. 数据源与获取策略

### 3.1 本地文件（必须/可选）

- 民航禁飞区：`config/civil_airport_no_fly.geojson`（本地读取，CAAC 转换结果）。
- 自定义禁飞区：`--custom-no-fly-kml`（可多次传入，本地读取）。
- DEM（可选）：`--dem-tif`。

### 3.2 在线数据（按 bbox）

- Overpass：
  - POI（已排除 `school/kindergarten`，避免与学校硬约束重复）
  - landuse
  - roads / HSR
  - hydro（水面与水系）
  - line-risk（highway/hsr/high-voltage power line）
  - school/kindergarten（硬约束专用）
  - building/obstacle
  - infrastructure
  - military/heli no-fly
- OpenTopoData：仅在未提供 DEM 时采样地形。

### 3.3 人口栅格

- WorldPop China 1km 数据（本地不存在时首次下载）。
- 每次运行裁切到当前 bbox，作为本次规划人口采样输入。

## 4. 缓存与增量复用

运行缓存目录：`output/auto_route_bbox_cache/<city>_<bbox_hash>/`

每次运行会生成 `download_summary.json`，并记录：
- `cache_reuse.overlap_cache_count`
- `cache_reuse.incremental_query_bbox_count`
- `cache_reuse.incremental_query_bboxes`

策略说明：
- 若历史缓存与新 bbox 有重叠：优先复用重叠层数据。
- 仅对未覆盖区域发起增量查询。
- 合并后按要素 ID/几何去重，再裁切回目标 bbox。

## 5. 约束体系

### 5.1 硬约束

- 民航机场禁飞区（CAAC 本地数据）
- 军用机场禁飞区（开放数据）
- 学校/幼儿园避让区（硬缓冲）
- 关键基础设施硬缓冲（按参数启用）
- 既有航线 3D 避让约束（按参数启用）
- 高度硬上限（默认 200m）

### 5.2 软约束与风险项（代价函数）

- 距离
- 人口暴露
- landuse 风险
- 基础设施邻近风险
- 建筑/障碍高度压力
- 软禁飞惩罚
- 人群点与关键设施点惩罚
- 线性风险（高速/高铁/高压线）叠覆惩罚
- 转弯惩罚

## 6. 候选与选线

默认内部候选包含：
- `safety_default`
- `efficiency`

选择机制：
- 先计算候选的风险、距离、垂向工作量等指标。
- 可启用 Pareto 选择。
- 可启用垂向权衡切换（在可接受绕行比内优先降低爬升/下降工作量）。

## 7. 高度规划

高度规划使用以下约束：
- 地形（DEM/OpenTopoData）+ 建筑/障碍顶面
- 最小净空
- 爬升/下降能力包线
- 真高上下限与端点真高目标

输出为绝对高程航点（KML），并同步生成剖面样本用于 HTML 和质量评估。

## 8. 输出清单

- `<name>.kml`
- `<name>.html`
- `<name>_meta.json`
- `<name>_candidates.json`
- `<name>_pareto.json`
- `<name>_coverage.json`
- `<name>_evidence.json`
- `snapshots/<name>_snapshot.json`

## 9. 当前唯一可执行入口

只使用：
- `scripts/plan_auto_route.py`

历史脚本与旧文档仅保留在 `archive/`，不参与运行流程。
