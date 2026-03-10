# plan-auto-route 算法简明说明与使用指南

本指南对应当前脚本：
`skills/plan-auto-route/scripts/plan_auto_route.py`

适用场景：城市无人机物流 OD（起终点）自动航线规划，输出可用于方案比选与风险评估前置输入。

## 1. 算法做什么（简版）

`plan-auto-route` 的核心流程：

1. 读入 OD（坐标或 KML 首尾点）和城市缓存数据（人口、土地利用、路网、水系等）。
2. 构建导航图：道路/水系 + 斜向空域网格（air lattice），并连接起终点。
3. 施加硬约束：禁飞区、学校/敏感设施/关键基础设施硬缓冲等不可穿越区域。
4. 进行风险感知 A* 搜索：综合距离、人口暴露、土地利用、基础设施（含 P1/P2：储罐/管线/工厂、给排水与大坝等）、软禁飞、线性风险、转弯惩罚等代价。
5. 生成两条候选航线：
   - `safety_default`（安全优先，默认主航线）
   - `efficiency`（效率优先）
6. 对候选航线做后处理：弯折裁剪、转角约束、航点压缩。
7. 做高度剖面规划：结合地形 + 建筑/障碍物顶面 + 净空与爬升下降包线，输出绝对高度航点。
8. 输出 KML/HTML/Meta/候选汇总/Pareto 汇总（可选 evidence 与 snapshot）。

## 2. 输入与输出

### 输入方式

- 坐标输入（二选一）：
  - `--start-lon --start-lat --end-lon --end-lat`
- KML 输入（二选一）：
  - `--od-kml /absolute/path/to/route.kml`（取首点和尾点作为 OD）

### 主要输出（默认目录 `output/auto_routes/`）

- `<name>.kml`：最终航线（含 absolute 高度）
- `<name>_safety_default.kml`：安全优先候选航线
- `<name>_efficiency.kml`：效率优先候选航线
- `<name>.html`：预览地图（可切换候选图层）
- `<name>_meta.json`：主航线元数据
- `<name>_candidates.json`：候选路线摘要
- `<name>_pareto.json`：Pareto 前沿摘要
- `<name>_evidence.json`：证据包（开启 `--write-evidence` 时）
- `snapshots/<name>_snapshot.json`：运行快照（开启 `--write-snapshot` 时）

## 3. 最小可用命令（推荐先跑通）

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "合肥市" \
  --start-lon 117.2788 --start-lat 31.8030 \
  --end-lon 117.2110 --end-lat 31.8640 \
  --name "hf_quickstart" \
  --open-data-no-fly \
  --profile balanced \
  --top-k 1
```

## 4. 常用参数（按优先级）

### A. 选线与风格

- `--profile fastest|balanced|safest`：全局风格模板
- `--select-candidate safety_default|efficiency`：指定主输出候选
- `--pareto-select`：允许基于距离/人口/垂向工作量从 Pareto 前沿切换最终路线
- `--pareto-policy-name default|conservative|efficiency|urban_dense|suburban_logistics`

### B. 约束与风险

- `--open-data-no-fly`：启用开源禁飞约束
- `--soft-no-fly-scale`：软禁飞惩罚强度（大=更保守）
- `--safety-sensitive-hard-buffer-m`：敏感设施硬缓冲
- `--safety-infra-hard-buffer-m`：关键基础设施硬缓冲

### C. 高度与飞行包线

- `--clearance-m`：相对局部顶面净空
- `--endpoint-true-height-m`：起降点真高
- `--min-true-height-m --max-true-height-m`：航段真高上下限
- `--speed-ms --climb-ms --descend-ms --turn-radius-m`：机体能力参数

### D. 平滑与航点控制

- `--min-turn-keep-deg`：低收益弯折保留阈值
- `--min-turn-angle-deg`：最小转角硬约束（默认较保守）
- `--turn-prune-passes`：弯折裁剪轮数
- `--max-waypoints`：航点硬上限（`0` 为自动）

## 5. 三种典型使用模板

### 5.1 安全优先（默认推荐）

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "深圳市" \
  --start-lon 114.0579 --start-lat 22.5431 \
  --end-lon 114.1392 --end-lat 22.5017 \
  --name "sz_safety" \
  --open-data-no-fly \
  --profile safest \
  --select-candidate safety_default \
  --pareto-select \
  --pareto-policy-name urban_dense
```

### 5.2 效率优先

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "合肥市" \
  --start-lon 117.2788 --start-lat 31.8030 \
  --end-lon 117.2110 --end-lat 31.8640 \
  --name "hf_efficiency" \
  --open-data-no-fly \
  --profile fastest \
  --select-candidate efficiency \
  --soft-no-fly-scale 0.8
```

### 5.3 垂向工作量优化（更关注爬升下降负担）

```bash
python3 skills/plan-auto-route/scripts/plan_auto_route.py \
  --city "杭州市" \
  --start-lon 120.1500 --start-lat 30.2800 \
  --end-lon 120.2200 --end-lat 30.3300 \
  --name "hz_vertical_tradeoff" \
  --open-data-no-fly \
  --vertical-tradeoff \
  --vertical-detour-limit-ratio 1.20 \
  --vertical-improve-ratio 0.10 \
  --vertical-energy-weight 1.4
```

## 6. 常见问题与排查

- 报错 `No feasible route found`：
  - 降低硬缓冲（如 `--safety-sensitive-hard-buffer-m`）。
  - 放宽转角限制（如降低 `--min-turn-angle-deg`）。
  - 确认 OD 不在硬禁飞区内部。
- 报错 `No feasible route after altitude constraints`：
  - 增大 `--max-true-height-m` 或 `--hard-ceiling-m`。
  - 调整 `--clearance-m` 与爬升能力（`--climb-ms`）。
- 路线太绕：
  - 使用 `--select-candidate efficiency` 或 `--profile fastest`。
  - 降低 `--soft-no-fly-scale`，并检查 Pareto/vertical tradeoff 是否过于保守。

## 7. 使用边界（务必注意）

- 本算法用于自动规划与风险评估支撑，不等于运营放飞许可。
- 开源禁飞和障碍数据可能不完整，实际运行前需做独立核验。
- 建议把本输出接入后续 SORA/JARUS 或企业合规流程作为独立 gate。
