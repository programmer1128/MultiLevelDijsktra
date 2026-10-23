# Routing Engine — Fix & Integration Plan

Created: 2026-10-18 · Baseline commit: `a233c5c`

A per-file record of every identified flaw, what it breaks, and how to fix it. Work through it by **phase** (bottom of this file); tick the checkbox on each issue as it lands.

---

## Legend

**Problem areas**

| Code | Problem |
|---|---|
| **P1** | User location snaps to the wrong road (esp. dual carriageways) |
| **P2** | No live HMM map matching in the backend |
| **P3** | Routes loop / straight roads lose to turns |
| **P4** | No vehicle profiles (car / motorcycle / foot) |
| **PERF** | Live-navigation latency and concurrency |
| **HYG** | Data-pipeline hygiene and latent bugs |

**Symptom codes from the original analysis** (referenced below)

- S1 heading term is a no-op · S2 forward/reverse EBG nodes indistinguishable · S3 snaps to vertices, not roads · S4 snaps to footways/service roads · S5 degree-space radius · S6 no phantom nodes · S7 wrong corridor bearings to client
- L1 wrong-direction origin/destination → U-turn loops · L2 flat turn costs · L3 phantom intersection penalties · L4 inverted speed table · L5 incomplete one-way handling · L6 no access/barriers/full restrictions · L7 cheap U-turns everywhere

**Severity:** 🔴 high (directly causes reported bugs) · 🟠 medium · 🟢 low / cleanup

---

## Baseline audit (2026-10-18, current `parsing/*.bin`)

Re-run after the Phase 2 rebuild and compare.

| Metric | Value |
|---|---|
| EBG nodes / base edges | 46,836,006 / 131,660,654 |
| Turn penalty values present | **2 s on 85.4M edges, 45 s (U-turn) on 46.2M edges, nothing else** |
| Nodes clamped to `wt == 1` | 3,146,302 (6.7%) |
| Segments < 25 m | 14.5% |
| Segment length median / p90 / p99 | 69 m / 356 m / 1,937 m |
| road_type 0 ("other", 30 km/h) | 11.85M nodes (25%) |
| road_type 1 (residential, 15 km/h) | 27.44M nodes (59%) |
| road_type 5 (footway/path) | 631k nodes |
| One-way flagged | 2.4% |
| Cells / overlay shortcuts | 5,947 / 12,275,623 |
| Edges with underflowed weight (> 1e6) | 0 (`patch_u_turns` was never applied) |

**Verified OK — do not spend time here**

- Overlay is consistent with current `cell_id`s in `metadata.bin`: 100% of shortcuts are intra-cell, entry → exit.
- Bidirectional MLD stopping criterion and meeting-node logic are correct.
- `reverse_graph_generator.cpp` correctly reverses both base and overlay graphs.
- Weight convention is consistent end-to-end: edge `u→v` = turn cost + `wt[v]`.

---

## Offline pre-processing

### `parsing/parser2.cpp`

#### PARSE-01 · Every `highway=*` value is accepted as a road 🔴 `[ ]`
- **Where:** `NodeReferenceCounter::way` L35-40, `Pass2Handler::way` L144-145
- **Logic today:** any way with a `highway` tag enters the graph, including `construction`, `proposed`, `abandoned`, `platform`, `steps`, `corridor`, `bus_stop`, `raceway`, `elevator`.
- **Causes:** P3 (routes over non-existent or non-drivable ways), P1/S4 (snap candidates on them). → L4, L6
- **Fix:** add an explicit classification step driven by the profile config (NEW-01):
  - *exclude entirely* non-existent or non-way values (`proposed`, `construction`, `abandoned`, `razed`, `platform`, `bus_stop`, …);
  - keep every real way in the topology, but assign per-mode access (PARSE-05).
  - The same filter must apply in the node counter (L35) **and** the NBG builder (L144), otherwise intersection detection diverges. See STITCH-01.
- **Rebuild:** full chain, including Dinic (the NBG changes).

#### PARSE-02 · Ways split at every node shared with *any* highway 🔴 `[ ]`
- **Where:** `NodeReferenceCounter` L43-48, split condition L208-212
- **Logic today:** a node referenced by ≥2 highway ways (footway, path, service included) becomes an intersection, so a main road is chopped at every footpath that touches it.
- **Causes:** P3/L3. Each split adds a 2 s turn edge, even when going straight along the same road, plus the 1 s minimum-weight rounding. Arterials accumulate tens of seconds of fake cost compared with parallel lanes.
- **Fix:** keep the splits (the foot profile needs that topology), but make the turn cost **0 at pass-through junctions**, i.e. where the profile has only one drivable continuation excluding the U-turn. That logic lives in PARSE-08. Optionally, do per-profile degree-2 compaction later.

#### PARSE-03 · One-way tags parsed incompletely 🔴 `[ ]`
- **Where:** L147-152
- **Logic today:** only `oneway=yes|1` is recognised.
- **Causes:** P3/L5. `oneway=-1` is treated as two-way, and implied one-ways are missed (`junction=roundabout|circular`, `highway=motorway`). Only 2.4% of nodes are flagged one-way, and roundabouts can be driven the wrong way.
- **Fix:**
  - Parse `yes|true|1` → forward, `-1|reverse` → reverse-only (emit only the reverse EBG node), `no|false|0` → two-way.
  - Apply the implied one-ways.
  - Honour mode overrides (`oneway:motorcycle`, `oneway:foot`); foot ignores one-way by default.
  - For foot, both directions must exist in the topology (see PARSE-10).

#### PARSE-04 · Speed table inverted and incomplete; `maxspeed` parsing naive 🔴 `[ ]`
- **Where:** speed table L157-175, `maxspeed` override L178-182
- **Logic today:**
  - The default 30 km/h applies to everything unlisted: `service`, `unclassified`, `track`, `*_link`, `living_street`, `cycleway`, … That is 25% of the graph, and it gets 2× residential's speed (15 km/h).
  - `atoi("50 mph")` is read as 50 km/h.
  - Symbolic values (`IN:urban`, `signals`, `none`) are ignored.
  - `maxspeed` can make a footway fast.
- **Causes:** P3/L4. The router cuts through parking aisles, petrol pumps and tracks, producing zigzag and loop shapes.
- **Fix:**
  - Move the table into the profile config (NEW-01) with a sane order: motorway > trunk > primary > secondary > tertiary > unclassified ≈ residential > service (~10 km/h; `service=parking_aisle|driveway|drive-through` lower or excluded) > living_street. `*_link` ≈ 60–70% of its parent road.
  - Parse units (`mph`, `knots`) and country symbolic values.
  - Clamp `maxspeed` to `[profile_min, profile_max]` for that highway class.

#### PARSE-05 · No access tags, no barrier nodes 🔴 `[ ]`
- **Where:** absent. Pass 2 (L300-305) reads nodes only for their locations.
- **Causes:** P3/L6 (private roads, gated colonies and `motor_vehicle=no` ways are routable), P1/S4 (no way to filter snap candidates by mode), P4 (no per-mode access at all).
- **Fix:**
  - Resolve the access hierarchy per mode: `access` → `vehicle` → `motor_vehicle` → `motorcar` / `motorcycle`; `foot` separately. Handle `private` / `destination` / `no`.
  - Add a node-tag pass to collect `barrier=*` nodes (gate, bollard, lift_gate, block, …) and `highway=traffic_signals` nodes (used by PARSE-08).
  - Emit an `access_mask` byte per EBG node (bit 0 car, 1 motorcycle, 2 foot) into metadata (PARSE-11).

#### PARSE-06 · Distance truncated to whole metres per vertex pair 🟠 `[ ]` *(found while compiling this record)*
- **Where:** `calculate_distance` returns `uint32_t` (L96-109); summed per pair at L200-206.
- **Logic today:** every vertex-to-vertex distance is floored before summing, so curved roads with dense vertices (3–5 m spacing) lose up to 1 m per pair.
- **Causes:** P3 (curvy roads systematically too cheap), P2 (HMM transition probabilities compare route distance against GPS distance, so biased lengths hurt matching).
- **Fix:** compute and accumulate in `double`; round once per segment. Store decimetres if you want sub-metre precision.

#### PARSE-07 · Weights in integer seconds with `max(1, …)` clamp 🟠 `[ ]`
- **Where:** L222
- **Causes:** P3/L3. 6.7% of nodes are clamped to 1 s, and rounding error accumulates per segment; short segments are over-penalised.
- **Fix:** switch the weight unit to **deciseconds** (OSRM's choice). `uint32` deciseconds is plenty. Compute from double distance (PARSE-06). Document the unit in the manifest (NEW-03). Every consumer must use saturating addition (MLD-03, OVL-01).

#### PARSE-08 · Turn costs flat; turn angle never computed; U-turn rule wrong 🔴 `[ ]`
- **Where:** Phase 4 L586-597; `ExpandedEdge::turnAngle` (GraphStructures.hpp L61) is never written.
- **Logic today:**
  - Every non-U-turn costs 2 s; a U-turn costs 45 s and is allowed at every intersection.
  - The U-turn test `to.end == from.start` also flags turns onto a *different* way that connects the same two intersections.
- **Causes:** P3/L2 (straight has no advantage over turns), L7 (cheap U-turns enable loops), L3 (pass-through penalties).
- **Fix:**
  1. Compute the turn angle from **direction-aware** bearings: the last geometry piece of the incoming node vs. the first piece of the outgoing node, honouring `is_reverse_geometry`. Store it in `turnAngle`.
  2. Make the cost a function of angle, with a left-hand-traffic bias: straight (±20°) ≈ 0, near-side (left) small, far-side (right) larger, sharp larger again. Parameters go in the profile config.
  3. Pass-through junction (one drivable continuation for the profile) → 0 cost.
  4. Add a `traffic_signals` node penalty (from the PARSE-05 node pass).
  5. U-turns: detect via the **twin** node (same `geometry_offset`, opposite direction; see PARSE-11). Forbid them except at dead-ends for the profile, or apply a very large penalty.

#### PARSE-09 · Turn restrictions: via-way dropped, mode tags ignored, linear scan 🟠 `[ ]`
- **Where:** `TurnRestrictionHandler` L55-92; matching loop L570-582
- **Logic today:**
  - A relation whose `via` member is a way keeps `via_node_id = 0` and is discarded.
  - Only the `restriction` key is read; `restriction:motorcar`, `restriction:motorcycle` and `except=` are ignored.
  - Every candidate turn scans **all** restrictions: O(turns × R).
- **Causes:** P3/L6 (illegal turns are taken), P4 (no mode-specific restrictions), slow preprocessing.
- **Fix:**
  - Index restrictions in a hash map keyed by `(from_way_id, via_node_id)`.
  - Parse mode-specific keys and `except=` into per-profile banned flags; in the multi-profile world a ban becomes INF in that profile's turn metric.
  - Via-way restrictions need EBG node duplication. Defer them, but log the count so you know what is being dropped.

#### PARSE-10 · Topology and weight fused in `RoutingEdge`; unsaturated add 🟠 `[ ]`
- **Where:** Phase 5 L612-631 (the fusion is at L628)
- **Causes:** P4 (one topology can't carry several profiles), HYG (INF + weight overflows `uint32`).
- **Fix:**
  - *Short term:* saturating add.
  - *Phase 5:* write `routing_topology.bin` (`to`, `turn_angle`) and parallel `edge_weights_<profile>.bin`. Generate both directions of every way and every turn in the topology; a profile marks forbidden ones with INF.

#### PARSE-11 · `metadata.bin` missing fields and a header 🟠 `[ ]`
- **Where:** Phase 3.9 L523-546
- **Logic today:** a `size_t` count followed by 53-byte packed records. No magic or version. No twin id, access mask, or travel-direction bearings.
- **Causes:** P1/S2 (the server can't find a node's opposite direction or its true bearing), P1/S4 (no access mask), HYG (silent layout drift).
- **Fix:** add a header (magic, version, record size, count) and fields `twin_id` (UINT32_MAX if one-way), `access_mask`, `bearing_start`, `bearing_end` (int16 decidegrees, in travel direction).
  - **Every reader must change in the same commit:** `sticher.cpp`, `OverlayGraphConstructor.cpp::loadCellIDs`, `GraphDataView.hpp::PackedMetadata`.

#### PARSE-12 · No segment-level spatial index is produced 🔴 `[ ]`
- **Where:** absent; the server builds a KD-tree at startup instead.
- **Causes:** P1/S3, S4. Also needed by P2 for HMM candidate generation.
- **Fix:** see NEW-02. A separate tool reading `geometry.bin` + `metadata.bin` keeps the parser lean.

#### PARSE-13 · Partition hot-load depends on NBG ordering 🟢 `[ ]`
- **Where:** L404-458
- **Logic today:** reads `node_cell_mapping.bin` indexed by NBG id. After PARSE-01/05 change the NBG, the old mapping is invalid. The size check catches count changes only.
- **Fix:** after any filter change, rerun Dinic. Enforce this in the pipeline script (NEW-03) via the manifest's NBG checksum.

#### PARSE-14 · `name_id` is always 0 🟢 `[ ]` *(found while compiling this record)*
- **Where:** L229
- **Causes:** no street names for turn-by-turn instructions, and harder HMM debugging.
- **Fix:** build a string dictionary (`names.bin` + offsets). Low priority, but cheap to add during the Phase 2 rebuild.

#### PARSE-15 · Hard-coded input and output paths 🟢 `[ ]`
- **Where:** L266 and all output filenames
- **Fix:** take CLI arguments (input pbf, output dir, profile dir) so the pipeline script can drive it.

---

### `partitioning/sticher.cpp`

#### STITCH-01 · Re-implements the parser's NBG id assignment by re-parsing OSM 🟠 `[ ]`
- **Where:** `NodeReferenceCounter` L17-36, `NBGIndexRebuilder` L39-73
- **Logic today:** reconstructs NBG ids by replaying parser passes 1A and 2. Any filter change in the parser (PARSE-01, PARSE-05) that isn't mirrored here silently assigns wrong cells.
- **Causes:** HYG (corrupt cells → broken MLD).
- **Fix:** split the parser into stage A (NBG, flow graph, and `nbg_osm_ids.bin`) and stage B (EBG build using the cell mapping). Then either:
  - delete the sticher (stage B does the injection, as parser2 L404-458 already does), or
  - make the sticher read `nbg_osm_ids.bin` instead of re-parsing.

#### STITCH-02 · Cell id narrowed to `uint16_t` without a range check 🟢 `[ ]`
- **Where:** L143
- **Fix:** assert `cell < 65536`.

---

### `partitioning/OverlayGraphConstructor.cpp`

*Verified correct for current data.*

#### OVL-01 · Unsaturated distance add 🟠 `[ ]`
- **Where:** L115
- **Causes:** once INF weights exist (PARSE-05, PARSE-08, Phase 5), sums overflow into small numbers, producing fake cheap shortcuts.
- **Fix:** skip edges with INF weight; saturating add; never emit a shortcut with INF weight.

#### OVL-02 · Shortcuts stored as a sparse `(to, weight)` list per entry node 🟠 `[ ]`
- **Where:** L133-143, flattening in `main`
- **Causes:** P4. Different profiles produce different sparsity, so the overlay topology can't be shared.
- **Fix (Phase 5):** CRP layout. Per cell, store entry list + exit list (shared topology) and an `entry × exit` weight matrix per profile (INF = unreachable). The query reads `matrix[cell_offset + i*exits + j]`.

#### OVL-03 · Hard-coded paths; `unordered_map` per Dijkstra 🟢 `[ ]`
- **Where:** L82, `main`
- **Fix:** CLI arguments; a local-index array per cell instead of a hash map (faster customization, which matters when it runs once per profile).

---

### `partitioning/Dinic_Partitioner*.cpp`

Not audited in detail; no known defects. **Must be rerun** whenever the NBG changes (PARSE-01, PARSE-03, PARSE-05).

### `parsing/reverse_graph_generator.cpp`

Correct. **Must be rerun after every forward-graph change.** In Phase 5, extend it to reverse the topology once and permute each profile's weight array with the same index map.

---

### `update_weights/update.cpp` (+ binary `patch_u_turns`)

#### UPD-01 · Dangerous obsolete patcher; delete it 🔴 `[ ]`
- **Where:** L36 reads `cell_id` as 4 bytes (it is 2) and skips `is_reverse_geometry`, misaligning every record after the first. L81 treats *any* same-way transition (including going straight) as a U-turn. L83 does `weight - 120 + 5` on `uint32`, which underflows to ~4.29e9.
- **Status:** not applied to current data (audit: 0 underflowed edges).
- **Causes if ever run:** catastrophic P3 (straight roads become astronomically expensive; overflowing sums produce loops).
- **Fix:** delete `update.cpp` and `patch_u_turns`. U-turn policy now lives in PARSE-08.

---

## Map server (`map_server/`)

### `load_data/KDTreeHolder.cc` / `.hpp`

#### KD-01 · KD point and bearing ignore travel direction 🔴 `[ ]`
- **Where:** `KDTreeHolder::initialize` L30-49
- **Logic today:** point = `coordinates[offset]`, bearing = `offset → offset+1`, for *both* directed nodes of a two-way road, so they get the same point and the same bearing.
- **Causes:** P1/S2, P3/L1 (about 50% wrong-direction snaps on two-way roads → U-turn loops at the start and end of a route).
- **Fix (Phase 1 quick win):** for `is_reverse_geometry` nodes, use `coordinates[offset + size - 1]` and the bearing from the last point to the second-last. Replaced entirely by NEW-04 in Phase 3.

#### KD-02 · One indexed point per directed segment 🔴 `[ ]`
- **Where:** same loop
- **Causes:** P1/S3. On a 1–2 km highway segment, the indexed point can be far away, so side roads win.
- **Fix:** NEW-04 (segment index with projection). There's no good patch within the KD design.

#### KD-03 · No access filter on candidates 🟠 `[ ]`
- **Causes:** P1/S4 (car snaps to footway or service road).
- **Fix:** NEW-04 filters by `access_mask` (PARSE-05, PARSE-11). Interim: skip `road_type == 5` for car requests.

#### KD-04 · Dead duplicate `calculate_distance` 🟢 `[ ]`
- **Where:** L4-17 (also truncates to `uint32`)
- **Fix:** delete.

---

### `search/kd_tree.cc` / `kd_tree.h`

*Whole file retired in Phase 3 by NEW-04; the Phase 1 items keep it usable until then.*

#### KDQ-01 · Heading term is a placeholder 🔴 `[ ]`
- **Where:** `findBestNodeWithHeading` L159 (`road_bearing = 0.0`). `KDNode::bearing` (kd_tree.h) is never read.
- **Causes:** P1/S1. Heading has no effect; selection is pure nearest-point.
- **Fix (Phase 1):** use `node.bearing` (after KD-01). Ignore heading when compass is unknown (add a sentinel such as `compass < 0`) or when speed is below ~2–3 m/s.

#### KDQ-02 · Distances in raw degrees 🟠 `[ ]`
- **Where:** L49, L91, L165; radius L141-142
- **Causes:** P1/S5. Anisotropic distances (lon degrees are about 7% shorter at 22°N, about 17% at 34°N); the "20 m" radius isn't 20 m.
- **Fix:** scale Δlon by `cos(lat)` (equirectangular) and work in metres.

#### KDQ-03 · Fallback drops heading; arbitrary cost blend 🟠 `[ ]`
- **Where:** fallback L147-150; cost L169
- **Causes:** P1. When nothing is within 20 m (common, because points are sparse), it falls back to nearest-point with no heading.
- **Fix (Phase 1):** expand the radius progressively (20 → 50 → 100 m) while keeping the heading term. Weight heading by speed confidence.

#### KDQ-04 · Destination snapped with `findClosestNode` (single node) 🟢 `[ ]`
- **Where:** called from SearchService.cc L108
- **Fix:** Phase 3 returns destination candidates for the phantom router (SVC-01).

---

### `search/SearchService.cc` / `.h`

#### SVC-01 · Single-node origin/destination; no phantom nodes 🔴 `[ ]`
- **Where:** `normal_routing` L107-108
- **Causes:** P1/S6, P3/L1. The full first and last segments are always included, and the wrong direction can't be recovered by the router.
- **Fix (Phase 3):** fetch the top-k candidates for each end from NEW-04, with fraction along the segment, and pass candidate sets to the multi-source router (MLD-02).

#### SVC-02 · No profile parameter 🟠 `[ ]`
- **Where:** `normal_routing` signature L101
- **Fix (Phase 5):** thread `profile` through snapping (access filter) and routing (weight arrays).

#### SVC-03 · No-path returns an empty JSON object 🟢 `[ ]`
- **Where:** `getPathJson` L86-89
- **Fix:** return a proper error (HTTP 404/422 with a reason code: `NO_ROUTE`, `SNAP_FAILED`).

#### SVC-04 · Absolute include path 🟢 `[ ]`
- **Where:** SearchService.h L3
- **Fix:** relative include via CMake include dirs.

---

### `search/SearchController.cc` / `.h`

#### CTL-01 · CPU-heavy routing runs on the IO thread 🟠 `[ ]`
- **Where:** `getRoute` L16. `config.json` has no `threads_num`, so Drogon defaults to 1 IO thread.
- **Causes:** PERF. Each request blocks the event loop for the whole query, so one route at a time and nothing else is served meanwhile. Rerouting will amplify this.
- **Fix:** move routing to a worker pool (fixed size ≈ cores) and `co_await` the result. Set `threads_num` for IO threads.

#### CTL-02 · API lacks profile, accuracy, speed, heading accuracy; compass is mandatory 🟠 `[ ]`
- **Where:** route pattern in SearchController.h
- **Fix:** optional query params (`profile`, `accuracy`, `speed`, `heading_accuracy`); compass optional with an "unknown" default.

#### CTL-03 · No navigation-session or location-stream endpoints 🔴 `[ ]`
- **Causes:** P2. A backend HMM needs a stream of fixes plus per-trip state.
- **Fix (Phase 4):** NEW-07.

---

### `search/multi_level_dijkstra.cc` / `.h`

#### MLD-01 · ~750 MB allocated and zero-filled per query 🔴 `[ ]`
- **Where:** L24-29 (4 × 46.8M × 4 B)
- **Causes:** PERF. Hundreds of ms of pure memset per request; risk of running out of memory under concurrency.
- **Fix:** a thread-local search workspace allocated once per worker thread. Pair each slot with a generation stamp: a slot is valid only if `stamp[v] == current_query`, so reset is O(1). Store parents as `uint32` plus a 1-bit "came via overlay" flag (helps MLD-05).

#### MLD-02 · Single source and target 🔴 `[ ]`
- **Where:** signature L15, seeding L39-43
- **Causes:** P1/S6, P3/L1.
- **Fix (Phase 3):** multi-source / multi-target.
  - Forward seed per origin candidate = `wt × (1 − f)` + heading penalty.
  - Backward: the untravelled tail of each target segment (`wt × (1 − f)`) must be subtracted. Apply it as a correction when evaluating a meeting node, or bias all backward seeds by a common constant (weights are unsigned).
  - Same-segment case (origin and destination on the same directed node with `f_o < f_d`) is answered without search.

#### MLD-03 · Unsaturated `uint32` additions 🟠 `[ ]`
- **Where:** L108, L125, L170, L186 (and L82, L151 in meeting checks)
- **Fix:** saturating add; skip INF edges.

#### MLD-04 · Shortcut unpacking is unbounded, with a silent straight-line fallback 🟠 `[ ]`
- **Where:** `localized_shortcut_unpack`. Cell filter commented out at L263; fallback L283-290; `unordered_map` L241-242.
- **Causes:** PERF (one unpack can explore far outside the cell), visual glitches (straight-line jumps).
- **Fix:** restore the cell restriction using **`node_metadata[v].cell_id`**. *Not* `node_cell_mapping`, which is wrong (see LOAD-02); the commented line would read garbage if simply uncommented. Use the thread-local workspace (MLD-01). Turn the fallback into a logged error metric. In debug builds, assert the unpacked weight equals the shortcut weight.

#### MLD-05 · Unpack prefers a direct base edge even when the shortcut was used 🟢 `[ ]`
- **Where:** `unpack_abstract_path` L316-330
- **Causes:** geometry may not match the cost actually used (rare).
- **Fix:** use the "via overlay" parent flag from MLD-01 instead of guessing.

#### MLD-06 · Payload uses full first/last segment weight, distance, geometry 🟠 `[ ]`
- **Where:** `extract_navigation_payload` L421-432
- **Causes:** P1/S6. The route line starts behind the user and ends past the destination; ETA and distance are off.
- **Fix (Phase 3):** trim the first and last polylines at the projected points; pro-rate `wt` and `distance_meters` by fraction.

#### MLD-07 · Corridor bearings ignore travel direction 🔴 `[ ]`
- **Where:** L460-468
- **Causes:** P1/S7. The client-side Viterbi receives reversed bearings for half the corridor nodes.
- **Fix (Phase 1 quick win):** same direction-aware rule as KD-01 (later read `bearing_start` from metadata, PARSE-11).

#### MLD-08 · Dead code and dead structs 🟢 `[ ]`
- **Where:** commented block L481-708; `EBG_Metadata`, `RoutingData` in `.h` L7-41
- **Fix:** delete (git has history).

---

### `load_data/DataLoader.cc` / `.hpp`

#### LOAD-01 · No validation of files at load 🟠 `[ ]`
- **Where:** `load_all` L12-50
- **Logic today:** trusts every header. Reverse-graph node count unchecked; overlay edges file has no header; file sizes unchecked.
- **Causes:** HYG. Mixing files from different pipeline runs gives silent wrong routes or out-of-bounds reads.
- **Fix:** read `manifest.json` (NEW-03), verify magic, version, counts and expected file sizes for every file, and abort on mismatch.

#### LOAD-02 · `node_cell_mapping.bin` mapped with the wrong type and index space 🟠 `[ ]`
- **Where:** L25-26
- **Logic today:** mapped as `uint16_t[]` at offset 0. The file actually holds a `uint32` count header followed by a `uint32` per **NBG** node (that's how sticher.cpp L101-104 reads it). Indexing it by EBG id returns garbage.
- **Status:** unused today, so latent, but it's the obvious thing to use when fixing MLD-04.
- **Fix:** remove `node_cell_mapping` from `GraphDataView` and the loader; `node_metadata[].cell_id` is the source of truth.

---

### `load_data/GraphDataView.hpp`

#### VIEW-01 · On-disk layout must follow PARSE-11 / PARSE-10 🟠 `[ ]`
- **Where:** `PackedMetadata` L7-21; absolute include L3
- **Fix:**
  - Update `PackedMetadata` in lock-step with PARSE-11 and add `static_assert(sizeof(PackedMetadata) == N)`.
  - In Phase 5, replace `routing_edges` with topology plus per-profile weight pointers.
  - Use a relative include.

### `include/GraphStructures.hpp` and `map_server/include/GraphStructures.hpp`

#### HDR-01 · Two diverged copies of the binary-format header 🟠 `[ ]`
- **Logic today:** the root copy *defines* globals; the server copy declares them `extern`. The structs (`RoutingEdge`, `ShortcutEdge`, `Coordinate`) are on-disk formats shared by both sides. If they drift, the server misreads the files.
- **Fix:** one shared `include/DiskFormats.hpp` with only the on-disk structs and `static_assert`s on their sizes, used by every tool and the server. Tool-only globals (`nbgGraph`, `expandedNodes`, …) move into the tools that own them.

### `plugins/GraphDataPlugin.cc` / `.hpp`

#### PLG-01 · Owns `KDTreeHolder`; will own the new components 🟠 `[ ]`
- **Fix:**
  - Phase 3: replace `KDTreeHolder` with `SpatialIndex` (NEW-04).
  - Phase 4: own the `MapMatcher` config and `NavigationSessionStore` (NEW-06/07).
  - Load the manifest (LOAD-01).

### `main.cc` / `config.json`

#### CFG-01 · Trace logging in production; IO thread count unset 🟢 `[ ]`
- **Where:** main.cc L6 (`kTrace`); `config.json` has no `threads_num`
- **Fix:** log level from config (`kInfo` default); set `threads_num`; add a worker-pool size setting for CTL-01.

### Client app (not in this repo)

- Gets correct corridor bearings after MLD-07.
- After Phase 4, client-side Viterbi becomes display-only smoothing; the server becomes authoritative for matching and off-route decisions.

---

## New components to create

| ID | Component | Location (proposed) | Purpose | Problems |
|---|---|---|---|---|
| NEW-01 | Profile configs | `parsing/profiles/{car,motorcycle,foot}.json` + loader in parser | Highway → access/speed tables, one-way rules, turn-cost params, barrier rules, mode restrictions | P3, P4 |
| NEW-02 | Spatial index builder | `parsing/build_spatial_index.cpp` → `spatial_index.bin` | Static grid (~100–200 m cells) or packed R-tree over **geometry pairs**; each entry stores fwd EBG id, rev EBG id, position in polyline. Physical geometry indexed once. | P1, P2 |
| NEW-03 | Pipeline script + manifest | `build_pipeline.sh`, `manifest.json` | Runs parser A → Dinic → parser B → spatial index → customizer (per profile) → reverse, in order. Writes counts, units, versions, checksums. | HYG |
| NEW-04 | `SpatialIndex` | `map_server/search/SpatialIndex.{h,cc}` | Query (lat, lon, radius, k, heading?, profile) → candidates {ebg node, projected point, fraction in travel direction, perp. distance, bearing}. Replaces `kd_tree.*` and `KDTreeHolder.*`. | P1, P2 |
| NEW-05 | Phantom routing | inside `multi_level_dijkstra.*` (MLD-02/06) | Multi-source/target seeding, same-segment case, trimming | P1, P3 |
| NEW-06 | `MapMatcher` | `map_server/matching/MapMatcher.{h,cc}`, `MatchState` | HMM (Newson & Krumm): Gaussian emission on distance + heading term; exponential transition on \|GPS dist − route dist\|; bounded distance-metric Dijkstras on the base graph; fixed-lag online Viterbi; HMM breaks; off-network state | P2, P1 |
| NEW-07 | Navigation sessions | `map_server/navigation/{SessionStore,NavigationController}.{h,cc}` | `POST /api/nav/start`; WebSocket (or batched POST) location stream → matched position, progress, off-route flag, reroute. In-memory TTL store. | P2 |
| NEW-08 | Test harness | `tools/`, `traces/` | Recorded GPS traces (dual carriageways, flyovers, roundabouts); OSRM in Docker on the same pbf as an oracle for `/route` and `/match`; regression metrics; the baseline audit script | all |
| NEW-09 | Per-profile metrics | parser + customizer + server | Shared topology and partition; per-profile base weights and CRP overlay matrices | P4 |

---

## Problem → issue map

| Problem | Issues |
|---|---|
| **P1** wrong-road snapping | KD-01, KD-02, KD-03, KDQ-01, KDQ-02, KDQ-03, KDQ-04, SVC-01, MLD-02, MLD-06, MLD-07, PARSE-01, PARSE-05, PARSE-11, PARSE-12, NEW-02, NEW-04 → finally solved by NEW-06 |
| **P2** HMM in backend | CTL-03, NEW-04, NEW-06, NEW-07, PARSE-06, PARSE-11, PLG-01 |
| **P3** loops / straight vs. turns | PARSE-01, PARSE-02, PARSE-03, PARSE-04, PARSE-05, PARSE-06, PARSE-07, PARSE-08, PARSE-09, KD-01, KDQ-01, SVC-01, MLD-02, UPD-01 |
| **P4** vehicle profiles | PARSE-03, PARSE-05, PARSE-09, PARSE-10, OVL-02, SVC-02, CTL-02, NEW-01, NEW-09 |
| **PERF** | MLD-01, MLD-04, CTL-01, CFG-01, OVL-03 |
| **HYG** | UPD-01, LOAD-01, LOAD-02, HDR-01, STITCH-01, STITCH-02, PARSE-13, PARSE-15, MLD-08, SVC-03, SVC-04, VIEW-01 |

---

## Work order

### Phase 0 — Measure before changing
- NEW-08: record traces; stand up the OSRM oracle on `india-260724.osm.pbf`; save the baseline audit numbers (above) and a set of golden origin/destination pairs, including the loop cases you've seen.

### Phase 1 — Quick wins, no data rebuild
- UPD-01, KD-01, KDQ-01, KDQ-02, KDQ-03, KD-03 (interim), MLD-07, LOAD-02, MLD-08, KD-04
- *Expected:* most start/end U-turn loops disappear; client Viterbi gets correct bearings.

### Phase 2 — Parser and profile overhaul, full rebuild
- HDR-01 first (shared on-disk header), then NEW-01, PARSE-01 … PARSE-11, PARSE-14, PARSE-15, STITCH-01, STITCH-02, OVL-01, VIEW-01, NEW-03, LOAD-01
- Then rerun everything: Dinic → customizer → reverse. Re-run the audit and compare against the baseline.

### Phase 3 — Segment index, phantom routing, performance
- NEW-02, NEW-04, PLG-01 (part), SVC-01, MLD-02, MLD-06, MLD-01, MLD-03, MLD-04, MLD-05, CTL-01, CFG-01, SVC-03, SVC-04, KDQ-04
- Retire `kd_tree.*` and `KDTreeHolder.*`.

### Phase 4 — Live map matching
- NEW-06, NEW-07, CTL-03, PLG-01 (rest); off-route → reroute from the matched candidate.

### Phase 5 — Multi-profile
- PARSE-10 (split), OVL-02, NEW-09, SVC-02, CTL-02; add motorcycle and foot profiles.

---

## Rebuild dependency cheat-sheet

| You changed… | Must rerun |
|---|---|
| Way filtering, one-way, access (PARSE-01/03/05) | parser A → **Dinic** → parser B → spatial index → customizer → reverse |
| Speeds, turn costs, units (PARSE-04/06/07/08) | parser B → customizer → reverse *(partition can be reused)* |
| Metadata layout (PARSE-11) | parser B + every reader (sticher, customizer, server `PackedMetadata`) |
| Profile weights only (Phase 5) | customizer for that profile → reverse for that profile |
| Server-only issues | rebuild `map_server` only |
