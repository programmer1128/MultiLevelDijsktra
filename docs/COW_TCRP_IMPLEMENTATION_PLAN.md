# CoW-TCRP: implementation and validation plan

Date: 2026-10-23  
Status: design baseline for discussion; no engine changes implemented  
Repository reviewed: `f17925c`  
Authoring model: owner implements one understood component at a time

## 1. Decision and scope

Build one new production design: a compact, turn-aware directed road graph, metric-independent nested partitions, CRP overlay matrices, and bidirectional multilevel Dijkstra. Publish immutable, versioned cost snapshots using copy-on-write cell matrices. Retain Inertial Flow plus Dinic as the initial partitioner, subject to measured partition quality. Use temporary arc-based expansion inside customization when it outperforms compact turn-aware search.

The project name is **CoW-TCRP: Copy-on-Write Tiled Customizable Route Planning**. It describes a synthesis of known methods, not an established new algorithm or a proven performance improvement over CRP or Valhalla. The primary algorithmic reference remains the supplied CRP paper [R1].

This plan replaces the implementation order in `ENGINE_FIX_PLAN.md` for the new build while preserving its regression requirements. The existing committed engine remains a baseline, not a second production engine to maintain. Do not build an old-format compatibility exporter unless separately requested.

Scope: Overture acquisition; canonical graph and disk contracts; partitioning; customization; exact static-snapshot CRP queries; snapping; full route reconstruction and guidance; live sessions and HMM; traffic integration; later prediction and additional services; measured deployment capacity.

The target is 10,000 active navigation sessions on the specified four-core ARM64, 24 GB machine. This is a capacity target, not an achieved result. Server GPS frequency, fallback matching rate, reroute frequency, distinct metrics, route length, geometry output and traffic update cadence must be stated with every benchmark. Verify actual cloud entitlement and storage separately; use provisioned hardware rather than a free-tier product label as the benchmark definition.

### Non-negotiable invariants

- Every returned path is legal for its declared vehicle profile, restrictions and supported temporal semantics.
- Search, base costs, shortcuts and unpacking use the same topology/profile/traffic snapshot.
- No missing route is replaced by a straight line. No unsupported condition is silently ignored.
- Road geometry orientation, physical connectivity, travel permission and route preference are separate facts.
- Optimize after a reference result exists. An optimization must preserve the declared result or be explicitly exposed as an approximation.
- Full origin-to-destination geometry remains available. Progressive delivery is optional and must not change the route silently.

## 2. Review of the supplied hybrid proposals

The three pasted proposals are design inputs, not implementation instructions. Their useful mechanisms are retained below; numeric predictions and exactness claims require independent evidence.

### 2.1 Adopt as architectural choices

- Compact global intersections and directed arcs, shared geometry, turn-type tables and sparse exceptions.
- Metric-independent multilevel separator topology and per-metric entry-to-exit matrices.
- Temporary local edge expansion during customization, without a global materialized EBG.
- Copy-on-write immutable metric versions and shared identical cell matrices.
- Incremental bottom-up customization over dirty cells and their ancestor union.
- Mapped storage organized for partition locality, with an independent geographic spatial directory.
- Multiple candidate map matching, partial-edge positions, bounded multi-target transition searches and explicit ambiguity handling.
- Shortcut unpacking caches with topology, metric and snapshot identity in their keys.

### 2.2 Correct before implementation

**Historical timings are measurements, not guarantees.** The paper's roughly 1.6-1.9 ms searches, matrix sizes, cache hit rates and microinstruction costs describe its datasets, machine and methodology. They do not guarantee sub-2 ms full routes on Ampere, nor 150 ms GNN/customization, 0.12 ms matching or a 6 GB India service. Geometry, names, indexes, reverse adjacency, sessions and runtime overhead must be counted.

**CRP already has compact turns.** Sections 4.1-4.3 describe the compact graph, shared turn types and arrival-edge state. Section 5.2 describes temporary arc-based cell extraction. These benefits do not require replacing multilevel Dijkstra with Valhalla's search.

**Runtime costing must match overlays.** Applying a user preference only inside source/target cells, while using a different archetype elsewhere, changes the objective by geography. Selecting the nearest archetype is an approximation. It is not exact personalized routing.

**Do not blend shortcut weights.** In general, `min_P(alpha*T(P)+beta*S(P))` differs from `alpha*min_P T(P)+beta*min_P S(P)`. If two paths have (time, discomfort) values (10,100) and (12,0), equal weights give a true minimum of 12; blending independent minima gives 10, which is not a realizable path cost. Every supported combination gets its own customization or an exact base-search fallback.

**Dirty updates propagate upward.** Deduplicate dirty leaf and ancestor cells, but a small leaf update may reach a large top-level matrix. Update work depends on boundary counts, state expansion, profile count and changed regions; it is not simply proportional to changed road count.

**NEON is four 32-bit lanes per 128-bit register.** Sixteen sources need multiple registers and additional state. Overflow-safe addition, reachability masks, memory traffic and active-source handling are mandatory. Benchmark batches 1, 4, 8 and 16 against scalar Dijkstra/Bellman-Ford. No fixed 32 KB local-buffer assumption is valid for every cell.

**mmap does not make bytes disappear.** Resident mapped pages consume physical RAM. Page cache for the same mapped file is not an additional independent copy. Millions of small per-cell files or mappings can add padding, VM areas, descriptors and fault overhead. Pack many cells into a few sectioned files and compare page alignment policies.

**Partition cells are not spatial tiles.** Natural-cut cells can have irregular or overlapping bounding boxes and disconnected pieces. Geographic queries need a spatial directory that can return multiple cells. Partition-aligned storage is a locality option; strict one-cell-per-page or geographic containment is unnecessary.

**No universal 64-bit edge descriptor.** Length, endpoints, geometry offsets, flags, rules, source mapping and restrictions cannot all be promised in eight bytes. Use measured fixed-width hot arrays plus cold sidecars. An 8-bit local turn order needs an overflow path for high-degree intersections.

**Matrix sharing is conditional.** The proposed 50-80% sharing is unknown. Upper-level cells often contain relevant roads even when most leaves do not. Compare matrix contents and semantics before sharing; account for pointer tables, allocation granularity and retained versions. CoW matrices are immutable application objects, not writes into a shared file mapping.

**RCU publication is not per-cell pointer mutation.** Publish one complete root descriptor after all dependent matrices are ready. Individual swaps expose a mixture of versions. Reader lifetime and memory reclamation need a concrete mechanism; use a simple tested reference-counted root first if appropriate.

**CPU pinning and lock-free caches are experiments.** One GNN/customizer core and three query cores is not an established optimum. A single-socket VM does not imply a global cache fits or runs at L3 speed. Start with bounded executors and a byte-budgeted sharded cache; measure contention before adopting affinity or lock-free algorithms.

**A scalar snapshot is not time-dependent routing.** Switching metrics when accumulated cost exceeds 45 minutes can require extra state; preference cost may not even be elapsed time. Time-dependent shortcuts require travel-time functions, FIFO checks and a suitable query. Treat simple horizon switching as a separately validated approximation, not a pointer-swap implementation of exact arrival-time prediction.

### 2.3 Correct the HMM proposals

- Never lock the vehicle onto its planned corridor. Keep competing carriageway, flyover and service-road candidates when observations permit them.
- A fixed radius of 35 m and `k <= 6` cannot guarantee all candidates. Adapt to accuracy, gaps and local density; expose truncation and use a bounded recovery policy.
- Viterbi can be exact for the supplied finite trellis while the candidate generator, transition cutoff or fixed-lag commitment still approximates a fuller model. Avoid claims of 100% real-world matching accuracy.
- In a distance transition model, the units are metres. Never add turn delays in seconds. Turn legality still applies; a physically legal turn contributes zero geometric distance unless separately modeled geometry adds distance.
- On an atomic directed edge with nonnegative distance, increasing offsets admit a direct distance shortcut. Adjacent atomic edges also admit a direct distance shortcut when the complete restriction context permits it. A forbidden direct turn does not imply the target is unreachable: a legal detour can exist.
- A geographically close pair can require a long legal detour. Queries may leave and re-enter the same partition cell. Fixed cell clipping is not globally exact.
- Dijkstra stopped at a distance bound gives exact answers for targets settled within that bound. Others are `unresolved/beyond bound`, not proven globally unreachable. Expand or use a defined truncated transition model. `max(150m, 3*GPS displacement)` is a heuristic to test, not a theorem.
- A distance transition needs a distance-customized overlay. Fastest or scenic matrices cannot answer shortest road distance, and cross-cell access generally needs a search plus endpoint costs, not one O(1) matrix lookup.
- Matching workspace is indexed by arrival-edge/restriction states; 256 intersections can require many more than 256 labels.
- Heading must reflect motion and confidence. Speed, traffic, altitude and grade are optional calibrated features. Grade and usable altitude are not guaranteed in Overture; stopped vehicles and stale traffic must not erase correct hypotheses. Parallel decks can remain observationally ambiguous.
- Turn stalling is metric- and rule-dependent. Prove its assumptions before use; context-dependent rules may invalidate a shared simple stalling table.

## 3. Target architecture and component contracts

```text
Pinned Overture Parquet + catalogs + extraction manifest
  -> Overture adapter + semantic normalization + dependency checks
  -> canonical physical topology + directed arcs + rule store
  -> Inertial Flow/Dinic nested hierarchy + local numbering
  -> packed mapped graph sections + spatial index + overlay topology
  -> profile/traffic customization -> immutable MetricSnapshot
  -> candidate projection -> bidirectional turn-aware MLD
  -> exact shortcut unpacking -> trimmed path -> guidance/API

GPS stream -> session -> candidate trellis -> transition-distance engine
           -> online HMM -> progress / confidence / reroute decision

Traffic feed -> road-ID mapping -> validated observations -> optional GNN
             -> dirty metrics/cells -> customization -> atomic publication

Places/addresses -> separate search indexes -> routable entrance candidates
                -> route/matrix services for travel-time ranking
```

### Canonical graph

Physical nodes represent Overture connectors and any deliberately introduced attribute-boundary nodes. Never connect roads merely because their coordinates cross. Physical sections retain source segment ID, source version and `[start_lr,end_lr]`. Directed arc identities are dense, snapshot-local IDs with an explicit opposite arc where present, geometry direction, length, access/rule references and source mapping.

Store geometry once in a shared pool with cumulative distances and travel-direction tangents. Source Overture segments can contain interior connectors; all such decision points must be respected. Attribute-only boundaries can remain interval data. Compile splits where partial access or constant-cost runtime assumptions require them. Never add artificial turn delay at an introduced pass-through node.

Keep physical identity separate from restriction-state identity. Complex restriction expansion must map back to the same physical road for display, matching and traffic updates; traffic must not be counted twice just because routing states were duplicated.

### Search and cost interfaces

Ordinary base state is `(incoming directed arc)` or equivalently `(intersection, entry order)`. Complex restrictions extend it with sufficient history. Define `Allowed`, `TraversalCost`, `TransitionCost` and `PartialTraversalCost` for a named profile and snapshot. Return generalized objective cost separately from elapsed time and distance.

For static snapshots, all costs are nonnegative; unreachable is INF. Start with explicit uint64 accumulated distances/costs and checked arithmetic. Benchmark uint32 matrix storage only after proving units and maximum finite costs fit, with a reserved INF and saturating operations. Choose documented units once; prefer milliseconds and millimetres/centimetres where useful, with adequate width. Do not copy the old plan's decisecond choice without checking error requirements.

Do not prorate total travel time by geometry fraction when speed varies along an edge. Integrate the retained subranges. A source offset uses remaining directed-edge cost; a destination offset uses prefix cost. Derive forward/reverse accounting from this convention and verify all same-edge cases.

### Overlay contract

Each nested cell defines ordered boundary entry and exit states. Turn context at the boundary must be sufficient to preserve legal paths. `matrix[cell,i,j]` is the exact minimum in-cell objective cost from that entry to that exit under one snapshot. Unreachable pairs retain INF in fixed shared topology. Crossing-edge accounting must be specified to prevent double charging road or turn costs.

Store the hierarchy, parent/child relationships, all level assignments, boundary mappings and local state IDs. Pack partition vectors only after the level/branch bit budget is known; handle equal vectors and zero XOR explicitly. Do not assume CLZ plus lookup takes a fixed cycle count.

### Snapshot and cache identity

`MetricSnapshot = {topology_id, profile_id, traffic_id, rule_epoch, base_metric_root, matrix_root}`. Derived matrices, stalling tables and unpacking cache entries carry compatible identity. A query captures a root once and holds it through geometry/guidance extraction. Publish a completed root atomically. Bound old-version lifetime, CoW allocation and cache residency; coalesce pending traffic updates instead of building unlimited versions.

Matrix sharing uses a content hash plus equality check, dimensions and semantics. Deduplication is optional for correctness. Cache keys include cell, level, boundary pair, topology and metric/version. A bounded sharded LRU is sufficient initially. Cached paths are checked against version and closure policy.

## 4. End-to-end phases and acceptance gates

All phases below build the same target engine. Each starts on small hand-verifiable fixtures, then a city, then a larger region. India-wide processing follows passed correctness and memory gates. Owner-written code is the default; the assistant explains, reviews and helps refine when requested.

### Phase 0 - Baselines, contracts and evidence

Deliver: architecture decisions, fixture format, stable units/ID namespaces, build manifest schema and a runnable reference shortest-path test harness. Record baseline routes and actual graph counts from the old engine without changing it.

- Preserve traces for parallel carriageways, flyovers, service roads, roundabouts, legitimate U-turns, loops, toll alternatives and disconnected roads. Keep ground-truth uncertainty explicit.
- Create tiny directed fixtures with known legal paths, via restrictions and fractional endpoints. A simple expanded-state Dijkstra is a test reference, not a second production engine.
- Compare with OSRM/Valhalla only when data and policy are comparable. They are external comparators, not geographic truth or an oracle for Overture-only changes.
- Define the versioned disk contract before writing runtime files: magic, schema, endian convention, units, section offsets/alignment, counts, build IDs, topology hash and checksums. No native `size_t` or unaligned struct reinterpretation contract.

Exit: fixture expected results reviewed by the owner; decisions and benchmark recording format committed. Learning task: explain why one distance label per intersection loses turn context.

### Phase 1 - Reliable Overture acquisition and inventory

Deliver: immutable raw snapshot, catalogs, schemas, counts, extraction manifest, dependency report and restartable extraction status.

- Pin one resolved release and its schema. Preserve all columns/nested data and complete geometries for transportation segments/connectors in a declared acquisition region. Keep raw rail/water/path records even when excluded from a car graph.
- Preserve segment and connector versions, sources/licences, names, prohibited transitions, scoped access/speeds/flags and levels. Transportation does not include the places/address datasets; acquire those separately later.
- Validate nonempty unique IDs, connector references, interval ranges, geometry type and file readability. Account for restriction/destination references outside the bbox; fetch required dependencies or mark boundary completeness and restrict the service envelope accordingly.
- Repair the notebook's memory behavior: project connector columns first, use SELECT-list UNNEST or a flattened streaming stage rather than correlating whole segment rows, then disk-backed/partitioned deduplication and joins. Peak memory can arise from the UNNEST plan as well as DISTINCT. Inspect EXPLAIN and profile; do not assert one operator is the sole cause.
- Resume from completed `segments.parquet`; keep atomic file completion markers. Detect cgroup memory and disk availability before choosing memory limits. Host `sysconf` RAM may exceed container limits. Lower threads, explicit spill budgets, partitioned validation and checkpoints must allow progress without redownload.
- Treat null bbox as an exception to resolve using geometry, not a reason to include every null-bbox feature globally. Record false-positive bbox overlaps and neighbouring-country coverage.
- Produce counts by subtype/class, null/unknown fields, connector degree, source mix and scoped-rule coverage. No assumed India node count or claimed telemetry coverage.

Exit: successful manifest and zero unresolved required connector IDs; other unresolved dependencies explicitly resolved or excluded from the served scope. Raw disk use, peak RAM and resume behavior measured. Learning task: referential closure across a bounding-box cut.

### Phase 2 - Streaming adapter and segment atomization

Deliver: a source-independent physical graph with source lineage and an exception report. Proposed modules: `pre_processing/overture/` for ingestion tools and `graph_builder/overture/` for the C++ Arrow/Parquet adapter.

- Read batches/row groups and decode WKB explicitly; normalize lon/lat ordering. Use Overture's WGS84 geodetic linear-reference semantics [R4]. Accumulate distance in double and quantize once, preserving a stated tolerance.
- Index opaque source IDs to dense internal IDs using a memory-budgeted external sort/join or equivalent. Persist this mapping; never recreate IDs by reparsing the source differently.
- Split at every interior connector. Clip attribute intervals onto pieces and preserve the original linear range. Retain non-routing attribute changes as ranges; split for partial access/weight representation where necessary.
- Handle near-endpoint tolerances, duplicate coordinates, tiny/zero-length pieces and inconsistent connector positions through explicit rules and reports. Do not silently merge different connector IDs or infer flyover links from level or proximity.
- Introduce synthetic attribute nodes with distinct identity and zero artificial intersection penalty. Avoid densifying vertices merely for geometry shape.

Exit: tiny curved/multi-connector fixtures reconstruct their source geometry and distance within tolerance; no interior routing connector remains inside an atomic piece; streaming memory stays bounded. Learning task: prefix lengths plus ordered cut positions, followed by interval intersection.

### Phase 3 - Access, directed arcs and compact restrictions

Deliver: a multiprofile-capable directed graph, normalized rules, compact turn types and source-to-arc mapping. Begin validation with car routing; other profile semantics are built into the contracts now.

- Classify physical existence, operational state, mode eligibility and route preference separately. Unknown values are logged and conservatively handled; maximum raw coverage does not imply all features enter the car graph.
- Interpret direction- and mode-scoped Overture access. Distinguish geometric forward/backward from legal travel. Retain potentially available directions and conditionals rather than deleting them based on one current profile.
- Separate legal speed limit from estimated traversal speed. Use explicit units and profile defaults with provenance; absent information does not become a default fast shortcut. Keep barriers/signals only when available or from an explicitly joined source; report unavailable attributes.
- Store explicit physical opposites; parallel arcs with identical endpoints are not necessarily U-turn twins. Compute tangents in travel direction. Define Indian left-hand-traffic turn policy, pass-through behavior, signal delays and dead-end U-turn exceptions as profile rules, not arbitrary constants baked into topology.
- Resolve simple prohibited transitions and mode/temporal scopes. Compile via sequences into local state expansion, initially using a straightforward correct construction; preserve boundary history in overlays. Test overlapping sequences and start/end inside a restricted sequence. No silent via-way deferral in a production car graph.
- Deduplicate turn-type layouts when equal. Use wider local order IDs or an overflow representation when degree exceeds the compact format. Validate compaction against an expanded-state reference. Delay stalling until this reference passes.

Exit: legal direction, access and restriction tests pass; no fabricated connectivity; state expansion is measured. Unsupported conditional rules yield an explicit unsupported profile/time result or a documented conservative policy. Learning task: incoming arc plus restriction-history state.

### Phase 4 - Packed mapped graph and spatial candidates

Deliver: shared disk-format library, validated loader, geometry/name/rule sidecars and a segment-level spatial index.

- Use sectioned arrays for node offsets, directed arcs, incoming arc references, geometry, rules and names. Keep hot fields small and descriptive/provenance fields cold. Validate all ranges and multiplication overflow before mapping. Fixed disk layouts use explicit alignment, versioning and compile-time checks.
- Start with packed files that support cell-order remapping after partitioning. Batch multiple cells per mapping. Compare geometry-locality and page-fault behavior before enforcing page-aligned cell blocks.
- Index physical polyline pieces once, then emit legal directed candidates. Use a packed R-tree or geographic grid plus an index to irregular partition cells. Search all intersecting cells/segments; partition membership is not a nearest-road criterion.
- Return `{physical_arc, directed_arc, projected_position, directed_fraction, distance_m, tangent, access_status}`. Use metre-based distance, adaptive radii, optional motion heading and heading confidence. Never silently discard heading or switch to unconstrained endpoint-nearest fallback.
- Multiple candidates include both legal headings and plausible parallel roads. Respect destination access/entrances separately from heading. Empty network and no-candidate outcomes are explicit errors.

Exit: long-edge, reverse geometry, dual-carriageway, overpass and cross-cell projection fixtures pass. Loader rejects mixed builds, corrupt offsets and wrong units. Learning task: nearest point on a polyline versus nearest stored endpoint.

### Phase 5 - Inertial Flow/Dinic hierarchy

Deliver: a deterministic nested hierarchy, full cell tree, local node/state order and boundary maps. Retain the existing algorithm family; refactor its data interfaces for the canonical graph.

- Partition the unexpanded physical routing topology with documented symmetric cut capacities; count directed connectivity consistently. Weighting bidirectional roads by 2 and one-way roads by 1 is a starting option, not travel-time weighting or a claim of optimality.
- Preserve every parent/child split and requested query level. The current `recursiveBisection` stores only leaf IDs with a 5,000-node threshold; it cannot supply a full CRP hierarchy as-is.
- Guarantee split progress, nonempty children, balance, cell-size limits and handling of islands/disconnected components. Validate Dinic residual pairs, virtual terminals, per-cell isolation, numeric capacity ranges and deterministic concurrency. Existing code is not assumed fully audited.
- Measure boundary entry/exit counts, `sum(p_C*q_C)`, expanded state sizes, max matrices and expected workspace; cut count alone is insufficient. Reorder nodes/arcs by leaf cell with explicit forward/reverse remaps.
- Sweep leaf sizes (for example 256, 1,024 and 4,096), level counts and balance parameters on the same region. The proposed five level sizes and phantom level 16 are benchmark configurations, not fixed design constants. Compare against KaHiP only as an optional quality baseline if practical.

Exit: hierarchy consistency tests pass and matrix/workspace estimates fit measured resource budgets. Learning task: nested partition IDs and boundary state accounting.

### Phase 6 - Correct CRP customization and snapshots

Deliver: profile metric evaluator, metric-independent overlay topology, static metric matrices and snapshot publication.

- Build complete boundary-pair topology once, keeping INF for unreachable pairs. Customize leaf matrices with compact turn-aware Dijkstra first; customize ancestors using child overlays plus cut arcs. Verify every pair on fixtures against reference distances.
- Add temporary cell-local arc expansion and compare results bit-for-bit/tolerance-aware with the compact reference. Allocate by actual state/transition counts under a bounded arena; high-degree or complex cells can exceed L1 cache.
- Benchmark Dijkstra, scalar Bellman-Ford and batched NEON customization. Prove monotone nonnegative arithmetic and saturation; fallback to scalar for unsupported layout. Phantom levels and contraction microinstructions are optional later experiments; omit microinstructions initially.
- Publish coherent base and overlay roots. First establish immutable snapshots with safe reader lifetime; then add incremental dirty-cell/ancestor customization and matrix sharing. Compare an incremental rebuild with a complete customization after random updates.
- Maintain separate distance metrics for matching and profile-specific travel objective metrics. Store elapsed-time reconstruction information or recompute along the chosen route; generalized cost is not ETA.

Exit: all matrix pairs correct on small fixtures; incremental/full customization agree; publication/reclamation stress tests expose no mixed versions. Learning task: an entry-by-exit matrix and bottom-up recomputation.

### Phase 7 - Full route queries and exact reconstruction

Deliver: bidirectional turn-aware multilevel Dijkstra, candidate endpoints, full path, distance/ETA and structured errors.

- Preserve entry/exit state when switching between base and overlay. Meeting logic must include the necessary bridging turn/history; an intersection ID match alone is insufficient. Stopping rules use the declared nonnegative metric and safe arithmetic.
- Seed multiple directed candidates at fractional positions. Define snap penalties separately from route objective. Test source/target on the same arc in both offset orders, opposite headings, parallel arcs and all boundary cases.
- Reuse worker-local query buffers indexed by source/target local states and overlay IDs or a measured sparse equivalent. Reset touched slots/generation stamps safely, including wraparound. Avoid four whole-India arrays per request or per session.
- Store exact predecessor type and arc/shortcut identity; never guess from existence of a direct base edge. Recursively unpack in the correct cell and level using the pinned metric. A missing reconstruction is an error with diagnostics, never a straight-line fallback.
- Trim first/last geometries at projected positions. Integrate piecewise costs for partial traversals. Check directed geometry continuity, cost equality, legal transitions and destination reachability.
- Add version-keyed, byte-budgeted unpacking caches after correctness. Return full route geometry and maneuver data. Progressive geometry is an optional transport optimization with explicit semantics.

Exit: randomized MLD/reference comparisons pass, including restrictions and INF; unpacked objective equals query objective and full path is continuous. Learning task: one meeting turn and one shortcut unpacking proof.

### Phase 8 - Server, profiles and usable navigation output

Deliver: the new graph-backed `map_server`, bounded workers, profile API, observability and navigation instructions.

- Keep Drogon IO separate from CPU-heavy search/matching/customization. Use bounded queues, cancellation, deadlines and overload responses. Worker count is measured against CPU/RAM; requests do not allocate a new nationwide workspace.
- Pin snapshot identity through response construction; return metric/profile identity, distance, elapsed time, objective and geometry. Define `NO_ROUTE`, `SNAP_FAILED`, `INVALID_INPUT`, `UNSUPPORTED_PROFILE`, `STALE_SESSION` and resource-limit results.
- Request fields include profile/preferences, coordinates, optional motion heading, speed, accuracy, heading accuracy and departure-time semantics. Validate coordinate ranges, timestamps and schema. Use configurable paths/log levels and consistent CMake/shared headers.
- Initial exact profiles: fastest car, strict no-tolls car and distance. Add scenic only with a declared nonnegative score source and detour policy; distinguish strict avoidance from a soft penalty. Compose combinations before customization. Do not blend overlays or substitute the nearest archetype silently.
- Add shared-metric caching, single-flight customization and admission limits for uncommon combinations. If a metric is not ready, use a bounded exact base search, asynchronous preparation or an explicit unsupported/deferred response. Never quietly use wrong shortcut weights.
- Build guidance from the full path, junction topology, names, route numbers, roundabouts and available signposts. Keep maneuvers, ETA and user-preference penalties separate. Include turn direction and reversal regressions.

Exit: city-to-city routes work through the API with full geometry, legal guidance and declared profiles; parallel requests do not block IO. Learning task: immutable shared graph plus isolated mutable worker workspace.

### Phase 9 - Online HMM and navigation sessions

Deliver: fixed-lag HMM, transition-distance service, bounded sessions and reroute policy. Candidate generation reuses Phase 4; matching never assumes the planned route is the observation truth.

- Session state includes topology identity, route references, recent fixes/candidates, trellis scores/backpointers, confidence and TTL. Serialize updates per session, deduplicate sequence numbers and handle out-of-order fixes, reconnects and expiry. Set hard bounds on history, candidates and route storage.
- Use log probabilities. Start with distance emissions calibrated by reported accuracy and heading only when motion supports it. Keep multiple plausible branches. An explicit off-network/gap state prevents forced false matches.
- Compute legal candidate-to-candidate shortest distances in metres, including partial arcs. One search per predecessor candidate can settle all current targets. Reuse work across pairs and local neighborhoods. Distance routing includes legal turns/history without adding second-valued penalties.
- Direct same/adjacent-arc shortcuts are certified under atomicity and restriction context. Otherwise run bounded multi-target distance Dijkstra across cells as required. Expand unresolved bounds or model truncated support explicitly. Add distance CRP for long transitions only after baseline correctness.
- Fixed-lag commitment, radius and candidate caps are disclosed approximations relative to an unrestricted full trace. Compare to an offline reference using the same model and dataset. Do not promise Newson-Krumm equivalence after adding extra emissions or pruning [R6].
- Add optional speed/altitude/grade emissions only after calibration, missing-data handling and ablation studies. Never infer a flyover deck confidently from unavailable elevation or traffic means alone. Record ambiguity and wrong-deck duration.
- Reroute after persistent off-route evidence, a relevant closure or a material route-cost improvement, with hysteresis. Routine progress does not invoke a full route search. The planned route can guide candidate discovery or serve as a soft prior only if competitors remain observable.
- Evaluate reusing a tested matching library/core versus custom trellis logic; Valhalla Meili is coupled to Valhalla's graph and is not a drop-in reader for these files. Any adaptation must use the same canonical topology and restriction semantics.

Exit: labelled traces pass parallel-road, flyover, roundabout, stop/start, gap, tunnel, reversal and off-network tests. Measure transition-search work, commit latency, false reroutes and dropped candidates. Learning task: a two-road, three-observation Viterbi trellis.

### Phase 10 - Real traffic ingestion and incremental metrics

Deliver: provider adapter, mapping coverage report, freshness policy, closure handling and coherent metric updates.

- Establish that the selected feed supplies usable directional speeds/events and permitted identifiers/geometry. Match provider road references or OpenLR to canonical physical arcs with confidence, version and subrange mappings. Overture IDs are not assumed to match provider IDs.
- Preserve timestamps, observed versus predicted values, confidence, units and expiry. Reject impossible speeds; use base/historical fallback for missing or stale data. Restrictions/closures are distinct from congestion and take precedence according to policy.
- Batch changes by interval and deduplicate dirty leaves and all ancestors for each affected metric. Respect crossing-arc costs; never assume only one endpoint cell matters. Compare incremental results with full customization.
- Build a complete new snapshot with memory/backpressure limits and single publication. Old queries can finish on old snapshots. Revalidate returned paths against urgent current closures and reroute or hold the response when necessary. Active-route cell intersections trigger assessment, not automatic rerouting of every user in an ancestor cell.
- For improvements on roads outside the current route, route-cell invalidation alone cannot detect all newly better alternatives. Use periodic/budgeted reconsideration or a justified affected-query strategy; otherwise disclose delayed optimization. Closures on the route receive priority.

Exit: repeatable road slowdown/closure/reopening tests produce correct snapshot-specific paths and bounded update lag. Learning task: leaf-to-root dependency invalidation and atomic publication.

### Phase 11 - Traffic prediction and time-dependent extensions

Deliver: recorded traffic dataset, baseline predictors, evaluation pipeline and an optional GNN producer behind the existing metric interface.

- First benchmark persistence, historical averages and simple statistical predictors. A pretrained universal India model is not assumed. Train offline using time-separated splits, sufficient directional coverage and missing-data masks; report prediction and route/ETA error against baselines.
- GNN inference runs in regional/time batches, not per user. Measure model weights, input history, activations and ARM inference CPU separately from routing. Training stays off the small serving VM. Coalesce deltas and monitor prediction confidence/expiry.
- A scalar forecast snapshot is exact only for its frozen metric, not for every edge's future arrival time. Label its forecast horizon and semantics. Avoid arbitrary switches based on generalized path cost.
- A true departure-time engine requires nonnegative travel-time functions, FIFO validation and function-valued shortcuts, plus compatible conditional restrictions, query and unpacking. Preserve interfaces for this now; implement only after measuring its memory/customization costs. Standard static bidirectional stopping rules cannot simply be reused.
- Profile-dependent and time-dependent state semantics must be explicit; avoid claiming exact shortest physical time when optimizing a different objective or applying forecast approximations.

Exit: prediction improves held-out baselines and meets serving budgets; true time-dependent support is a separate signed-off capability, not inferred from periodic updates. Learning task: why the same road has different traversal time at different arrival times.

### Phase 12 - Additional services and deployment acceptance

Deliver: optional places/address search, route matrices, isochrones/alternatives and a load-tested deployment package.

- Nearby places use separately acquired places/address data and spatial/text indexes. Map entrances to routable directed positions. Use routing/matrix costs to rank travel time after geographic filtering; POI records are not road vertices.
- Add alternatives with explicit overlap/detour criteria, then matrices and isochrones using verified profile/snapshot semantics. Transit needs schedule/transfer state and is a separate expansion of scope, not an automatic consequence of road profiles.
- Package one immutable topology build with compatible metric versions. Startup validates manifests and fails closed on mixing. Atomic rollback, health checks, graceful draining, request limits, cancellation and topology/session version migration are required.
- Run the capacity and storm tests in Sections 7-8. Record failures and choose geographic scope, update frequency, matching location, profile limits or hardware scaling based on evidence.

Exit: a full route plus live navigation works on the target machine under the declared workload with no correctness regressions, no growing queues, no swap thrashing and documented latency/memory/update-lag results.

## 5. Repository change map

These are proposed ownership boundaries, not files already implemented. Keep experimental tools outside the production dependency graph.

| Existing area | Target responsibility |
|---|---|
| `pre_processing/preprocess.ipynb` | Restartable acquisition/inventory; extract shared acquisition logic into a small script after review |
| `parsing/parser2.cpp` | Historical baseline; replace source handling with streaming Overture adapter and canonical builder |
| `include/GraphStructures.hpp`, server copy | Replace disk contracts with one `include/DiskFormats.hpp`; separate in-memory models from disk records |
| New `graph_builder/` | Atomization, semantic rules, compact turns, restriction compilation, source-ID mappings |
| `partitioning/Inertial_Flow.cpp`, `Dinic_Partitioner*.cpp` | Select one maintained partitioner implementation; adapt inputs, preserve hierarchy and local numbering |
| `partitioning/sticher.cpp` | Retire source replay; graph IDs and partition mappings come from persisted build artifacts |
| `partitioning/OverlayGraphConstructor.cpp` | Rewrite for compact turn-aware cells, fixed matrix topology and all query levels |
| New `customization/` | Reference/local expansion/SIMD kernels, profile metrics, incremental updates, matrix pool and snapshots |
| `parsing/reverse_graph_generator.cpp` | Replace with canonical incoming-edge references/shared metric identity; no unchecked old-format reversal |
| `update_weights/update.cpp`, `patch_u_turns` | Disable from build/pipeline, then retire through an explicit implementation change; never run on new artifacts |
| `map_server/load_data/` | Shared-format loader, mapped sections, bounds checks and snapshot registry |
| `map_server/search/kd_tree.*`, `KDTreeHolder.*` | Replace with physical-segment projection and directed candidate service |
| `map_server/search/multi_level_dijkstra.*` | Implement compact-state multilevel query, reusable workspace and exact unpacking |
| `SearchService.*`, `SearchController.*` | Candidate endpoints, bounded workers, profiles, structured result/error contract |
| `GraphDataPlugin.*`, `main.cc`, configs | Resource ownership, safe publication, configurable paths/thread counts and production logging |
| New `map_server/matching/`, `navigation/` | Trellis, transition distances, session lifecycle, confidence/progress and reroute policy |
| New `traffic/`, `prediction/` | Provider mapping, observations, forecast artifacts and freshness; no direct graph-file mutation |
| New `tests/fixtures/`, `tools/benchmarks/` | Reference comparisons, regression traces, resource reports and workload replay |

Names can be adjusted before implementation. Responsibilities and contracts matter more than directory spelling. The current clean repository state was observed at planning time; preserve any later owner edits.

## 6. Traceability to ENGINE_FIX_PLAN.md

Every legacy issue is carried into a replacement, regression or explicit retirement. An old checkbox is not considered fixed until its new acceptance test passes. The old plan's solution details are reference material: do not blindly port OSM assumptions into Overture.

| Legacy IDs | New treatment and gate |
|---|---|
| PARSE-01, PARSE-02 | Phases 2-3: subtype/access/operational classification; no artificial delay at attribute or non-car continuations |
| PARSE-03 | Phase 3: directional/mode scopes; forward/reverse roundabout fixtures; OSM tag parsing itself retires |
| PARSE-04 | Phase 3: explicit legal-speed units versus expected speed, conservative defaults and provenance |
| PARSE-05 | Phases 1/3: normalized access and actual available barrier/signal data; unknown availability reported |
| PARSE-06, PARSE-07 | Phases 0/2/3: precise cumulative distance, one quantization, declared units and overflow-safe costs |
| PARSE-08 | Phase 3: oriented tangents, physical opposite identity, legal U-turn policy and realistic pass-through costs |
| PARSE-09 | Phase 3: indexed restrictions, modes/conditions and complete via-sequence support; no silent dropping |
| PARSE-10 | Phases 3/6: topology/metric split and safe arithmetic; compact graph replaces weighted global EBG |
| PARSE-11 | Phases 0/4: versioned shared disk sections, opposite IDs, access and direction fields |
| PARSE-12, PARSE-14, PARSE-15 | Phases 2/4/8: physical segment index, names dictionary, CLI/config paths |
| PARSE-13 | Phases 0/5: graph fingerprints and namespace-safe hierarchy artifact compatibility |
| STITCH-01, STITCH-02 | Phases 2/5: persisted mappings; retire reparsing; validated cell-ID widths |
| OVL-01, OVL-02, OVL-03 | Phases 5/6: safe INF, fixed matrices, local dense workspaces and CLI configuration |
| UPD-01 | Phase 0/implementation cleanup: obsolete patcher excluded; new metric publisher is sole update path |
| KD-01, KD-02, KD-03, KD-04 | Phase 4: full geometry projection, explicit heading/access and single maintained geodesy implementation |
| KDQ-01, KDQ-02, KDQ-03, KDQ-04 | Phases 4/7: real tangents, metre units, adaptive candidate search and multiple destination candidates |
| SVC-01, SVC-02, SVC-03, SVC-04 | Phases 7/8: fractional candidate endpoints, profiles, error contracts and portable includes |
| CTL-01, CTL-02, CTL-03 | Phases 8/9: bounded CPU workers, sensor metadata and session/stream APIs |
| MLD-01, MLD-03 | Phases 6/7: compact reused worker workspace, generation lifecycle and checked arithmetic |
| MLD-02, MLD-06 | Phase 7: derived endpoint accounting and interval integration; do not copy old backward-seed formulas unverified |
| MLD-04, MLD-05 | Phase 7: exact arc identity, correct cell/level unpacking and explicit reconstruction errors |
| MLD-07, MLD-08 | Phases 4/7/8: direction-correct corridor/guidance and retirement of dead old structs/code |
| LOAD-01, LOAD-02, VIEW-01, HDR-01 | Phases 0/4: one disk schema, validated sections, no wrong-index-space node-cell mapping |
| PLG-01, CFG-01 | Phase 8: explicit component lifetime, manifests, production logging and separate IO/workers |
| NEW-01, NEW-09 | Phases 3/6/8: profile semantics and shared-topology compatible metrics |
| NEW-02, NEW-04 | Phase 4: built spatial index and candidate projection service |
| NEW-03 | Phases 0-6: staged dependency-aware build manifests and rebuild fingerprints |
| NEW-05 | Phase 7: virtual/fractional endpoint semantics without mutating shared topology |
| NEW-06, NEW-07 | Phase 9: calibrated HMM and bounded sessions |
| NEW-08 | Phases 0/12: reference fixtures, labelled traces, comparators and repeatable load replay |

### Qualifications to legacy conclusions

- Treat the audit's counts as the old OSM build's historical measurements, not counts for the new graph.
- KD heading `road_bearing=0` makes the heading score constant across candidates; it does not prove a northward preference. The stated roughly 50% wrong-direction rate is unverified.
- Current intra-cell overlay consistency and reverse-graph checks do not prove full multilevel compact-state correctness. The new algorithm needs fresh meeting, boundary and stopping proofs/tests.
- OSM ways tagged only with lifecycle prefixes were not accepted by an exact `highway` predicate. Likewise node-only tags do not necessarily enter a way handler. Avoid overclaiming the old parser's actual inputs.
- A fixed speed fraction for link roads, a blanket U-turn ban or a single construction exclusion is a policy choice requiring evidence and exceptions, not a universal law.
- The old patcher was not applied according to the recorded audit; do not attribute existing loops to its unsigned underflow.
- Old generations stamps alone still retain large whole-graph arrays per worker. The new workspace must be sized from actual local/overlay states.

## 7. Capacity model and hardware experiments

### Workload definition

Ten thousand sessions at a two-second GPS cadence produce 5,000 fixes/s. With six candidates at both ends, a naive trellis considers up to 36 pairs per fix, or 180,000 pair transitions/s. Multi-target searches and certified direct cases reduce work, but the reduction is measured, not assumed to be 90% or 95%.

At one full reroute per user per minute, the average is about 167 routes/s. Bursts can be much larger. Distinguish websocket connections, submitted fixes, transition searches, full routes, guidance output, customized metrics and model inference batches in reports.

CPU demand in core-seconds per second is approximately:

```text
fixes_per_second * measured_matching_cpu_seconds_per_fix
+ routes_per_second * measured_full_route_cpu_seconds
+ customization_cpu_seconds_per_second
+ inference_cpu_seconds_per_second
+ API/session/serialization/search overhead
```

For illustration only: 5,000 fixes/s at 0.1 ms CPU/fix need 0.5 core, while 1 ms needs 5 cores. A 0.12 ms/fix result needs 0.6 core, not automatically 35% of a core. Use CPU time as well as wall latency and include queued work. At three routing cores, 1,800 routes averaging 1.85 ms CPU require at least 1.11 seconds of ideal compute; real elapsed time also includes output, matching, cache misses, contention and queues.

### Memory accounting

Measure these separately: canonical node/arc arrays; geometry and names; restriction and source sidecars; incoming adjacency; spatial index; all-level boundary topology; matrices by profile; directory/allocator overhead; current plus retained old snapshots; unpacking caches; per-worker search/customization/matching arenas; session trellises and route references; traffic input history; GNN weights/activations; server/runtime/OS overhead.

Matrix payload starts with `cost_width * sum_cells(entry_count * exit_count)` per distinct full metric, before deduplication and version retention. Add profile-specific rule/metric data and matrix directories. On 24 GB, set an initial measured-service release ceiling of 18 GiB total resident usage to leave headroom; this is an acceptance budget, not an estimate. Adjust only with actual target-machine data. mmap residency and page-cache accounting must not be double-counted or omitted. Track disk and mapping count too.

Session memory is not guaranteed to be 4 KB. Full route arrays can dominate. Store byte-bounded route objects with shared references where identical; retain limited trellis history and explicit per-session limits. Do not allocate global search arrays per user.

### Optimization ladder

1. Correct scalar single-thread reference and all-level customization.
2. Local IDs, reusable arenas, bounded queues, packed graph storage and coarse geographic index.
3. Incremental customization, versioned caches and matrix sharing.
4. Temporary local expansion and scalar kernel comparison.
5. ARM NEON batch sizes, optional phantom levels and memory-prefetch experiments.
6. Executor allocation, affinity and contention changes based on end-to-end measurements.

Measure warm and cold pages, regional and nationwide users, high-degree cells, city and long-distance routes, cache-hit distribution, retained snapshots and model reloads. A four-core VM must share physical CPU and memory bandwidth among all stages. GNN inference is scheduled and rate-limited; async syntax does not remove its CPU cost.

If the target fails: reduce avoidable repeated work, adjust update cadence or geographic scope, move routine client progress locally, limit expensive unique profiles, or increase serving resources. Preserve model correctness; do not hide overload by forcing corridor matches or returning stale-metric paths as current.

## 8. Validation matrix and storm drill

| Test | Required result |
|---|---|
| Two opposite carriageways, same shared geometry | Direction-aware candidates; no forced wrong-direction source |
| Flyover/service road with near-identical XY/heading | Both hypotheses survive while evidence is ambiguous; no invented connector |
| Illegal direct turn with a legal loop | Legal detour found; matching direct-turn rejection is not global unreachable |
| Via restriction across a cell cut | Boundary state preserves history; MLD equals expanded reference |
| Source/target on one directed arc | Correct offset-order cases, partial geometry and interval travel time |
| Access changes midway, speed varies midway | Correct legal splits/range rules, no scalar-fraction timing error |
| Toll-only connection | Strict no-toll returns no route; soft avoidance remains a different named policy |
| Fastest versus scenic paths with opposite tradeoffs | Correctly customized objective; no blended impossible shortcut |
| INF/near-limit accumulation | No wraparound or fabricated cheap route |
| Missing file, mixed topology, corrupt offset | Loader rejects before serving |
| Long GPS gap/cross-cell detour | Expanded transition search or explicit uncertainty; no false finite exactness claim |
| Interrupted download/customization | Existing completed artifacts remain usable; no false success marker |
| Query while metric root changes | One coherent snapshot throughout search and unpacking |
| Slow reader across many traffic updates | Bounded old-version retention and backpressure; no premature reclamation |
| Cold graph pages and nationwide load | Page faults, latency and resident working set measured |

### Reproducible flash-storm workload

Use the proposed 10,000 sessions, two-second fixes, 120 changed physical roads across eight leaf cells and 3,000 reroute requests over two seconds as a workload fixture. Include 40% exact no-toll/scenic combinations that have been explicitly customized. A changed physical road may update several directed/restriction-state arcs.

1. Replay the same labelled trajectories and request timings for every design variant. Include flyover exits and users on unaffected roads in the same ancestor cell.
2. Record traffic observation time, mapping completion, each profile's customization start/end, coherent publication and closure revalidation time. Derive ancestors from the actual hierarchy, not a predetermined count of 19 cells.
3. Distinguish route invalidation from optional optimization. Prioritize imminent closures and confirmed deviations. Use a leaf/edge-to-route subscription index or measured cell-list intersection. A fixed 64-bit mask and O(1) check do not represent arbitrarily many dirty cells/routes without a defined mapping.
4. Do not assume 7,000 sessions need no work, 1,200 can be skipped, or 1,800 are the only searches. Report actual categories, queue depth and policy-triggered route counts. An unaffected route may gain a better alternative after a cost decrease elsewhere.
5. Measure full-response p50/p95/p99 latency, max wait, output bytes, route legality, snapshot age, match error, wrong-deck duration, CPU, resident RAM, page faults and matrix/cache retention.
6. Proposed initial test objectives: p95 ordinary full-route latency <= 200 ms under the declared steady workload; publish a localized traffic batch within 2 seconds; drain the 3,000-request storm within 5 seconds; no unbounded queue or stale closed-road result. These are engineering targets to approve after first ARM measurements, not achieved promises. Track geometry size and cancellation separately.

Mathematical acceptance precedes performance: exact profile-compatible shortcuts, legal reconstructed paths, coherent snapshots and reference agreement. Statistical matching acceptance is based on labelled traces and calibration, never a promised 100% deck decision.

## 9. Rebuild and publication rules

| Changed input | Required derived work |
|---|---|
| Overture topology/connector identity/atomization/restriction state | New canonical build; validate hierarchy reuse only if graph identity is unchanged, otherwise repartition; rebuild indexes/overlays/metrics |
| Descriptive names/signposts only | Regenerate affected cold metadata/guidance artifacts with verified identity compatibility |
| Base speeds or static turn policy | Recompute affected base metrics and leaf/ancestor matrices for affected profiles; keep topology when compatible |
| Live traffic/closure values | New compatible metric snapshot plus dirty-cell ancestor customization; no topology-file patching |
| New exact preference combination | New profile signature and customization; topology is shared when its state coverage is sufficient |
| Complex restriction semantics/state expansion | Rebuild affected canonical state topology and boundary contracts; never treat as a scalar-only edit without proof |
| Disk layout/units | Bump schema; regenerate artifacts and update every reader/tool together |
| Routing kernel only | Rebuild server; rerun reference equivalence, unpacking and concurrency tests |
| Matching emissions/lag/pruning | Version matcher config; rerun labelled trace and transition-equivalence evaluations |

No in-place write to published mapped graph files. Build into a new directory, verify, publish an immutable root and drain readers. Cache and session topology IDs prevent use after incompatible updates. The extraction/build pipeline logs every dependency and completion gate.

## 10. Owner-led implementation order

For each task: explain the minimal DSA statement and failure case; let the owner propose an algorithm and invariants; review; owner implements; compare against the reference; refine and benchmark only then. Do not advance to the next learning problem without the owner's prompt.

First task after this plan is approved: inspect a small downloaded segment/connector sample and manifest. Solve **segment atomization** on one curved polyline with interior connectors, geodetic prefix lengths, repeated coordinates and partial attribute ranges. No partitioner or SIMD implementation is needed to understand that first task.

The long-term decisions are fixed now: compact turn-aware canonical topology; true nested CRP; exact matching metric/overlay semantics; immutable snapshot publication; observable, bounded resource usage. SIMD width, cell size, cache size, strict storage alignment and CPU affinity remain benchmark-selected parameters.

## 11. Decision and progress register

- [x] One target engine; no mandatory legacy-format exporter.
- [x] Preserve bidirectional multilevel Dijkstra and CRP customization.
- [x] Compact global turns; temporary local expansion allowed.
- [x] Start with existing Inertial Flow/Dinic family; preserve and validate full hierarchy.
- [x] Keep all source columns; classify traversal separately from raw coverage.
- [x] Profiles and traffic use compatible immutable snapshots.
- [x] Reject shortcut blending, blind corridor locking and unproved bounded-search exactness.
- [ ] Owner review of this design and initial workload objectives.
- [ ] Phase 0 evidence/contracts/reference fixtures complete.
- [ ] Phase 1 validated acquisition complete.
- [ ] Phases 2-4 canonical graph and candidates complete.
- [ ] Phases 5-7 exact compact CRP and full reconstruction complete.
- [ ] Phase 8 working route/guidance API and profiles complete.
- [ ] Phase 9 HMM/session accuracy gate complete.
- [ ] Phase 10 traffic publication and closure gate complete.
- [ ] Phase 11 prediction gate complete, if pursued.
- [ ] Phase 12 target-machine capacity and deployment gate complete.

## 12. Sources and evidence boundaries

Local inputs reviewed: `docs/ENGINE_FIX_PLAN.md` (2026-10-18 baseline); supplied 31-page `CustomizableRoutePlanningInRoadNetworks.pdf` (July 24, 2013); all three pasted design proposals; current notebook extraction code; current `OverlayGraphConstructor.cpp`, `multi_level_dijkstra.cc`, and partitioner hierarchy code. The earlier audit's file findings are preserved in Section 6; this document is not a claim that every legacy line was re-audited or fixed. Repository status was clean during review.

- **R1 - Delling, Goldberg, Pajor, Werneck: Customizable Route Planning in Road Networks.** Supplied local PDF. Sections 4.1-4.3: compact turns and base graph; 5.1-5.3: nested overlay topology, customization, incremental updates, endpoint offsets and unpacking; 5.3: time-dependent caveats; 5.4: proposed matrix pooling; Section 7: hardware-specific experiments. [Author-hosted copy](https://renatowerneck.wordpress.com/wp-content/uploads/2016/06/dgpw14-crp-journal.pdf)
- **R2 - Valhalla dynamic costing.** Access/edge/transition interfaces and runtime attributes. These ideas motivate the cost interface; they do not establish CRP performance or justify using mismatched overlays. [Official documentation](https://valhalla.github.io/valhalla/sif/dynamic-costing/)
- **R3 - Valhalla architecture and speed data.** Tiled graph and separate traffic sources; no guaranteed footprint for this project. [Architecture](https://valhalla.github.io/valhalla/) / [Speed information](https://valhalla.github.io/valhalla/concepts/speeds/)
- **R4 - Overture transportation model.** Explicit connector-based connectivity, interior connectors and geodetic linear references. [Segments and connectors](https://docs.overturemaps.org/guides/transportation/segments-and-connectors/) / [Linear referencing](https://docs.overturemaps.org/guides/transportation/linear-referencing/)
- **R5 - Overture schema and catalog.** Pin the actual downloaded schema/release rather than assuming today's reference forever. [Segment schema](https://docs.overturemaps.org/schema/reference/transportation/segment/) / [Cloud catalog](https://docs.overturemaps.org/getting-data/cloud-sources/)
- **R6 - Newson and Krumm, 2009: Hidden Markov Map Matching Through Noise and Sparseness.** Reference formulation and data-dependent matching behavior; optional extra signals change the model and need separate calibration. [Microsoft Research paper](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/map-matching-ACM-GIS-camera-ready.pdf)
- **R7 - DuckDB performance and memory.** Larger-than-memory execution, thread limits and profiling; no assurance that every composed UNNEST/aggregate/join plan spills safely. [Tuning workloads](https://duckdb.org/docs/stable/guides/performance/how_to_tune_workloads) / [Out-of-memory issues](https://duckdb.org/docs/stable/guides/performance/oom)

Public source-derived claims are limited to the stated mechanisms. All project RAM, latency, throughput, sharing ratios, matching accuracy and model-inference figures remain measurements to collect unless explicitly labelled as arithmetic examples or acceptance targets.
