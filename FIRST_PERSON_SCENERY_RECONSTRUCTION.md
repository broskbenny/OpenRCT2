# Compact scenery reconstruction and bounded publication

Branch: `first-person/v13`. Baseline: `66ed8b048c82349651fad1022c151f9e0564b668`.

## What changed

Eligible static, full-tile small scenery now tries a compact convex height profile
before voxel carving. The family covers flat/sloping planar solids and interior-peak
roofs, including asymmetric peaks and raised bases. There are no object-name or
image-ID exceptions. The footprint is a hypothesis, and the base and upper heights
come from native silhouette evidence. This is not occupancy-box rendering.

Admission requires four distinct native views, no internal row/column gaps, at least
97% candidate coverage, 94% observed coverage, 92% worst-view IoU and at most one
pixel of edge error. Every face must be a supporting plane of the resulting convex
solid. Unsupported artwork retains the existing reconstruction path.

Work is bounded by 512x512 source extents, 65,536 opaque pixels per view, a 65x65
horizontal height search, at most 281 candidate profiles and at most nine faces.
Only the selected candidate gets the full independent raster certificate. Collision
occupancy retains the two-unit grid and its existing cell limits. No finer 3D grid,
all-face material repair or guessed palette filling has been introduced.

The bake now interpolates the projected face vertices used by the depth raster.
Previously, independently rounding each world-space texel displaced samples across
native silhouette edges. Triangular texture cells are sampled inside their covered
UV portion, preventing transparent diagonals when width and height differ. All
material still comes from native source pixels with the existing visibility checks.

Eligible full-tile instances share one canonical asset across four orientations.
Vertices and normals rotate around `(16,16)` at submission. Trees, partial footprints,
animated/glass/fountain scenery retain their separate representation paths.

Completed small-scenery assets enqueue waiting tiles by moving their set. Publication
no longer invalidates all instances in one loop: tile repainting drains at most 32
entries per frame with a 750-microsecond admission target, and reset clears pending
repaints. Pending visual geometry remains absent until ready.

Walking collision previously called the synchronous visual-hull cache on a miss.
It now uses a cheap authoritative quadrant mask immediately and accepts detailed
masks from worker-result publication. Empty observed masks are valid cached outcomes;
they do not reinstate the occupied fallback. Canonical masks rotate for shared
instances. Source identity and reconstruction generation invalidate stale masks.
This can temporarily block space that the detailed evidence later makes walkable.
The fallback is collision-only and never becomes visible geometry.

Optional general-plane diagnostics use a spatial broad phase. Horizontal and
general-plane scans each stop at 32,768 checks or a 500-microsecond cooperative
deadline per packet. A partial scan is reported explicitly. Allocation, individual
checks and cleanup can overrun the deadline. Geometry ownership is unaffected.

## Measured synthetic fixtures

These are procedural native sprite fixtures, **not SUMRF.DAT or ROOF11.DAT**. The
runner executes the actual snapshot, native projection and material-bake functions.
All four shapes were also checked with their base raised by four world units.

| Ground-level fixture | Baseline hull faces | New hull faces | Baseline worst-view IoU | New worst-view IoU | New missing material faces | New transparent covered texels |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Symmetric peak roof | 1,129 | 5 | 0.8602 | 1.0000 | 0 | 0 |
| Asymmetric peak roof | 970 | 5 | 0.8594 | 1.0000 | 0 | 0 |
| Box | 30 | 6 | 0.9524 | 1.0000 | 0 | 0 |
| Wedge | 159 | 5 | 0.9026 | 1.0000 | 0 | 0 |

The unobserved bottom face is deliberately not textured. New covered-texel counts
include triangle boundary cells that can be sampled by the GPU, not just cells with
centres inside the triangle. Original palette indices are checked unchanged.

Representative optimized isolated bakes took approximately 6–10 ms, versus 11–15 ms
for the baseline fixtures on this host. These are worker-stage measurements, not
park frame-time results or a hard latency guarantee.

A 10,000-instance repaint fixture consumed exactly 32 entries under a 32-unit budget,
then resumed without losing or repeating an entry. A 10,000-surface separated-plane
fixture performed zero peer comparisons during insertion, avoiding the former
49,995,000 potential pair checks. Dense and large-bound queries stop at their budget.

## Validation and reproduction

Fourteen new tests in `test/tests/FirstPersonRendererTests.cpp` cover compact geometry,
raised bases, openings, contradictory views, malformed bounds, texture footprints,
rotation contracts, excluded families, queue resumption, spatial budgets, cold
collision, publication replacement, collision-mask rotations and missing-versus-empty
source evidence. They pass in a
focused C++20 runner with warnings treated as errors, and under UBSan.

Eight actual material-bake fixtures pass normally and under UBSan. Run from the
repository root with Python 3 and a C++20 GCC-compatible compiler:

```sh
python3 test/first-person/verify_material_bake.py
python3 test/first-person/verify_material_bake.py --sanitize
```

The runner extracts exact production function bodies and compiles them against the
repository headers. It fails if an expected definition disappears. It is a focused
integration check, not a replacement for a complete application build. The complete
small-scenery include, including capture and publication, also passed a focused
syntax check. No complete GoogleTest, Windows or GUI build is claimed here. The fork
reported no Actions runs for this branch when checked.

The established target build command remains:

```bat
msbuild src\openrct2-win\openrct2-win.vcxproj /t:Build /p:Configuration=Release /p:Platform=x64 /m:1 /p:BuildInParallel=false /p:CL_MPCount=1
```

New implementation code is header-only; no additional translation unit is needed.

## Remaining acceptance work

1. Build on the target Windows/VS2019 host and inspect SUMRF/ROOF11 in all rotations,
   including texture seams and their actual placement height. The original DAT
   artwork was unavailable here; synthetic success does not prove those objects.
2. Compare trains, walls, trees and entrances in the same park. These geometry
   families are not redirected to the new compact fitter.
3. Measure frame-time tails while turning, walking into cold areas, receiving many
   shared instances, changing parks and editing objects while jobs are queued.
4. Static region packet assembly is still synchronous once admitted. Its previous
   diagnostic scans have count and time budgets, but the packet itself is not a
   preemptible unit. Repeated coincident boundary orientations are now aggregated.
   Native painting, ordinary rendering and GPU allocation remain additional costs.
5. General non-convex scenery still relies on the legacy reconstruction family.
   Multiple ridges, overhangs and ornamental structures need further representation
   work. No new stable tag should be created solely from these fixture results.

## Follow-up: missing collision evidence

A production-path regression exposed a distinction missing from the initial worker
publication change: a missing tree sprite could produce the same empty mask as four
successfully checked transparent views. Publishing the former incorrectly removed
collision. Snapshot mask construction now returns an optional result: unavailable,
null-data and palette sources are unknown; only a complete four-view observation may
publish an empty mask. The immediate occupancy fallback therefore remains in place
when source evidence is incomplete.

The production bake runner now checks each missing-view position and a genuinely
observed empty mask, in addition to the eight solid fixtures. The missing-view case
was reproduced as a failing assertion before the correction. All fourteen focused
regressions and the extended production runner pass under UBSan after the fix.

## Follow-up: region assembly stalls

Comparison baseline: `1d524edfabe59a8b21bba6c38364f56838b81236`.

Internal-boundary resolution previously bucketed exact quads, then re-quantized and
sorted those same vertices for every pair. Dense coincident faces repeated millions
of identical geometric decisions. The resolver now caches each owner and quad once.
Small buckets compare normals directly; larger buckets group identical normal
representations and count distinct-owner local/local and local/context pairs
arithmetically. Same-owner faces, authored walls, artwork, partial contacts and
context-only pairs retain their previous treatment. Retained surfaces stay in order.

Horizontal diagnostics had only a log limit, which did not bound the pair scan.
Both diagnostic scans now have work budgets. Requesting only plane diagnostics
also avoids the unrelated horizontal scan. Exhaustion reports a partial result
once per diagnostic generation.

Representative isolated optimized GCC C++20 measurements on this host:

| Prepared-cache fixture | Baseline region function | Revised region function |
| --- | ---: | ---: |
| 4,000 coincident eligible boundary faces, diagnostics off | 1,230 ms | 1.80 ms |
| 4,000 coincident horizontal faces, diagnostics on | 3,679 ms | 1.65 ms |
| 4,000 coincident vertical faces, diagnostics on | 15.63 ms | 1.41 ms |

These execute the complete production `RebuildStaticRegionPacket` body with
deterministic prepared-cache adapters. They are synthetic stress fixtures, not park
frame times or full game integration. Comparison checked retained face order,
vertices and UVs, ownership, image/mask dependencies, selection stamps, bounds,
publication revisions and all interface counters. A mixed fixture also exercised
terrain, water, overlays, semantic surfaces, selected native rotations, track
material projections, large scenery, entrance output and neighbouring context.
Native capture, entrance policy and material acquisition were fixture boundary
adapters, not end-to-end tests of those systems.

The resolver alone took 0.51 ms versus 1,279 ms for 4,000 coincident faces, and
2.76 ms versus 4.83 ms for 16,000 faces arranged as separate ordinary contacts.
Timings are individual observations affected by allocator state and host load.
Distinct normal orientations can still require pairwise work.

Three added GoogleTest cases cover 4,000 duplicate boundaries without losing
walls/artwork, dense shared-owner/context-only cases, and 1,000 deterministic
generated cases checked against an independent pairwise classification oracle.
The oracle covers the small/large bucket boundary, owner domains, invalid quads,
near-threshold normals, signed zero, infinities and NaNs. All nine surface
relationship tests pass in a focused assertion runner, normally and under UBSan.
Four complete-region fixtures pass under UBSan. Existing production material-bake
and missing-versus-empty collision checks still pass under UBSan.

Tests are in `test/tests/FirstPersonRendererTests.cpp`; the GoogleTest filter is
`FirstPersonSurfaceRelationshipTest.*`. No complete GoogleTest or target Windows
build is claimed.

A draft that paused the entire region assembly was not shipped. Native capture
may replace a vector without changing its output fingerprint, invalidating a paused
iterator without a region-content change. A safe scheduler must own immutable
inputs or reacquire them using stable keys and revisions. This patch removes the
demonstrated repetition without changing cache lifetimes or packet publication.
