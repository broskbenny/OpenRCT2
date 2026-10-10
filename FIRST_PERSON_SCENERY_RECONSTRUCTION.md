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

Optional coplanar diagnostics use a spatial broad phase and a 32,768-comparison
limit per packet. A partial scan is reported explicitly. This changes diagnostics,
not geometry ownership or rendering order.

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

Thirteen new tests in `test/tests/FirstPersonRendererTests.cpp` cover compact geometry,
raised bases, openings, contradictory views, malformed bounds, texture footprints,
rotation contracts, excluded families, queue resumption, spatial budgets, cold
collision, publication replacement and collision-mask rotations. They pass in a
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
   diagnostic pair scan is bounded, but the packet itself is not a preemptible unit.
   Native painting, ordinary rendering and GPU allocation remain additional costs.
5. General non-convex scenery still relies on the legacy reconstruction family.
   Multiple ridges, overhangs and ornamental structures need further representation
   work. No new stable tag should be created solely from these fixture results.
