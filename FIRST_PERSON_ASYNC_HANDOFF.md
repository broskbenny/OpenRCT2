# First-person asynchronous reconstruction

Base: `032fa8b01f32c0c776553c1f87830968052d6ff9`, branch `first-person/v13`.

## Rendering contract

Camera changes do not synchronously carve scenery, project semantic/track materials,
or construct vehicle hulls. Cold content is queued; ready content continues to draw.
Native tile/entity painting and authoritative state capture still run on the game
thread. Worker jobs own copied metadata, tile elements, geometry, component artwork,
and sprite bytes. Workers neither visit the map/object manager nor issue GL calls.

One CPU worker handles large scenery, small scenery, semantic components, track
geometry/materials and vehicle hulls. Its queue has eight outstanding slots and at
most two unconsumed completions. Submission and polling use try-locks; rejection
leaves work retryable instead of executing it inline. The worker sleeps when idle or
when its completion queue is full. Only application shutdown joins the worker.
Park/POV reset advances an epoch and discards pending results without joining.
Individual requests also validate their identity and source state before publication.
An already running obsolete job may finish in the background, but cannot publish.

Large-scenery spans and text/frame-offset pointers are not borrowed by workers.
Sprites are copied in bounded batches, and the worker lookup has no live-registry
fallback. Large assets reuse worker-owned baked templates; semantic hull keys use
artwork content and relative placement rather than snapshot allocation addresses.
Moving semantic meshes are retained in local coordinates and use the newest native
component transform during rendering. Native component IDs are normalized within
each tile because their original numbering is paint-session-local.

## Per-frame admission and uploads

* Cold visibility discovery: at most 2,048 tile reads, with a 2 ms admission target.
* Native static painting: at most 16 tile/view captures, admitted round-robin.
* Track capture: 16 requests / 1 ms; large-artwork capture: 64 sprites / 1.5 ms.
* Small/vehicle sprite capture: bounded during each native paint batch.
* Semantic capture: 64 sprites / 1.5 ms, round-robin; track materials: 32 / 1 ms.
* Completed CPU results: at most two publications / 0.75 ms admission target.
* Static packet assembly: at most two packets / 1 ms admission target.
* New GL material and static-vertex transfers share 512 KiB per frame. Streamed
  material admission and static-buffer upload each have a 2 ms time slice.

These are admission budgets, not hard real-time guarantees: a unit already started,
packet assembly, GL allocation, normal scene drawing and driver scheduling can exceed
a time target. No worker reconstruction is waited on or run as a cache-miss fallback.

Static packets retain immutable shared surface storage while a replacement VBO is
uploaded in slices. Native coplanar replay order is preserved across slices. A buffer
becomes drawable only when complete. Authoritative source revisions invalidate both
staged and resident buffers; ordinary newly-ready geometry can replace an older
valid buffer without blanking it first. Material admission is fair across streamed
surfaces, including transparency. Atlas growth preserves pixels through a GPU pixel
transfer buffer rather than reading the atlas back into host memory.

The existing 96 MiB GPU cache target applies to unseen residents. Visible content
plus one staged replacement is allowed to exceed it, avoiding both the old unbounded
per-frame streaming fallback and permanent omission of dense visible regions. This
can increase resident VRAM compared with the old streaming fallback.

## Validation

Linux GCC 13 / C++20 Debug GUI, CLI and test targets build successfully. GUI
`--version` runs successfully. Configuration has scripting and Discord RPC disabled;
this checkout's scripting test includes QuickJS unconditionally, so compilation used
`CPLUS_INCLUDE_PATH=$PWD/src/thirdparty/quickjs-ng` even with scripting disabled.

Five new tests cover a deliberately blocked worker, nonblocking reset/publication,
epoch rejection, queue backpressure, source lifetime/immutability, caller-thread
publication and budget exhaustion/retry. All five pass. The broader first-person
run passes 90 tests when the following four existing failures are excluded:

* `FirstPersonAssetReconstructionTest.QuarterCellFallbackKeepsExactOccupancyFaces`
* `FirstPersonPathArtworkTest.SemanticDeckUsesNativeRotationAdjustedSpriteOrigin`
* `FirstPersonVehiclePoseTest.NativePassengerAnchorOwnsPositionWithoutRideName`
* `FirstPersonVehiclePoseTest.MissingAnchorAndSeatCalibrationIsUnsupported`

The first two fail assertions and the latter two crash. All four were reproduced
with the starting renderer/headers and starting test bodies, applying only the
namespace/include fixes and removing the test for the already-removed semantic-role
whitelist so those baseline tests could compile. Existing GCC shadow/unused-variable
errors in touched headers/renderer code were also corrected without changing their
algorithms. No claim is made that the complete unfiltered suite passes.

No interactive park/frame-time benchmark or Windows/VS2019 build was available in
this run. Verify rapid head turns into uncached large scenery, editing/deleting an
object while it is queued, leaving/re-entering POV, loading a different park while
work is running, moving ride components and large texture-atlas growth on the target
machine. Existing transparency peeling and normal scene traversal/drawing are not
made constant-time by this reconstruction refactor.
