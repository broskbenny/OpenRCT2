# OpenRCT2 First-Person v13 Post-Checkpoint Refactor Handoff

## Scope

Repository: `broskbenny/OpenRCT2`

Branch: `first-person/v13`

Stable pre-refactor checkpoint: `first-person-v13-win7-known-good-2026-09-26`

Stable checkpoint commit: `59e4d72e2603564001298fdae1424d91e973faf6`

This document describes the renderer refactor applied after the known-good Windows 7 checkpoint. The stable tag is intentionally unchanged and remains the rollback point.

## Refactor goals

The refactor addresses the independently audited first-person defects:

1. wall planes and walking collision inherited undersized native sorting boxes;
2. supported footpaths can omit their separate surface sprite;
3. connected static artwork was allowed to rotate as independent billboards;
4. head-look quarter-turn thresholds could repaint static tiles and pop sprite sides;
5. mixed cache hits/misses could reorder otherwise identical GPU geometry;
6. path geometry was artificially raised into guest sprites;
7. terrain sprite transparency could expose seams despite shared world vertices;
8. raw-masked placement used colour-image offsets instead of the native mask-image contract.

## Implemented architecture

### Walls

`BuildFirstPersonWallPlane()` is now the shared semantic wall definition used by rendering and walking collision.

* Walls occupy the complete 32-unit tile edge rather than 28/29-unit paint sorting bounds.
* Native wall slope values are respected.
* Slope endpoint elevation is 16 world units, matching native wall placement semantics.
* Walking collision interpolates the wall base at the actual XY contact point instead of expanding a sloped wall into a rectangular prism.
* Wall visual height uses the object height rather than the painter's `-2` sorting-box reduction.
* Doors remain excluded from the solid semantic plane because their animated/open-panel geometry is not yet reconstructed.

### Paths

Native path surface-image selection is now shared through `GetPathSurfaceImageOffset()`.

* A semantic walking deck is emitted from authoritative path/bridge artwork even when the native painter omits the separate surface sprite.
* Bridge/support sprites are not flattened.
* Path geometry remains at the true simulation floor height; the previous `+1` world-unit lift was removed.
* Coplanar separation is handled by a small depth-only render bias, so physical geometry and UV mapping remain truthful.
* Synthesized bridge decks compute their own native surface sprite origin rather than inheriting an arbitrary support/fence root position.
* Deck synthesis is accepted only from a native path-surface or bridge root so ghost/highlight remaps come from the correct path image template.

### Static scenery representation

Only artwork backed by meaningful world geometry is reconstructed as a fixed plane.

* semantic walls and path decks are built from authoritative world geometry and derive UVs by projecting those world points back into the selected native image;
* genuinely narrow/tall physical paint bounds may still become fixed planes when they represent a real wall-like surface;
* arbitrary track, large-scenery, small-scenery and support artwork is not assigned a false world-fixed horizontal axis;
* connected large-scenery/track fragments instead share one canonical group anchor and one native source rotation, forming a coherent passenger-facing group impostor;
* entities and ordinary isolated props are passenger-facing impostors.

This intentionally prefers a truthful 2-D impostor over fabricated 3-D orientation when the source asset contains no recoverable physical surface.

### Native sprite-side selection and cache

Native quarter-turn sprite selection is no longer driven by head gaze.

* Sprite side is selected from passenger position relative to the tile.
* The exact tile-centre singularity preserves the previous side instead of falling back to head-look.
* A five-degree hysteresis margin prevents physical movement near a 45-degree boundary from toggling sides repeatedly.
* Each static tile retains all four painted rotation variants.
* Crossing a sector therefore reuses a previously painted variant instead of destroying it.
* Semantic edits invalidate all variants.

### Stable GPU ordering

Freshly painted static surfaces are no longer appended after cache hits.

* Both cache hits and misses are collected into their tile cache first.
* The visible static scene is then flattened once in deterministic visible-tile order.
* GPU-region fingerprints remain order-sensitive, but equivalent content no longer changes order merely because one tile repainted.

### Terrain seams

Terrain world geometry remains exact 32-unit shared-edge geometry.

* Terrain surfaces now opt into a one-pixel four-neighbour palette coverage fallback when the sampled source pixel is transparent.
* The fallback is terrain-only; walls, scenery, path artwork and masks keep their authored transparency.
* This targets stepped transparent boundaries in the source isometric land sprite rather than hiding the problem by overlapping world geometry.

### Masked artwork

First-person raw-masked placement now follows the native OpenGL contract.

* Composite placement uses mask-image offsets.
* Composite dimensions are clipped to the minimum mask/colour dimensions.
* Colour and mask textures share the same local composite coordinates.

### Guest grounding

No arbitrary guest Z offset was added.

The path's false `+1` elevation was removed first because it was a confirmed source of lower-sprite intersection and UV displacement. Residual guest-foot appearance, if any, should now be inspected against actual walking frames and slope cases before introducing peep-specific grounding.

## Regression coverage

`FirstPersonRendererTests.cpp` now checks:

* walls use exact complete tile edges and adjacent edges share their corner;
* native wall slope raises the correct endpoint while preserving wall height;
* sloped walking collision uses the wall height at the actual contact point;
* shared native path surface-image selection rotates consistently for flat and sloped paths.

## Validation status

Source-level review has been performed against the stable checkpoint and the relevant native wall/path/mask contracts.

The repository's CI workflow is configured for push, but the fork currently reports no GitHub Actions runs for `first-person/v13`. Therefore no automated build or test pass is claimed here.

The Windows 7 / VS2019 build constraints documented in `FIRST_PERSON_V13_HANDOFF.md` remain unchanged. The prebuilt GoogleTest binary incompatibility on that host also remains unchanged.

## Second independent audit corrections

A subsequent audit at `d6f67d06a42a36b72b47b8d66f0577b032361b8c` found five additional first-principles defects. They are corrected on the current development branch.

### Native path sprite origin

Semantic path UV projection now uses the exact rotation-adjusted tile sprite origin shared with `TileElementPaintSetup`.

The regression suite verifies the former wrong-minus-native projection deltas explicitly:

* rotation 0: `(0, 0)`
* rotation 1: `(-32, -16)`
* rotation 2: `(0, -32)`
* rotation 3: `(+32, -16)`

Geometry remains in unrotated world coordinates; only source-art projection uses the native shifted sprite origin.

### Vehicle yaw convention

Ride POV no longer treats the 32-step native vehicle orientation as a conventional +X-zero angle.

`FirstPersonVehicleYawRadians()` now preserves all 32 heading steps while reflecting the X component into camera coordinates:

`native forward = { -cos(theta), +sin(theta) }`

This gives the audited cardinal mapping `0=-X, 8=+Y, 16=+X, 24=-Y` without quantising curved headings through the eight-way free-roam table. Presentation interpolation and simulation-time ride audio both consume this same conversion.

### Connected static-art reconstruction frames

Native artwork selection and physical fallback-plane orientation are now separate concerns.

* multi-tile large scenery reconstructs around the canonical object placement origin derived from its sequence offset and placement direction;
* multi-sequence track/support artwork reconstructs around `GetTrackSegmentOrigin()`;
* non-tree small scenery uses its stable tile/placement frame;
* trees (small and large scenery) and entities remain passenger-facing impostors.

Adjacent tiles belonging to one connected object therefore no longer acquire perpendicular physical sprite planes merely because their tile centres select different native paint rotations.

### First-person interpolation admission

`EntityTweener` now accepts ordinary guests, staff and vehicles when they intersect the first-person camera frustum, independently of overhead viewport bounds or zoom.

The first-person controller publishes the current presentation camera/frustum to the tweener and clears it on POV exit. The existing attached-vehicle tracking remains in place.

### Immutable scrolling text

First-person collection no longer relies on the eventual meaning of one of the 256 mutable scrolling-text image IDs.

During a first-person native paint session, scrolling-text bitmap bytes are captured when the corresponding `PaintStruct` or attached paint entry is created. The resulting `FirstPersonSurface` owns immutable pixels. At draw time those pixels are uploaded to transient atlas slots which are released for reuse on the following frame.

Snapshot-backed surfaces are streamed rather than stored in resident static-region VBOs, and tiles containing them are repainted as animated content. The normal 2-D renderer and its existing 256-slot scrolling-text cache remain unchanged.

## Third independent audit corrections

A further audit at `3d86396e2ae5033f4abd0ad6b5f7f39dd6e70f59` found five structural mismatches that remained after the second pass. They are corrected on the current development branch.

### Native rotation is now reconstruction-group state

The four native quarter-turn tile variants remain the source-image cache, but a tile no longer has to choose one native view for every object it contains.

* multi-tile large scenery has a stable group key derived from its canonical placement origin, direction and object entry;
* each connected track piece has a stable group key derived from `GetTrackSegmentOrigin()`, ride identity and track type;
* native source rotation for those groups is selected from the passenger position relative to that canonical group origin;
* every fragment of one connected group is reconstructed around that same canonical anchor;
* unrelated groups on one tile may legally select different cached native variants;
* grouped arbitrary sprite artwork is streamed as one coherent group impostor rather than being stored as resident fixed geometry.

Reconstruction-group state contains only the selected source rotation and `lastSeen`; it carries no historical GPU-region membership and expires with the ordinary cache lifetime.

### Persistent static regions are the render unit

Fixed opaque geometry no longer has to be copied into a fresh `FirstPersonScene`, individually frustum-tested, regrouped and vertex-fingerprinted every frame.

The persistent CPU region packet now owns:

* the selected fixed opaque surfaces;
* a conservative region bound;
* a generation number;
* compact unique texture dependencies.

The GL region buffer retains the matching generation and a texture-revision dependency stamp. If both still match, an ordinary frame region-frustum-tests the packet and draws the existing VBO without static surface traversal, vertex hashing or packing.

Native tile caches also separate `residentSurfaces` from `streamedSurfaces`, so the collector does not iterate resident non-animated surfaces merely to skip them. Billboards, transparency, immutable scrolling text, animated tiles and entities remain streamed by design.

Visibility discovery, live terrain admission and entity occupancy still perform tile-level work; this change specifically removes the previous O(visible static geometry) CPU reconstruction/hash path rather than claiming that the whole scene collector is O(regions).

The former 2x/4x flat-terrain substitution has been removed. Full-resolution per-tile terrain remains resident in static regions at every distance, preserving the exact world-to-texel scale instead of stretching one tile image over a larger physical area.

### Walking follows swept floor topology

Walking translation no longer accepts an arbitrary destination terrain height and snap the camera to it.

The movement segment is sampled across the actual terrain/path floor field:

* continuous native terrain and path slopes are followed sample-by-sample;
* an explicit native 8-world-unit step is allowed;
* larger upward or downward discontinuities are rejected as cliffs/ledges;
* exposed water is non-walkable when terrain is the selected floor;
* an actual path surface can still provide the accepted floor over water.

The final camera Z comes from the same accepted swept floor traversal used to validate the move. The native-step limit is exposed in the math layer and has regression coverage.

### Arbitrary scenery no longer receives a fabricated fixed plane

The interim semantic classifier for small scenery has been superseded by the stricter projection invariant above.

Unless an artwork root can be reconstructed from meaningful world geometry, it remains an impostor. This applies to compact props, rotatable signs/decorations, large scenery, track artwork and supports. Connected large-scenery/track pieces still share a canonical group anchor/source view, so this does not reintroduce per-tile independent rotation.

### Renderer and tweener share one resolved view

`FirstPersonResolvedView` now owns the resolved camera, FOV, aspect, near plane and complete-park far plane.

`ResolveFirstPersonView()` applies `CompleteParkFarClip()` once, and the same resolved view contract is consumed by:

* first-person scene/frustum collection;
* OpenGL projection/depth range;
* entity interpolation admission.

There is therefore no longer a 32,768-unit interpolation cutoff inside a larger renderer far plane. Regression coverage verifies that a 1024x1024 park resolves beyond 46,000 world units.

### Static-region invalidation details

Region invalidation immediately dirties affected packets. Dirty tile-level terrain/static cache entries are excluded from a packet until their live state is revalidated, preventing an edited off-centre tile from surviving inside an otherwise visible old VBO.

Expensive packed-tile signatures and reconstruction-group discovery are no longer performed every frame. They run on authoritative invalidation and on staggered fallback semantic probes, preserving protection against direct game-side mutations that bypass normal invalidation hooks.

## Fourth independent audit corrections

A further audit at `f60ae08ef22928c6d6c91a13fa02082f92b10653` identified five remaining renderer invariants. They are corrected on the current development branch.

### Generic sprite reconstruction no longer invents a fixed world axis

The previous fallback mapped native sprite X directly onto a horizontal world vector even though the native isometric projection couples X/Y/Z. That mapping could not reproduce its own source view.

The fallback is now a passenger-facing impostor. Connected large-scenery and track fragments still use one canonical group anchor and source rotation, so their native image-space offsets remain mutually coherent. World-fixed geometry is reserved for semantic walls/path decks and genuine physical planes whose UVs are derived from projecting their actual world vertices.

### Moving entities own their native source rotation

`Paint.Entity` now exposes a targeted one-entity native paint entry point.

The first-person collector:

* computes source rotation from the current presentation position of each entity, not its tile centre;
* retains five-degree hysteresis independently by `EntityId`;
* queues each entity into the matching native rotation paint session;
* continues to use the native 32-direction orientation lookup inside the ordinary entity painter.

Two entities in the same tile may therefore legitimately use different native source quadrants. Entity rotation state is also lifetime-bounded.

### Terrain texel scale is invariant

The 2x/4x terrain patch substitution has been removed. It stretched one 32x32 tile image across 64x64 or 128x128 world areas and estimated screen error from an artificial z=0 patch centre.

Static-region residency already removes the principal CPU/VBO cost of exact terrain, so every admitted terrain tile now retains its native world dimensions and UV mapping at all distances.

### Palette-filter transparency is peeled per screen-space tile

Transparent palette filters remain order-dependent and are not converted to ordinary alpha blending.

Instead of using the total number of transparent quads in the entire view as the peel depth:

* transparent surfaces are conservatively projected into 256x256 viewport tiles;
* each screen tile contains only surfaces whose projected quad can touch its pixels;
* the exact peel count is bounded by that tile's local candidate count, up to the existing six-pass exact limit;
* overflow approximation is also tile-local;
* a near-plane-crossing surface is conservatively assigned to the whole viewport rather than risk missing fragments.

Screen tiles are disjoint in pixel space, so a surface listed in multiple tiles is scissor-restricted and cannot be composed twice at one pixel.

### Reconstruction-group state is lifetime-bounded

Reconstruction groups no longer retain historical GPU-region sets.

A group now stores only source rotation and `lastSeen`; stale entries expire with the ordinary 240-frame cache policy. Because arbitrary grouped sprite artwork is streamed rather than resident fixed geometry, group-sector changes do not recreate or dirty historical static-region packets.

## Required manual verification before creating a new stable tag

Use the same real Windows 7 SP1 / VS2019 path documented in `FIRST_PERSON_V13_HANDOFF.md`, then verify:

* walking mode and ride-attached POV still enter, render and exit normally;
* repeated flat walls and 90-degree wall corners have no gaps;
* sloped walls follow their terrain edge without open wedges;
* flat and sloped supported paths always show a horizontal/ramped walking deck in all four native paint rotations, without texture displacement or disappearance;
* ride POV heading agrees with native vehicle travel at cardinal and intermediate 32-step orientations, including ride audio left/right orientation;
* guests on flat and sloped paths are reassessed for foot contact before any peep offset is considered;
* ordinary terrain tile boundaries are seam-free at near and far viewing distances;
* trees retain the desirable upright impostor appearance;
* buildings, multi-tile large scenery, ride parts and supports retain coherent shared group-impostor placement/source views across tile boundaries without strange fixed-card orientation;
* guests, staff and non-attached vehicles remain smoothly interpolated when visible only to the first-person camera, including after entering from a zoomed-out overhead view;
* parks with more than 256 distinct simultaneously collected scrolling-text variants do not show text from later signs on earlier signs;
* head turns in place do not swap native sprite sides or trigger park-wide static repaints;
* slow physical movement across sprite-sector boundaries does not chatter;
* masked/glass artwork with differing mask/colour offsets stays aligned;
* repeated turns and edits do not cause avoidable resident-region `glBufferData()` churn;
* walking collision meets adjacent full wall edges and remains consistent on sloped walls.
* a multi-tile large-scenery object and a multi-sequence track piece keep one coherent native source view across tile and 32x32 GPU-region boundaries, including near 45-degree source-view thresholds;
* a tile containing unrelated connected groups can select different native variants without mixing variants inside either group;
* in a dense static park, steady camera frames reuse resident region packets/VBOs without static-surface copy, cull, grouping, vertex hashing or repacking; exact terrain texel scale remains unchanged with distance;
* walking into a vertical terrain cliff stops instead of snapping upward, and stepping off a large ledge stops instead of snapping downward;
* legal terrain/path slopes and an 8-world-unit step remain traversable;
* exposed water blocks walking while valid path surfaces over water remain usable;
* compact, directional and connected arbitrary scenery remains visible from arbitrary azimuths without being forced onto a mathematically false fixed plane;
* on a very large park, guests/staff/vehicles visible beyond 32,768 world units are admitted to the same presentation interpolation range as rendered scenery;
* two guests/vehicles within one tile but on opposite sides of a 45-degree camera-relative source boundary can show different native sprite quadrants without waiting for a tile crossing;
* a moving entity crossing a native source-view boundary changes side with its own hysteresis and does not pop merely because its spatial-index tile changes;
* flat terrain keeps identical apparent texel scale before and after distances that previously selected 2x/4x substitutions, including elevated terrain and pitched cameras;
* many screen-disjoint glass/water surfaces do not trigger six global redraw passes; transparency cost scales with local screen-tile overlap instead;
* a genuinely layered stack of more than six palette-filter surfaces still uses six exact local peel passes plus the deterministic overflow surface;
* long first-person exploration/editing does not monotonically grow reconstruction-group state or recreate expired static-region packets when a group changes source quadrant;

Do not replace the existing stable tag until this manual runtime verification is complete.
