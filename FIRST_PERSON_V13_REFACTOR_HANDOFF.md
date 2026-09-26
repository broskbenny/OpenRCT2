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

### Palette-filter transparency uses a complete logical order

Transparent palette filters remain order-dependent and are not converted to ordinary alpha blending.

Screen-space tiling is retained only as a cost bound: transparent quads are conservatively projected into 128x128 viewport tiles, and a near-plane-crossing surface is assigned to the whole viewport rather than risk missing fragments. There is no six-layer overflow approximation.

Within each screen tile every local candidate is peeled exactly with a two-stage ordering rule:

* stage one selects the farthest remaining **physical** depth;
* stage two considers only fragments at that exact physical depth and selects the earliest remaining stable paint ordinal;
* the selected layer stores the palette-filter row plus ordinal in an R32UI target;
* subsequent passes advance lexicographically by **(physical depth, paint order)**.

The ordinal is a logical tie-break only. World geometry is not moved by arbitrary Z epsilons. After deterministic scene assembly, blended surfaces are rebased to one unique logical order so cached surfaces painted in different presentation epochs cannot lose a coplanar layer.

Screen tiles are disjoint in pixel space, so a surface listed in multiple tiles is scissor-restricted and cannot be composed twice at one pixel.

### Ride POV now consumes one passenger-pose contract

Ride POV is no longer defined as vehicle origin plus a universal eight-world-unit Z offset.

`FirstPersonPassengerPose` now carries:

* world eye position;
* passenger forward/right/up basis;
* car-local calibrated eye offset;
* pinned seat index and seating row;
* whether a ride-specific cabin transform was recovered.

The seat is selected once on ride entry (first occupied seat, otherwise seat zero) and remains stable for the POV session. Render-time presentation and simulation-time spatial audio both consume the same pinned-seat pose contract.

For ordinary tracked/spinning vehicles, native 32-step yaw, track pitch/roll and spinning-car rotation remain authoritative. Seating-row/left-right placement is calibrated from the vehicle object's native sprite bounds because RCT2 rider rows are baked into artwork rather than stored as 3-D seat coordinates.

Top Spin is the first flat ride with a recovered cabin transform: the passenger pose reuses the same native arm X/Y/Z offsets and independent seat-bank frame used by the ride painter. Both 48-frame arm motion and 16-frame seat-bank motion are interpolated cyclically at presentation rate, including wrap boundaries such as 47→0.

Other sprite-baked flat rides deliberately remain on the explicit calibrated fallback until a real ride-specific transform is derived; their animation bytes are not reinterpreted as track pitch/roll.

### Animated static paint is keyed to simulation generation

Animated static tile variants no longer become stale merely because another presentation frame was rendered.

Each cached native-paint rotation records the last simulation animation generation (`currentTicks`). It is repainted when that generation changes, on authoritative invalidation, or on a bounded fallback refresh that is itself keyed to simulation generation. Multiple 120/144 Hz renders between game updates therefore reuse the same native animation paint result and do not accelerate fallback source probes.

### Leaf visibility bounds are persistent

`DiscoverVisibleTiles()` no longer decodes every tile-element stack on every rendered frame.

Each static tile cache entry now retains a semantic min/max-Z visibility bound and its last simulation-generation fallback scan. Normal map invalidation marks that bound dirty immediately. A staggered signature probe remains as protection against mutation paths that bypass normal invalidation without scaling with presentation refresh rate. Dynamic entity admission remains independent through the entity spatial index.

### The dead adaptive quality actuator was removed

The old `FirstPersonQualityController`, `pixelTolerance`, target-frame feedback and scene tolerance field were removed because no renderer choice consumed them after distant-terrain substitution was deleted.

CPU/GPU timings remain diagnostic only. No topology-changing approximation was reintroduced merely to keep the old control loop alive.

### Interpolation admission uses entity-specific visual bounds

First-person tweener admission no longer assumes every guest/staff/vehicle fits a sphere centered 16 units above its anchor with radius 64.

The conservative sphere now derives from the entity's native `spriteData` extents and, for vehicles, the current `CarEntry` sprite bounds. The invariant is that interpolation admission should contain renderer-visible art, especially for unusually large/tall vehicle sprites near a frustum edge.

### First-person positional audio retains native environment semantics

Ordinary positional SFX now derive world/environment attenuation before choosing listener projection.

The existing underground attenuation rule is shared by overhead and first-person listener modes, including continuously refreshed first-person one-shot channels. First-person mode changes listener position/pan/distance; it no longer bypasses that unrelated world acoustic rule.

### Crowd ambience uses the entity spatial index

First-person crowd noise no longer runs a full 3-D spatial calculation over every guest in the park.

It resolves the current first-person listener once, visits only entity tiles whose XY bounds intersect the existing 48-tile hard audio cutoff, and then applies the unchanged exact 3-D distance/gain calculation to those guest candidates. Guests outside the cutoff contributed zero before and still contribute zero.

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
* many screen-disjoint glass/water surfaces scale transparency work with local screen-tile overlap rather than a global layer count;
* several coplanar palette filters on one wall/water plane all survive and compose in deterministic paint order without Z offsets;
* a genuinely layered stack of more than six palette-filter surfaces remains exact rather than entering an overflow approximation;
* long first-person exploration/editing does not monotonically grow reconstruction-group state or recreate expired static-region packets when a group changes source quadrant;
* at 120/144 Hz, an animated static tile is natively repainted at most once per simulation animation generation unless invalidated/fallback-refreshed;
* a stationary dense park no longer walks every admitted tile-element stack every presentation frame merely to rebuild min/max Z;
* first-person ride audio and camera remain attached to the same pinned seat through inversions/spinning and Top Spin cabin motion;
* Top Spin presentation interpolation crosses 47→0 arm and 15→0 seat-bank boundaries without sweeping through the opposite half of the animation;
* unusually large vehicle artwork remains presentation-interpolated until the whole conservative visual bound leaves the first-person frustum;
* ordinary positional SFX retain the native underground attenuation rule in first person;
* first-person crowd ambience is unchanged audibly while large parks avoid full all-guest 3-D distance/gain work;

Do not replace the existing stable tag until this manual runtime verification is complete.
