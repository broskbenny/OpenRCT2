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

The fallback sprite representation is now selective.

* Guests/entities and trees remain upright passenger-facing impostors.
* Other static tile-element artwork uses a shared world-fixed quarter-turn axis.
* This prevents separately painted components of architecture, rides and supports from independently rotating toward the passenger.

This remains sprite-based reconstruction, not a claim that arbitrary RCT2 artwork contains recoverable volumetric depth.

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
* buildings, multi-tile large scenery, ride parts and supports retain a common physical reconstruction frame across tile boundaries while trees remain upright impostors;
* guests, staff and non-attached vehicles remain smoothly interpolated when visible only to the first-person camera, including after entering from a zoomed-out overhead view;
* parks with more than 256 distinct simultaneously collected scrolling-text variants do not show text from later signs on earlier signs;
* head turns in place do not swap native sprite sides or trigger park-wide static repaints;
* slow physical movement across sprite-sector boundaries does not chatter;
* masked/glass artwork with differing mask/colour offsets stays aligned;
* repeated turns and edits do not cause avoidable resident-region `glBufferData()` churn;
* walking collision meets adjacent full wall edges and remains consistent on sloped walls.

Do not replace the existing stable tag until this manual runtime verification is complete.
