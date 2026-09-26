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
* shared native path surface-image selection rotates consistently for flat and sloped paths.

## Validation status

Source-level review has been performed against the stable checkpoint and the relevant native wall/path/mask contracts.

The repository's CI workflow is configured for push, but the fork currently reports no GitHub Actions runs for `first-person/v13`. Therefore no automated build or test pass is claimed here.

The Windows 7 / VS2019 build constraints documented in `FIRST_PERSON_V13_HANDOFF.md` remain unchanged. The prebuilt GoogleTest binary incompatibility on that host also remains unchanged.

## Required manual verification before creating a new stable tag

Use the same real Windows 7 SP1 / VS2019 path documented in `FIRST_PERSON_V13_HANDOFF.md`, then verify:

* walking mode and ride-attached POV still enter, render and exit normally;
* repeated flat walls and 90-degree wall corners have no gaps;
* sloped walls follow their terrain edge without open wedges;
* flat and sloped supported paths always show a horizontal/ramped walking deck;
* guests on flat and sloped paths are reassessed for foot contact before any peep offset is considered;
* ordinary terrain tile boundaries are seam-free at near and far viewing distances;
* trees retain the desirable upright impostor appearance;
* buildings, large scenery, ride parts and supports no longer independently swivel at shared corners;
* head turns in place do not swap native sprite sides or trigger park-wide static repaints;
* slow physical movement across sprite-sector boundaries does not chatter;
* masked/glass artwork with differing mask/colour offsets stays aligned;
* repeated turns and edits do not cause avoidable resident-region `glBufferData()` churn;
* walking collision meets adjacent full wall edges and remains consistent on sloped walls.

Do not replace the existing stable tag until this manual runtime verification is complete.
