# Real-asset reconstruction checks

This optional, standalone tool measures native **large-scenery** DAT preview silhouettes
against an externally produced triangle mesh. It does **not** run the first-person
renderer, reconstruct the object itself, certify texture ownership, or guarantee
that architectural openings are correct.

All source art remains local. **Do not commit DATs, screenshots, models generated
from proprietary DATs, or copied palette art.**

## Install and run

Python 3.10+ is required. DAT inspection uses only the Python standard library.
Comparison additionally requires `numpy`, `opencv-python-headless`, and `trimesh`:

```sh
python3 -m pip install numpy opencv-python-headless trimesh
python3 test/first-person/real_asset_eval.py inspect /path/to/SMH2.DAT
python3 test/first-person/real_asset_eval.py compare /path/to/SMH2.DAT /path/to/mesh.glb --baseline 72 --output /tmp/smh2.json
python3 -m unittest discover -s test/first-person -p 'test_real_asset_eval.py'
```

The `--baseline 72` above is **specific to the experimentally calibrated
SMH2 laboratory**, not an RCT2 constant. Use explicit `--center-x`,
`--center-y` when model coordinates differ from the decoded tile layout.
Do not silently optimize registration on evaluation images and claim the
result is an independent validation.

Compare has optional, **explicit** failing conditions:

```sh
python3 test/first-person/real_asset_eval.py compare /path/to/SMH2.DAT /path/to/mesh.glb \
    --baseline 72 --min-view-iou 0.92 --max-interior-misses 0 \
    --max-triangles 750 --require-watertight --output /tmp/smh2-strict.json
```

A failing condition writes machine-readable evidence and exits nonzero.
`--require-watertight` is appropriate **only** when the true object is
physically closed; authentic doorways and arches must not be sealed to pass it.
A missing pixel deep inside a native silhouette is reported separately from
a texture-alpha defect; it is not proof of a physical hole.

## Ground truth established against SMH2.DAT

SHA-256 of the privately supplied DAT:
`97c0540a0882b74a53c9d76a753621474df038cee02b7e2301dfed3809d866d8`

The reusable parser found four tiles and 20 images. Compared to the independent
Python mesh from the October 2026 laboratory (486 triangles), the exact four
view IoUs were **0.8681, 0.8030, 0.8481, 0.8124**. The unsimplified mesh
(6,128 triangles) scored **0.8297, 0.8122, 0.8260, 0.8083**. The proposed
strict gates **fail** for the candidate, as intended.

These are checks of an independent experimental mesh, NOT the OpenRCT2 production
path. The source preview images were also used by that experiment to build the
mesh, so these IoUs are not held-out scores.

## Next useful integration

Use actual tile-sequence sprite paint/compositing as reconstruction input and
reserve preview images for evaluation; validate depth-owner material evidence
and real openings separately. Then export production-generated meshes through
a deterministic test adapter, enabling headless comparison against these same
checks. More DATs should cover both enclosed and genuinely open structures,
different roof profiles and different tile layouts.

## Native source-composite consistency (SMH2)

The original DAT includes four preview sprites and four painted tile
sequences with four direction images per sequence. Unlike independently
captured photographs, these preview pixels are **not independent source
observations**. The same indexed artwork appears within the per-tile composites.
A calibrated translation contains all native preview foreground pixels with
the **exact palette index**:

| Rotation | Exact preview pixels | Preview foreground | Other composite foreground pixels |
| --- | ---: | ---: | ---: |
| 0 | 2715 | 2715 | 3529 total |
| 1 | 2523 | 2523 | 3253 total |
| 2 | 2439 | 2439 | 3272 total |
| 3 | 2623 | 2623 | 3349 total |

The registration is fitted against each preview, so this is a proof of
**decoding and palette consistency after fitted translation**, not an
independent test of geometry or an independently determined world placement.
Extra composite pixels need separate analysis; they are not scored as failed
preview containment. OpenRCT2's full painter remains the source of truth for
scene occlusion/ownership.

To reproduce without committing a DAT:

```sh
python3 test/first-person/audit_native_artwork.py /path/to/SMH2.DAT \
  --require-exact-preview-containment --output /tmp/smh2-native-audit.json
python3 -m unittest discover -s test/first-person -p 'test_*.py'
```

**Important corrective finding:** Any score described as a model being
independently validated by SMH2's previews must be revised. The previews
reuse tile-artwork pixels and were also used to construct the experimental
mesh. Future native appearance checks must compare geometry and palette
ownership per frontmost 3D surface and report genuinely held-out-view results
separately. This audit does not execute the game's C++ reconstruction.

### Observed sequence attribution

For SMH2, the exact preview-palette pixels were attributed by the fitted,
approximate native depth-order composite to these tile sequences
(`[sequence 0, sequence 1, sequence 2, sequence 3]`):

- Rotation 0: `[0, 0, 202, 2513]`
- Rotation 1: `[0, 2523, 0, 0]`
- Rotation 2: `[2243, 0, 196, 0]`
- Rotation 3: `[0, 0, 2613, 10]`

The owner map is screen-pixel provenance from native 2D compositing;
**it is not proof that a given reconstructed 3D triangle belongs to that
sequence**. Establishing 3D frontmost material ownership is the next
nontrivial integration step. Fitted image shifts must not be silently
baked into the runtime reconstruction transform.

The optional `.github/workflows/first-person-reconstruction.yml` schedules
portable source-artwork regressions and the existing focused production
small-scenery material bake on relevant branch pushes and PRs. Its presence
does not imply GitHub Actions has executed successfully; check workflow status
before treating CI coverage as verified.

## Three-dimensional free-space preservation

Four-view silhouette IoU is insufficient to certify simplification. An open
passage may be sealed inside the model while all native projections look
nearly identical. Use `check_mesh_free_space.py` to compare a candidate
**watertight triangle mesh** with the occupancy it was reconstructed from:

```sh
python3 test/first-person/check_mesh_free_space.py /private/occupied_xyz.npy \
  /private/candidate.glb --step 2 --origin -1 -1 -1 \
  --max-sealed-exterior-cells 0 --max-lost-solid-cells 10 \
  --output /tmp/space.json
```

Important: `--origin -1 -1 -1` applies **only to the independent October
2026 Python prototype**. Its marching-cubes vertices were offset one world
unit from the original sampled cell-centre positions. The validator explicitly
requires the model's true sample registration; an uncalibrated offset can
manufacture hundreds of false errors. Production geometry must use its own
correct origin, not inherit this workaround.

On the private SMH2 occupancy (shape 32×32×24, 6,250 occupied cells), the
experiment measured:

| Candidate | Faces | False-filled empty | Sealed exterior free | Lost solid | Ambiguous rays |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw marching-cubes hull | 6,128 | 0 | 0 | 9 | 2 |
| Clustered compact mesh | 486 | 76 | 76 | 493 | 0 |

These counts use the corrected mesh/occupancy registration. The clustered
candidate **fails strict gates**, notwithstanding its compact geometry,
watertightness and apparently reasonable four-view silhouette IoU.

The test uses parity intersections of vertical rays and explicitly fails
ambiguous ray counts; it does not require `rtree`. It cannot validate a
non-watertight open shell, prove that the source occupancy respects genuine
architectural openings, or certify native palette mapping. Do not commit the
original asset or source-derived occupancy/meshes to this repository.

## Material source transparency safety

`FirstPersonMaterialContinuity.h` can replace only isolated **unknown**
checkerboard material samples surrounded by direct native-colour evidence.
It must never change a native-known transparent pixel. The actual SMH2 source
contains isolated transparent pixels; blindly painting them opaque would
corrupt the artwork. Production selection also excludes authored negative
passages and caps the face-size budget. This is a narrowly scoped material
quality correction, **not** a substitute for a connected-surface reconstruction.

### Conservative candidate that preserves native-grid free space

A second search over real SMH2 occupancy tested 66 combinations of Gaussian
preconditioning and vertex clustering. After correcting the prototype's
one-unit XYZ mesh offset, one candidate using blur sigma 0.35 and clustering
grid 1.5 had these **measured but experimental** characteristics:

- **5,198 triangles**, compared with the 6,128-triangle raw hull (~15.2% fewer)
- Native preview four-view mean silhouette IoU: **0.82625** (in-sample)
- Re-tested at the native 2-unit lattice: **0 falsely filled empty cells,
  0 lost occupied cells, 0 ambiguous vertical rays**
- At 1-unit spacing (the input binary cell occupancy upsampled 2× in XYZ):
  **1,238 falsely filled and 1,463 lost cells**, no ambiguous rays
- Geometry watertight; **native palette coverage still unverified**

The sub-grid deviations show that even a model passing native-grid sampling
can diverge between samples. Its original shape is not guaranteed by either
the matched silhouette or the watertight property.

This conservative candidate is an **independent Python experiment**, not the
OpenRCT2 production algorithm or a production-ready mesh; it must not be
silently promoted. Stronger admission should combine grid + sub-grid
volume checks, connected-component/opening preservation, four-view
projective error, material provenance, and reconstruction cost.

The original 486-triangle candidate fills 76 native-grid empty cells and
loses 493 occupied cells. It remains deliberately **rejected** despite
its apparent silhouette quality.

## Production candidate volume admission

The production planar path now has an additional conservative gate:
`FirstPersonVolumePreservation.h` compares candidate closed-surface geometry
with **both occupied and empty voxel centres** in the original C++ visual hull.

It first checks every candidate polygon edge is paired with another
opposite-winding edge, rejecting open/nonmanifold seams (including a missing
vertical wall, which simple top-down ray parity cannot detect). It then
uses bounded vertical ray-parity probes, disallowing newly closed passages,
lost solid occupancy, or ambiguous odd intersection counts. The check is
short-circuited **after** four-view silhouette checks and limited to
2,000,000 face/column comparisons. On any failure or exceeded budget, the
previous reconstruction is retained.

Portable tests compile and run the **actual production C++ header** using
`test_production_volume_preservation.py`; native full-scene C++ execution,
GPU visualization, and in-game FPS are not tested by this adapter.

The geometry certificate cannot establish real doors absent from the input
visual hull; nor does it validate native palette mapping or finer sub-grid
appearance. Structural openings remain a separate evidence requirement.


## Exact connected planar-patch retessellation (offline candidate)

A real-asset experiment found that **connected, exactly coplanar** triangles
can be recombined into polygonal surface regions, then retriangulated while
preserving *every original boundary edge and vertex*. No voxel cells, texture
pixels, or world-space vertex positions are altered by this operation.

On the privately supplied SMH2 DAT, using the independent marching-cubes
visual hull from the research laboratory:

| Measurement | Original | Connected planar patches |
| --- | ---: | ---: |
| Triangle count | 6,128 | **3,660** |
| Triangle reduction | — | **40.3%** |
| Watertight | yes | yes |
| Consistent winding | yes | yes |
| Euler characteristic | -16 | -16 |
| Signed enclosed-volume delta | — | **0** |
| View 0 IoU | 0.8297 | 0.8297 |
| View 1 IoU | 0.8122 | 0.8122 |
| View 2 IoU | 0.8260 | 0.8260 |
| View 3 IoU | 0.8083 | 0.8083 |

All four projected **binary silhouettes were pixel-for-pixel identical**, not
merely close in average IoU. **169** patches were successfully replaced,
saving **2,468** triangles. There was no need to guess new vertex positions
or seal known openings. The nontrivial original tunnel topology remained
unchanged.

The reusable source in \`retessellate_planar_patches.py\` implements strict
boundary-loop, coplanarity, area, contour edge, manifold/winding, Euler,
and signed-volume checks. The synthetic tests include a subdivided box,
a genuine through-opening, and non-coplanar shapes. Run:

\`\`\`sh
python -m pip install numpy trimesh 'shapely>=2.1' scikit-image
python test/first-person/retessellate_planar_patches.py \\
  /private/visual_hull.glb /tmp/retessellated.glb --report /tmp/patches.json
python -m unittest discover -s test/first-person -p 'test_retessellate_planar_patches.py' -v
\`\`\`

This is **not a production renderer modification**. The source mesh comes from
an independent research prototype; it is not the game's current greedy
voxel-face boundary. This pass changes triangle connectivity, not 3D surface
geometry, and it **does not solve source-artwork UV ownership, porosity caused
by incorrect alpha, or intrinsically voxel-derived silhouette staircases**.

The useful next step is to exploit these exact planar regions as larger
native-textured surface domains, and separately fit non-coplanar roof/wall
surfaces under existing four-view plus 3D free-space certificates. Do not
promote this geometry-only export into the renderer without material and
runtime validation. Never commit private DAT-derived meshes.


## Exact native C++ connected-planar surface operator

The geometry-only algorithm is now implemented in **production-compatible
header-only C++** as \`FirstPersonConnectedPlanarMesh.h\`. It accepts indexed
triangles and 3D vertices (without moving any source vertex), clusters
edge-connected oriented coplanar triangles, extracts simple patch boundaries,
ear-clips the region, and admits the replacement only if:

- the patch has one unambiguous closed boundary loop (holes/branches fall back);
- the result uses fewer triangles, with identical patch border edges;
- its triangles share the original outward direction, without zero-area folds;
- the total 3D patch area agrees with the original within a strict tolerance.

The C++ test runner consumes a native binary triangle mesh and returns indexed
triangles. No proprietary asset is bundled. \`test_native_connected_planar.py\`
compiles the exact C++ header with GCC and undefined-behavior sanitizer and
tests a heavily subdivided box, a genuine through-opening, a noncoplanar sphere,
and (when \`SMH2_RAW_HULL_MESH\` is set) the actual privately supplied raw hull.

All four tests passed locally against the exact source committed to Git,
including the optional SMH2 fixture. Actual SMH2 C++ result:
**6,128 → 3,660 triangles (40.3% fewer), 169 regions retessellated**;
unchanged enclosed volume, Euler characteristic -16, consistent winding,
watertight and all four original view masks **pixel-identical**. The
standalone C++ operation took approximately 6 ms in a local optimized run
(nonportable measurement, excludes sprite decoding, GPU uploads and textures).

**The header is NOT called by the production renderer yet**: current
large-scenery surfaces are independently textured quads, while this algorithm
outputs triangles. Converting them directly into independent material patches
would risk *increasing* draw calls and seams. Native artwork must be mapped
onto connected planar regions coherently first; only then should this
algorithm replace the existing voxel-face publication path. There is no
claim of improved game FPS or solved roof appearance.

### Real-source surface provenance (experimental)

For the 3,660-triangle SMH2 result, there were 1,001 connected oriented
coplanar surface regions. A depth-tested four-view raster found direct
native foreground pixels for 515 regions; 486 had no such evidence. Some
regions may be fully occluded, internal or on undersides.

Selecting a single native view per region and projecting that view's palette
index directly resulted in a 100% same-view match **by construction** using
pixel-cell (floor) sampling. It does **not** independently validate materials:
cross-view exact palette agreement remained very poor (roughly 1–7% in visible
overlap), due to native view-dependent artwork and/or surface correspondence.
The experimental pipeline had previously rounded pixel-centre projections,
causing avoidable one-pixel lookups. This is a Python sampler defect, not proof
that the OpenRCT2 C++ integer-coordinate projection is wrong. Do not change
production source projection on that basis alone.

The candidate textured GLB remains diagnostic: unknown material stays
visibly marked, never silently painted over or made transparent.


## Unobservable underside diagnosis (October 12, 2026)

In the production large-scenery baker, `face.normal.z < -0.5f` previously
skipped *all* ordinary down-facing boundaries, before any native-material
sampling. Additionally, an underside with no native-visible source was
discarded when `fallbackImage` remained unset. These are omitted 3D faces,
not merely black/transparent pixels.

Accepted elevated underside faces now bypass those omissions and receive
the existing conspicuous unknown-material checkerboard. Ground-contact
bottoms remain hidden to avoid Z-fighting with terrain. Native artwork is
not hallucinated. The diagnostic texture is capped at 16×16 palette
pixels per face for memory/CPU control; its actual 3D geometry is unchanged.
Entrance-specific opaque-boundary handling is unchanged.

**Crucial limitation:** the original SMH2 pick log independently records
`REJECTED_MATERIAL pixel=0` on a *side-facing* reconstructed face, with
normal approximately `(0.980,-0.140,0.140)`. Therefore missing downward
faces do **not** explain all observed pixel porosity. The reported
`1165 boundary / 969 materialized / 924 opaque` face counts cannot
uniquely attribute their missing material to undersides.

No complete Windows build, renderer frame or FPS result has been verified.
This marker is an evidence/debuggability improvement, not a certified
overall porosity fix.
