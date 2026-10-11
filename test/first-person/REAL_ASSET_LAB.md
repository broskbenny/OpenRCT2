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
