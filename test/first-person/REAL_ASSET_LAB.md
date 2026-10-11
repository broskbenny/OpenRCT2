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
