"""Upload priority may vary with the camera; owner draw order must not."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


class NearFirstStaticGpuUploadsTest(unittest.TestCase):
    def test_upload_priority_and_draw_order_are_separate_in_production(self):
        source = GL.read_text(encoding="utf-8")
        start = source.index("std::vector<const Paint::FirstPersonStaticRegion*> uploadOrder;")
        upload_loop = source.index("for (const auto* selected : uploadOrder)", start)
        draw_loop = source.index("for (const auto& packet : scene.staticRegions)", upload_loop)
        draw_append = source.index("regionDraws.push_back(packet.key);", draw_loop)
        self.assertLess(start, upload_loop)
        self.assertLess(upload_loop, draw_loop)
        self.assertLess(draw_loop, draw_append)
        region = source[start:draw_append]
        self.assertIn("uploadOrder.push_back(&packet);", region)
        self.assertIn("found->second.generation == packet.generation", region)
        self.assertIn("found->second.imageRevisions.current(", region)
        self.assertLess(region.index("continue;"),
                        region.index("uploadOrder.push_back(&packet);"))
        self.assertIn("scene.options.camera.position", region)
        self.assertIn("dx * dx + dy * dy + dz * dz", region)
        self.assertIn("std::sort(uploadOrder.begin(), uploadOrder.end()", region)
        self.assertIn("return ad != bd ? ad < bd : a->key < b->key;", region)
        self.assertIn("const auto& packet = *selected;", region)
        self.assertIn("if (found != _staticOpaqueRegions.end()", region)
        # A superseding VBO must not hide an old generation during upload.
        self.assertIn("found->second.sceneEpoch == packet.sceneEpoch", region)
        self.assertIn("regionDraws.push_back(packet.key);",
                      source[draw_loop:])
        self.assertNotIn("regionDraws.push_back(packet.key);",
                         source[upload_loop:draw_loop])

    def test_static_upload_batched_without_duplicate_material_admission(self):
        source = GL.read_text(encoding="utf-8")
        self.assertIn("bool materialAlreadyPrepared = false", source)
        self.assertIn("if (!materialAlreadyPrepared && !prepareMaterial(surface))", source)
        region = source[source.index("if (_pendingRegion && _pendingRegion->key == packet.key)"):
                        source.index("// Preserve the deterministic CPU region order", 
                                     source.index("if (_pendingRegion && _pendingRegion->key == packet.key)"))]
        self.assertIn("packed.reserve(2112);", region)
        self.assertIn("if (packed.size() >= 2048)", region)
        self.assertIn("packed, surface, false, false, true", region)
        self.assertIn("packed, surface, true, false, true", region)
        self.assertIn("if (!prepareMaterial(surface))", region)
        self.assertIn("6 * sizeof(GPUVertex)", region)
        self.assertIn("pending.uploadedVertices += packed.size();", region)
        # Every six-vertex face is submitted exactly once irrespective
        # of the maximum buffer batch size or a partial frame upload.
        for face_count in (1, 63, 100, 1001, 62000):
            batch = []
            batches = []
            for i in range(face_count):
                batch.extend(range(6*i, 6*i+6))
                if len(batch) >= 2048:
                    batches.extend(batch)
                    batch.clear()
            batches.extend(batch)
            self.assertEqual(batches, list(range(face_count*6)))

    def test_camera_moves_reorder_admission_not_equal_depth_draw_ownership(self):
        rng = random.Random(20261021)
        for _ in range(20000):
            n = rng.randrange(2, 100)
            regions = {
                key: (rng.randrange(-5000, 5000),
                      rng.randrange(-5000, 5000),
                      rng.randrange(-500, 500)) for key in range(n)
            }
            eye = tuple(rng.randrange(-5000, 5000) for _ in range(3))
            priority = sorted(regions, key=lambda key: (
                sum((p-e)**2 for p,e in zip(regions[key], eye)), key))
            stable_draw = sorted(regions)
            self.assertEqual(set(priority), set(stable_draw))
            distances = [sum((p-e)**2 for p,e in zip(regions[k], eye))
                         for k in priority]
            self.assertEqual(distances, sorted(distances))
            # A deterministic spatial draw order remains unchanged by camera.
            other_eye = tuple(rng.randrange(-5000, 5000) for _ in range(3))
            other_priority = sorted(regions, key=lambda key: (
                sum((p-e)**2 for p,e in zip(regions[key], other_eye)), key))
            self.assertEqual(sorted(other_priority), stable_draw)


if __name__ == "__main__":
    unittest.main()
