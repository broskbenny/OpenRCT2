"""Foreground 60 Hz diagnostics and native capture budget wiring."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class FrameBudgetTelemetryTest(unittest.TestCase):
    def test_presenter_separates_cpu_time_from_presentation_cadence(self):
        renderer = (ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp").read_text(encoding="utf-8")
        self.assertIn('FP_60FPS_BUDGET', renderer)
        self.assertIn('cpuOver16ms=', renderer)
        self.assertIn('cadenceOver16ms=', renderer)
        self.assertIn('worstPaintUs=', renderer)
        self.assertIn('perfStart - budgetWindow.previousStart', renderer)
        self.assertIn('paintDone - collectionDone', renderer)
        self.assertIn('finished - drawStart', renderer)

    def test_opengl_call_stage_attribution_is_active(self):
        source = (ROOT / "src/openrct2-ui/drawing/engines/opengl/OpenGLDrawingEngine.cpp").read_text(encoding="utf-8")
        self.assertIn("FP_GL_CALL_STAGES", source)
        self.assertIn("flushUs=", source)
        self.assertIn("renderUs=", source)
        self.assertIn("FlushCommandBuffers();", source)
        self.assertIn("_firstPerson->Draw(", source)

    def test_transparency_uses_one_element_upload_for_all_screen_tiles(self):
        gl = (ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp").read_text(encoding="utf-8")
        transparency = gl.split("std::vector<TransparentIndexRange> tileRanges(", 1)[1]
        self.assertIn("range.start * sizeof(GLuint)", transparency)
        self.assertIn("GL_UNSIGNED_INT,indexOffset", transparency)
        self.assertIn("FP_GL_RENDER_STAGES", gl)
        self.assertIn("coverageWaitUs=", gl)
        self.assertIn("executedPeels", gl)
        self.assertEqual(transparency.count("glBufferData,GL_ELEMENT_ARRAY_BUFFER"), 1)
        self.assertNotIn("GLsizei(tileIndices.size())", transparency)

    def test_camera_independent_surface_deduplication_is_hash_indexed(self):
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
        self.assertIn("seenIndependentResident", paint)
        self.assertIn("seenIndependentStreamed", paint)
        self.assertIn("seenByTile.try_emplace(key)", paint)
        self.assertIn("seenIt->second.insert(", paint)
        self.assertNotIn("const bool duplicate =\n                                            std::any_of(",paint)

    def test_experimental_analytic_owner_depth_is_disabled(self):
        gl = (ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp").read_text(encoding="utf-8")
        self.assertNotIn("flat in vec4 fPlane", gl)
        self.assertNotIn("uViewport", gl)
        self.assertIn("GL_LEQUAL", gl)
        self.assertIn("surface.nativePaintOrdinal", gl)

    def test_worker_congestion_retries_native_capture_without_world_invalidation(self):
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")
        small = (ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text(encoding="utf-8")
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
        self.assertIn("void RequestFirstPersonNativeSceneryRetry(", core)
        self.assertIn("found->second.nativeRetryMask |=", core)
        self.assertIn("found->second.rotations[rotation].valid = false;", core)
        self.assertIn("rotationMask |= cached.nativeRetryMask;", paint)
        self.assertIn("cached.nativeRetryMask &=", paint)
        self.assertEqual(small.count("RequestFirstPersonNativeSceneryRetry(ps.MapPos, rotation);"), 3)
        self.assertNotIn("InvalidateFirstPersonSceneTile(ps.MapPos);", small)
        self.assertIn("_reconstructionRepaints.enqueue(", small)

    def test_native_work_is_admitted_in_smaller_foreground_batches(self):
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
        self.assertIn("nativeCapturesRemaining = 2;", paint)
        self.assertIn("FirstPersonFrameBudget captureScanBudget(", paint)
        self.assertIn("256, std::chrono::microseconds(1200)", paint)
        self.assertIn("nativeCapturesRemaining != 0 && captureScanBudget.take()", paint)
        self.assertIn("captureCursor += captureExamined;", paint)
        self.assertIn("scannedTiles=%zu", paint)
        self.assertIn("FP_NATIVE_STALL", paint)
        self.assertIn("nativeWorstAppendUs", paint)
        self.assertIn("worstImage=%u", paint)
        self.assertIn("nativePostUs", paint)
        for stage in ("allocUs=", "setupUs=", "arrangeUs=", "extractUs=", "staticItems=", "entityItems="):
            self.assertIn(stage, paint)
        self.assertIn("nativeCapturesRemaining - 1", paint)
        self.assertNotIn("for (size_t visit = 0; visit < scene.visibleTiles.size(); ++visit)", paint)
        self.assertIn("kTilesPerPaintSession = 32;", paint)
        self.assertIn("FP_PAINT_STAGES", paint)
        self.assertIn("nativeSessionUs=", paint)
        self.assertIn("packetsUs=", paint)
        self.assertIn("selectedViewsDone", paint)
        self.assertIn("reconstructionPrepDone", paint)
        self.assertIn("nativeCaptureDone", paint)
        self.assertIn("assemblyDone", paint)
        self.assertNotIn("requestedTileRotations", paint)
        self.assertNotIn("requested->second", paint)
        self.assertIn("cached.requestedRotation = tileRotation", paint)
        self.assertIn("cached.requestedRotation >= 4", paint)


    def test_first_person_automatically_uses_variable_pacing_without_changing_preferences(self):
        context = (ROOT / "src/openrct2/Context.cpp").read_text(encoding="utf-8")
        tweener = (ROOT / "src/openrct2/entity/EntityTweener.h").read_text(encoding="utf-8")
        controller = (ROOT / "src/openrct2-ui/FirstPersonController.cpp").read_text(encoding="utf-8")
        policy = context.split("bool ShouldRunVariableFrame()", 1)[1].split("void RunGameLoop()", 1)[0]
        self.assertIn("Config::Get().general.uncapFPS", policy)
        self.assertIn("|| EntityTweener::get().hasFirstPersonView()", policy)
        self.assertIn("if (gGameSpeed > 4)", policy)
        self.assertIn("return false;", policy)
        self.assertIn("FP_FRAME_PACING firstPerson=%d variable=%d userUncap=%d", context)
        self.assertIn("bool hasFirstPersonView() const noexcept", tweener)
        self.assertIn("return _firstPersonView.has_value();", tweener)
        self.assertIn("setFirstPersonView(", controller)
        self.assertIn("clearFirstPersonView();", controller)
        self.assertNotIn("general.uncapFPS =", context)


if __name__ == "__main__":
    unittest.main()
