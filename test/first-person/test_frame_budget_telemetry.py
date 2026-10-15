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

    def test_native_work_is_admitted_in_smaller_foreground_batches(self):
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
        self.assertIn("nativeCapturesRemaining = 2;", paint)
        self.assertIn("FirstPersonFrameBudget captureScanBudget(", paint)
        self.assertIn("256, std::chrono::microseconds(1200)", paint)
        self.assertIn("nativeCapturesRemaining != 0 && captureScanBudget.take()", paint)
        self.assertIn("captureCursor += captureExamined;", paint)
        self.assertIn("scannedTiles=%zu", paint)
        self.assertIn("FP_NATIVE_STALL", paint)
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


if __name__ == "__main__":
    unittest.main()
