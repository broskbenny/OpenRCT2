"""Production incremental packet state and publication regression."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class IncrementalPacketBuildTest(unittest.TestCase):
    def test_real_cpp_cursor_restarts_and_waits_for_every_phase(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        program = r"""
#include "src/openrct2/paint/FirstPersonPacketBuildProgress.h"
#include <cassert>
using OpenRCT2::Paint::FirstPersonPacketBuildProgress;
int main() {
    FirstPersonPacketBuildProgress state{};
    assert(state.needsRestart(0,1));
    state.restart(0,1);
    assert(!state.needsRestart(0,1));
    state.nextTile=511;
    state.groupsAdded=true;
    state.relationshipsResolved=true;
    assert(!state.readyToPublish(1024));
    state.nextTile=1024;
    assert(state.readyToPublish(1024));
    assert(state.needsRestart(1,1));
    state.restart(1,1);
    assert(state.nextTile==0 && !state.groupsAdded && !state.relationshipsResolved);
    state.nextTile=1024;
    state.groupsAdded=true;
    assert(!state.readyToPublish(1024));
    state.relationshipsResolved=true;
    assert(state.readyToPublish(1024));
    assert(state.needsRestart(1,2));
    state.restart(1,2);
    assert(!state.readyToPublish(1024));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            src, exe = Path(folder)/"packet.cpp", Path(folder)/"packet"
            src.write_text(program, encoding="utf-8")
            compiled = subprocess.run([compiler,"-std=c++20","-I",str(ROOT),str(src),
                                       "-o",str(exe)],capture_output=True,text=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            run = subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)

    def test_cold_region_tiles_skip_unneeded_map_readiness_checks(self):
        source = (ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        build = source[source.index("[[nodiscard]] bool RebuildStaticRegionPacket"):
                       source.index("void SubmitVisibleStaticRegions")]
        tiles = build[build.index("const auto tileKey = TerrainKey(tx, ty);"):
                      build.index("if (!staging.progress.tilesDone(kRegionTileCount))")]
        self.assertEqual(tiles.count("_staticPaintCache.find(tileKey)"), 1)
        before_map = tiles[:tiles.index("FirstPersonLocalFlatPathReady(")]
        self.assertIn("if (cached.semanticResidentSurfaces.empty()", before_map)
        self.assertIn("cached.cameraIndependentResidentSurfaces.empty()", before_map)
        self.assertIn("std::none_of(cached.rotations.begin(), cached.rotations.end()", before_map)
        self.assertIn("return variant.valid && !variant.residentSurfaces.empty();", before_map)
        self.assertLess(tiles.index("!cacheIt->second.valid || cacheIt->second.dirty"),
                        tiles.index("FirstPersonLocalFlatPathReady("))
        self.assertLess(tiles.index("FirstPersonLocalFlatPathReady("),
                        tiles.index("while (staging.tileCursor.phase < 6)"))
        self.assertIn("if (cached.animated && phase != 0)", tiles)
        self.assertIn("surfaces = &cached.semanticResidentSurfaces;", tiles)
        self.assertIn("surfaces = &cached.cameraIndependentResidentSurfaces;", tiles)
        self.assertIn("surfaces = &cached.rotations[phase - 2].residentSurfaces;", tiles)
        self.assertIn("ResumeFirstPersonPacketSurfaces(", tiles)
        # Geometry already valid for animated semantic items must not vanish.
        self.assertNotIn("|| cacheIt->second.animated)", tiles)

    def test_physical_group_meshes_do_not_restart_on_camera_turns(self):
        world = (ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        build = world[world.index("[[nodiscard]] bool RebuildStaticRegionPacket"):
                      world.index("void SubmitVisibleStaticRegions")]
        group_setup = build[build.index("const auto gatherGroups ="):
                            build.index("staging.groupsInitialized = true;")]
        # Compiled geometry does not need a source rotation. Native fallback
        # surfaces do, and must still guard worker completion.
        self.assertNotIn("staging.selectedGroups.emplace(", group_setup)
        self.assertIn("staging.selectedGroups.emplace(\n", build)
        self.assertIn("staging.selectedGroups.emplace(\n"
                      "                                    surface.reconstructionGroup, captured);", build)
        self.assertIn("FirstPersonStagedPacketSelectionChanged(pending->second)", build)

    def test_renderer_stages_and_publishes_only_complete_packet(self):
        source=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        core=(ROOT/"src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        renderer=(ROOT/"src/openrct2/paint/FirstPersonRenderer.cpp").read_text()
        build=source[source.index("[[nodiscard]] bool RebuildStaticRegionPacket"):
                     source.index("void SubmitVisibleStaticRegions")]
        self.assertIn("while (!staging.progress.tilesDone(kRegionTileCount)",build)
        self.assertIn("staging.progress.groupsAdded = true;",build)
        self.assertIn("staging.progress.relationshipsResolved = true;",build)
        self.assertIn("published = std::move(packet);",build)
        self.assertIn("packet.generation = published.generation + 1;",build)
        self.assertIn("staging.boundaryCandidateCount = size_t(std::count_if(",build)
        self.assertIn("if (staging.boundaryCandidateCount == 0)",build)
        self.assertIn("staging.boundaryCandidateCount >= 16",build)
        self.assertIn("published.surfaceStorage = staging.provisionalSurfaces;",build)
        self.assertIn("input->source = staging.provisionalSurfaces;",build)
        self.assertIn("input->surfaces = *input->source;",build)
        self.assertIn("input->context = std::move(relationshipContext);",build)
        self.assertIn("published.dirty = true;",build)
        self.assertLess(build.index("published.surfaceStorage = staging.provisionalSurfaces;"),
                        build.index("_regionRelationshipWorker.submit("))
        self.assertIn("const bool havePublishedPacket =",build)
        self.assertIn("staging.initialSourceRevision = published.sourceRevision;",build)
        self.assertIn("const bool missedSourceChanges =",build)
        self.assertIn("packet.dirty = missedSourceChanges;",build)
        self.assertIn("packet.publishedSourceRevision = staging.initialSourceRevision;",build)
        self.assertIn("staging.progress.invalidationSerial != published.invalidationSerial",build)
        self.assertLess(build.index("staging.progress.relationshipsResolved = true;"),build.index("published = std::move(packet);"))
        self.assertIn("published.invalidationSerial",build)
        self.assertIn("staging.boundaryCandidateCount >= 16",build)
        self.assertIn("staging.relationshipSubmitted = true;",build)
        self.assertIn("pending->second.progress.relationshipsResolved = true;",build)
        self.assertIn("_sceneEpoch != epoch",build)
        self.assertIn("pending->second.buildToken != token",build)
        self.assertIn("staging.buildToken = ++_nextStaticPacketBuildToken",build)
        self.assertIn("MarkStaticRegionPacketDirty(uint64_t key)",core)
        self.assertIn("_staticRegionPacketBuilds.clear();",renderer)


if __name__=="__main__":
    unittest.main()
