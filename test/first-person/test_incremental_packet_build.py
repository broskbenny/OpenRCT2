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
        self.assertIn("const bool havePublishedPacket =",build)
        self.assertIn("staging.initialSourceRevision = published.sourceRevision;",build)
        self.assertIn("const bool missedSourceChanges =",build)
        self.assertIn("packet.dirty = missedSourceChanges;",build)
        self.assertIn("packet.publishedSourceRevision = staging.initialSourceRevision;",build)
        self.assertIn("staging.progress.invalidationSerial != published.invalidationSerial",build)
        self.assertLess(build.index("staging.progress.relationshipsResolved = true;"),build.index("published = std::move(packet);"))
        self.assertIn("published.invalidationSerial",build)
        self.assertIn("packet.surfaces.size() >= 256",build)
        self.assertIn("staging.relationshipSubmitted = true;",build)
        self.assertIn("pending->second.progress.relationshipsResolved = true;",build)
        self.assertIn("_sceneEpoch != epoch",build)
        self.assertIn("pending->second.buildToken != token",build)
        self.assertIn("staging.buildToken = ++_nextStaticPacketBuildToken",build)
        self.assertIn("MarkStaticRegionPacketDirty(uint64_t key)",core)
        self.assertIn("_staticRegionPacketBuilds.clear();",renderer)


if __name__=="__main__":
    unittest.main()
