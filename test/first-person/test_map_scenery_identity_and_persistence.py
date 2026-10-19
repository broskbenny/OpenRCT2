"""Map-based scenery ownership: authoritative identity and last-known-good transitions."""
import os
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class MapSceneryPersistenceTest(unittest.TestCase):
    def test_exact_identity_and_published_ownership_rules(self):
        local = LOCAL.read_text(encoding="utf-8")
        small = SMALL.read_text(encoding="utf-8")
        for marker in ("uint32_t(small.getEntryIndex())",
                       "uint32_t(small.getBaseZ())",
                       "FirstPersonLocalSmallSceneryInstanceKey(",
                       "old.sourceInstanceKey != instance",
                       "FirstPersonMapSmallSceneryPending(",
                       "previousCell->second.surfaces",
                       "carried.gpuRegion = owningRegion;"):
            self.assertIn(marker, local)
        for marker in ("return found == _smallSceneryJobs.end() || !found->second->ready;",
                       "FirstPersonMapSmallSceneryEligible(*entry, small, viewFlags)"):
            self.assertIn(marker, small)
        self.assertIn("packet.viewFlags != scene.options.viewFlags", local)
        self.assertIn("packet.trackDesignSaveMode != gTrackDesignSaveMode", local)

    def test_exact_cpp_identity_from_repository(self):
        compiler = os.environ.get("CXX") or shutil.which("clang++") or shutil.which("g++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        local = LOCAL.read_text(encoding="utf-8")
        start = local.index("        [[nodiscard]] uint64_t FirstPersonLocalSmallSceneryInstanceKey(")
        end = local.index("        void SubmitVisibleLocalSceneryRegions(", start)
        implementation = local[start:end]
        cpp = """
#include <cstdint>
#include <cassert>
#include <random>
struct SmallSceneryElement {
    uint32_t entry=0;
    int32_t z=0;
    uint32_t getEntryIndex() const { return entry; }
    int32_t getBaseZ() const { return z; }
};
void ExtendStableKey(uint64_t& k, uint64_t v) { k ^= v; k *= 1099511628211ULL; }
""" + implementation + """
int main() {
    SmallSceneryElement element{5,32};
    auto baseline=FirstPersonLocalSmallSceneryInstanceKey(10, 20, 3, element);
    for (uint32_t variant=1;variant<10000;++variant) {
        assert(FirstPersonLocalSmallSceneryInstanceKey(10,20,3,element)==baseline);
        SmallSceneryElement identity{5+variant,32};
        assert(FirstPersonLocalSmallSceneryInstanceKey(10,20,3,identity)!=baseline);
        SmallSceneryElement height{5,32+int32_t(variant)};
        assert(FirstPersonLocalSmallSceneryInstanceKey(10,20,3,height)!=baseline);
        assert(FirstPersonLocalSmallSceneryInstanceKey(10+int(variant),20,3,element)!=baseline);
        assert(FirstPersonLocalSmallSceneryInstanceKey(10,20+int(variant),3,element)!=baseline);
        assert(FirstPersonLocalSmallSceneryInstanceKey(10,20,3+variant,element)!=baseline);
    }
}
"""
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "identity.cpp"
            source.write_text(cpp, encoding="utf-8")
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("identity.exe" if is_msvc else "identity")
            args = ([compiler, "/nologo", "/std:c++20", "/EHsc", str(source), "/Fe:" + str(exe)]
                    if is_msvc else [compiler, "-std=c++20", "-O0", str(source), "-o", str(exe)])
            build = subprocess.run(args, capture_output=True, text=True, cwd=folder)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True, cwd=folder)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_randomized_authoritative_replacement_policy(self):
        rng = random.Random(20261019)
        for _ in range(100000):
            old = (rng.randrange(40), rng.randrange(1000), rng.randrange(15),
                   rng.randrange(400), rng.randrange(400))
            current = list(old)
            changed = rng.random() < 0.5
            if changed:
                i = rng.randrange(len(current))
                current[i] += 1
            pending = rng.random() < 0.5
            ready_valid = rng.random() < 0.5
            visible_last_good = pending and not ready_valid and tuple(current) == old
            if changed:
                self.assertFalse(visible_last_good)
            if ready_valid:
                self.assertFalse(visible_last_good)


if __name__ == "__main__":
    unittest.main()
