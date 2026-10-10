#!/usr/bin/env python3
"""Focused production material-bake regression runner; requires Python 3 and a C++20 compiler."""
from pathlib import Path
import argparse
import subprocess
import tempfile

parser = argparse.ArgumentParser(description="Compile and run the production small-scenery bake against synthetic native sprites.")
parser.add_argument("--compiler", default="g++")
parser.add_argument("--sanitize", action="store_true", help="Enable undefined-behaviour sanitizer")
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
paint=root/'src/openrct2/paint'
core=(paint/'FirstPersonRenderer.Core.inc').read_text()
def function(s, marker):
    start=s.index(marker); opening=s.index('{',start);level=1;end=opening+1
    while level:
        level += (s[end]=='{')-(s[end]=='}');end+=1
    return s[start:end]+'\n'
base = root
headerroot = root.as_posix()
out=f'''#include "{headerroot}/src/openrct2/paint/FirstPersonSmallSceneryCollision.h"
#include "src/openrct2/paint/FirstPersonRenderer.h"
#include "src/openrct2/paint/FirstPersonSmallSceneryAppearance.h"
#include "src/openrct2/paint/FirstPersonSpriteSnapshot.h"
#include "src/openrct2/paint/FirstPersonMaterialInference.h"
#include <iostream>
#include <cassert>
using namespace OpenRCT2;
using namespace OpenRCT2::Paint;
'''
out+=function((root/'src/openrct2/drawing/Drawing.Sprite.cpp').read_text(),'size_t G1CalculateDataSize(')
scenery=(root/'src/openrct2/world/Scenery.cpp').read_text();start=scenery.index('const CoordsXY SceneryQuadrantOffsets[]')
out+=scenery[start:scenery.index('};',start)+2]+'\nnamespace OpenRCT2 {\n'
out+=function((root/'src/openrct2/interface/Viewport.cpp').read_text(),'ScreenCoordsXY Translate3DTo2DWithZ(int32_t rotation, const CoordsXYZ& pos)')
source=(root/'src/openrct2/world/tile_element/SmallSceneryElement.cpp').read_text()
for signature in ['uint8_t SmallSceneryElement::getSceneryQuadrant() const','uint8_t SmallSceneryElement::getAge() const']:
    out+=function(source,signature)
source=(root/'src/openrct2/world/tile_element/TileElementBase.cpp').read_text()
for signature in ['Direction TileElementBase::getDirection() const','void TileElementBase::setDirection(Direction direction)', 'Direction TileElementBase::getDirectionWithOffset(uint8_t offset) const','uint8_t TileElementBase::getOccupiedQuadrants() const','void TileElementBase::setOccupiedQuadrants(uint8_t quadrants)']:
    out+=function(source,signature)
out+='}\nnamespace OpenRCT2::Paint {\n'
for marker in ['void EmitQuad(', 'inline void ExtendStableKey(', '[[nodiscard]] std::optional<std::vector<uint8_t>>\n            DecodeFirstPersonSpritePixels(const G1Element& g1)\n','[[nodiscard]] bool SameFirstPersonMaterialTemplate(\n            ImageId a, ImageId b)\n']:
    out+=function(core,marker)
body=(base/'src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc').read_text()
out+=body[:body.index('        struct SmallSceneryJob')]+'}\n'
out+=(Path(__file__).with_name('material_fixtures.inc')).read_text()
# Compile exact production bodies; no copied implementations or mock pixel
# decoder. Fail on a renamed/missing definition rather than silently testing a
# stale implementation. This focused harness does not replace the full build.
with tempfile.TemporaryDirectory(prefix="first-person-bake-", dir=Path.cwd()) as temporary:
    source = Path(temporary) / "material_bake.cpp"
    binary = Path(temporary) / "material_bake"
    source.write_text(out)
    command = [args.compiler, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
               "-Wno-unknown-pragmas", "-pedantic", "-I" + str(root),
               "-I" + str(root / "src/thirdparty"), str(source), "-o", str(binary)]
    if args.sanitize:
        command += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
    subprocess.run(command, check=True)
    subprocess.run([str(binary)], check=True)
