"""Validate the EXACT OpenGL shaders embedded in the production first-person renderer.

A C++/Windows compilation does not compile shader strings. Run this in the
first-person Actions job before treating a GPU edit as release-candidate ready.
This is shader compilation/linkage validation, NOT a visual regression test.
"""

from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"
SHADERS = (
    ("kVertexShader", "kFragmentShader", "scene"),
    ("kComposeVertexShader", "kComposeFragmentShader", "composition"),
)
EXTRACT = re.compile(
    r'constexpr\s+char\s+(k\w+Shader)\[\]\s*=\s*R"GLSL\((.*?)\)GLSL";',
    re.DOTALL,
)


def validate() -> None:
    validator = shutil.which("glslangValidator")
    if validator is None:
        raise RuntimeError(
            "glslangValidator is required: install glslang-tools in the CI image"
        )
    code = SOURCE.read_text(encoding="utf-8")
    shaders = dict(EXTRACT.findall(code))
    if len(shaders) != 4:
        raise AssertionError(
            f"Expected four embedded production GLSL shaders, found {list(shaders)}"
        )
    with tempfile.TemporaryDirectory(prefix="fp-glsl-") as directory:
        for vertex_name, fragment_name, title in SHADERS:
            vertex = Path(directory) / f"{title}.vert"
            fragment = Path(directory) / f"{title}.frag"
            vertex.write_text(shaders[vertex_name], encoding="utf-8")
            fragment.write_text(shaders[fragment_name], encoding="utf-8")
            result = subprocess.run(
                [validator, "-l", str(vertex), str(fragment)],
                capture_output=True,
                text=True,
            )
            if result.returncode != 0:
                raise AssertionError(
                    f"{title} shader compile/link failed:\n"
                    + result.stdout + "\n" + result.stderr
                )
            print(f"GLSL 330 scene {title}: compile/link passed")


if __name__ == "__main__":
    validate()
