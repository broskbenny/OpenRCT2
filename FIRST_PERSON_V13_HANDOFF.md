# OpenRCT2 First-Person v13 Developer Handoff

## 1. Purpose and status

This branch implements a first-person walking renderer and ride-attached POV for OpenRCT2.

Repository: `broskbenny/OpenRCT2`

Branch: `first-person/v13`

Pinned upstream/base commit: `a8f6922e117249584d3c674656110386bfd74be5`

Known-good local source HEAD before this documentation commit: `ad1c57896020b526b6ad901f0abfa0770201559c`

**Verified status:**

* First-person walking has been manually tested and works.
* Ride-attached first-person POV has been manually tested and works.
* This was tested on a real Windows 7 Ultimate SP1 x64 machine.
* VxKex was NOT required to run the tested OpenRCT2 executable.
* The OpenGL drawing engine is required for first-person mode.

This documentation commit records that known-good state. It is **not** itself a stable release tag; tagging happens later.

## 2. First-person v13 architecture

Important v13 design decisions:

* Camera/ride orientation uses native OpenRCT2 vehicle geometry instead of a handcrafted pitch lookup.
* Ride audio listener semantics are attached to the simulation vehicle rather than presentation-time tweening.
* Mouse-look input is consumed at presentation/render rate rather than being quantised to the 40 Hz simulation tick.
* Transparency peeling is bounded to six exact layers plus a nearest-remaining fallback.
* The previous synchronous coverage-query wait was removed.
* Saturation of first-person audio effect tracking no longer steals/stops the oldest tracked voice; a new effect may continue untracked.
* Walking collision uses actual map semantics where reliable: terrain/path height, walls, and conservative large-scenery occupancy.
* Ride POV uses interpolated visual vehicle orientation for rendering while audio remains tied to simulation-time vehicle semantics.

Key added components:

* `src/openrct2-ui/FirstPersonController.cpp` / `.h`
* `src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp` / `.h`
* `src/openrct2/paint/FirstPersonRenderer.cpp` / `.h`
* `src/openrct2/paint/FirstPersonMath.h`
* `src/openrct2/paint/FirstPersonStreaming.h`
* `src/openrct2/paint/FirstPersonVehiclePose.h`
* `src/openrct2/audio/FirstPersonSpatialAudio.h`
* First-person renderer/audio tests under `test/tests`

## 3. User controls

### Walking mode

* Requires OpenGL renderer.
* View menu → Walking.
* In Swedish UI the reused string currently appears as `Går`.
* Mouse = look.
* W/S or Up/Down = forward/backward.
* A/D or Left/Right = strafe.
* Shift = faster movement.
* Escape = exit first-person.

### Ride POV

* Open a ride window.
* Main page has a POV control on the right side.
* POV attaches to a ride vehicle.
* Mouse = independent passenger head look.
* R = recenter head orientation.
* Escape = exit POV.

## 4. Actual Windows 7 test environment

* Actual host: Windows 7 Ultimate SP1 x64.
* Actual OS version: 6.1.7601.
* PowerShell 2.0.
* .NET Framework 4.8.
* SHA-2 support updates installed.
* KB4490628 installed.
* KB3140245 installed.
* TLS 1.2 support was configured for WinHTTP/.NET.
* Git for Windows 2.46.2.
* Visual Studio Build Tools 2019 16.11.
* MSVC v142, compiler 19.29.30147.
* Windows SDK 10.0.17763.0.
* MSBuild 16.11.

**Important warning about VxKex:**

VxKex is installed on the machine and can cause `ver` to falsely report:

```text
Microsoft Windows [Version 10.0.2660]
```

That spoofed `ver` output must **NOT** be used to conclude that the tested host is Windows 10. The actual target/test host is Windows 7 SP1 6.1.7601.

VxKex was not enabled/required for the successful OpenRCT2 runtime test.

## 5. VS2019 QuickJS compatibility commit

Commit:

* `ad1c57896020b526b6ad901f0abfa0770201559c`
* Subject: `Support QuickJS build with VS2019`

Bundled QuickJS expected C11 atomics support that is unavailable in MSVC 19.29.

The compatibility change in `src/thirdparty/quickjs-ng/quickjs-amalgam.c` does the following for `_MSC_VER < 1935`:

* defines `__STDC_NO_ATOMICS__`
* neutralises `_Atomic`
* avoids `<stdatomic.h>`
* disables QuickJS threading/worker atomics for that old compiler

This change was not merely theoretical: it was present in the source that successfully compiled and ran first-person mode on the target Win7 system.

## 6. Required compiler options

```bat
set "OPENRCT2_CL_ADDITIONALOPTIONS=/D_SILENCE_CXX20_U8PATH_DEPRECATION_WARNING /wd4267 /U__ENABLE_DISCORD__"
```

* `/D_SILENCE_CXX20_U8PATH_DEPRECATION_WARNING` suppresses an existing upstream filesystem deprecation warning promoted to error.
* `/wd4267` suppresses an existing upstream conversion warning promoted to error.
* `/U__ENABLE_DISCORD__` is required for this VS2019 build because the prebuilt `discord-rpc.lib` references a newer Microsoft STL implementation symbol unavailable in MSVC 19.29.

Observed incompatible Discord symbol:

`_Cnd_timedwait_for_unchecked`

This does not mean Discord source itself is broken; it is a prebuilt binary / toolset compatibility issue.

## 7. Build commands that actually worked

Developer environment:

```bat
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
```

GUI executable:

```bat
msbuild src\openrct2-win\openrct2-win.vcxproj /t:Build /p:Configuration=Release /p:Platform=x64 /m:1 /p:BuildInParallel=false /p:CL_MPCount=1
```

CLI:

```bat
msbuild src\openrct2-cli\openrct2-cli.vcxproj /t:Build /p:Configuration=Release /p:Platform=x64 /m:1 /p:BuildInParallel=false /p:CL_MPCount=1
```

Unrestricted compiler/MSBuild parallelism previously hung with `cl.exe` workers no longer making progress. The single-worker build completed successfully.

## 8. Runtime asset staging

Commands needed after directly building the child projects:

```bat
msbuild openrct2.proj /t:CopyLanguageFiles /p:Platform=x64
```

```bat
msbuild openrct2.proj "/t:CopyShaders;CopyScenarioPatches" /p:Platform=x64
```

```bat
msbuild openrct2.proj "/t:BuildG2;BuildFonts;BuildTracks;BuildPalettes" /p:Configuration=Release /p:Platform=x64
```

Known successfully generated files:

* `bin\data\g2.dat` — 170650 bytes
* `bin\data\fonts.dat` — 70956 bytes
* `bin\data\tracks.dat` — 6213702 bytes
* `bin\data\palettes.dat` — 54421 bytes

A legitimate original RCT2 installation is still required separately. Proprietary RCT2 data is not part of this repository or handoff.

The tested machine used:

`C:\GOG Games\RollerCoaster Tycoon 2 Triple Thrill Pack`

Do not include or embed any proprietary game data.

## 9. Confirmed runtime compatibility

Actual observations:

* `bin\openrct2.exe --version` loaded natively and returned OpenRCT2 v0.5.5.
* Normal graphical title screen rendered.
* Title-screen animation remained healthy for at least 60 seconds.
* Mouse/UI interaction worked.
* Process exited normally with code 0.
* Audio was audibly heard by the user.
* First-person walking worked after switching the drawing engine to OpenGL.
* Ride-attached first-person POV worked without hassle.
* No first-person crash was observed during manual testing.
* VxKex was not needed.

Known elevation behaviour:

The Windows account currently launches processes elevated by default, so OpenRCT2 displays its standard warning:

`For security reasons, it is not recommended to run OpenRCT2 with elevated permissions.`

The Swedish localisation displayed:

`Av säkerhetsskäl är det inte rekommenderat att köra OpenRCT2 med extra behörigheter.`

Dismissing the warning allowed normal operation. This is not a first-person failure.

## 10. Audio observations

* The user directly confirmed that game audio was audible.
* Some runtime output contained repeated: `Unable to create audio source: Unsupported audio codec`
* Because audible sound was nevertheless working, this is recorded as a deferred investigation rather than a blocker.
* Do not infer “no audio” merely from those codec messages.

## 11. Known deferred issues / non-blockers

1. A few suspicious-looking wall elements were noticed during ordinary rendering.
   * This has not been proven to be a first-person regression.
   * The user explicitly chose not to pursue it before stabilising the working version.

2. Some audio sources report unsupported codec as described above.
   * Audible audio still works.

3. Whole-solution test linking under VS2019 is not currently clean because the prebuilt GoogleTest libraries reference newer Microsoft STL implementation symbols, including:
   * `__std_find_trivial_1`
   * `__std_search_1`
   * `__std_search_2`
   This does not prevent the actual game executable or CLI from building and running. A future effort could rebuild GoogleTest with the Win7/VS2019 toolset if whole-solution test execution on this host is desired.

4. Do not “fix” the VxKex-spoofed `ver` result by treating the machine as Windows 10.

## 12. Repository history / important commits

Pinned base:

* `a8f6922e117249584d3c674656110386bfd74be5`

First-person v13 bulk commit:

* `198873f69b7fe3d49c9a90a21225bd797b557a90`
* `Add first-person v13 refactor (except Ride.cpp)`

Ride.cpp follow-up:

* `af3a21e4da5a6e4319daeccf9207683ddf4ff50c`
* `Add files via upload`

VS2019 QuickJS compatibility:

* `ad1c57896020b526b6ad901f0abfa0770201559c`
* `Support QuickJS build with VS2019`

`Ride.cpp` was uploaded separately because it exceeded a previous remote-edit size restriction; it is part of the v13 state.

## 13. External known-good checkpoint

On the tested Win7 machine a self-contained checkpoint was created at:

`C:\Users\Admin\Desktop\openrct2-first-person-v13-win7-known-good-2026-09-26.zip`

This local archive is supplementary; future work must not depend on it being available.

Checkpoint ZIP:

* size: `122795642` bytes
* SHA-256: `4f334305a17edfbc067dac91cbbaa008c9d2400e48ebe031f7050f7354db3f15`

Working executable SHA-256:

`4bc440665ab716b471c0b3f1ba98781972c5d86592144d54816b433f95b3cbba`

Pre-commit local QuickJS patch SHA-256:

`24becb4c10f95c2032cc5a1198ec36c997ab10aec65f380beb82c39b14166aea`

The checkpoint contains no `g1.dat` or other proprietary RCT2 installation data.

## 14. Guidance for the next developer/agent

* Start from branch `first-person/v13` or the eventual stable v13 tag.
* Read this document before refactoring.
* Preserve the known-good Win7 path while making changes.
* Keep first-person rendering OpenGL-specific unless intentionally redesigning that dependency.
* Do not remove the QuickJS compatibility guard without replacing it with another proven VS2019 solution.
* Do not re-enable the prebuilt Discord dependency on this toolchain without addressing the STL binary incompatibility.
* Use single-worker compilation on this Win7 host unless parallel-build stability is independently re-established.
* Treat audio codec warnings and suspicious wall rendering as deferred investigations, not as evidence that the stable checkpoint failed.
* Verify both walking mode and ride POV after renderer/audio changes.
* Ride audio attachment and presentation rendering intentionally use different timing semantics; preserve that distinction unless there is a compelling, tested reason to change it.

This handoff represents a manually verified working checkpoint, not a claim that every edge case is bug-free.
