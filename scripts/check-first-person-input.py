#!/usr/bin/env python3
"""Run headless first-person input routing regressions with a C++17 compiler.

Usage: python3 scripts/check-first-person-input.py [--compiler g++]

Compiles the actual routing/focus methods extracted from the UI sources against
small SDL/window test doubles. This exercises event ownership without linking
the renderer or opening a park. It does not replace a complete game build or an
interactive SDL test. No generated files are written into the source tree.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


def function(source, signature):
    start = source.index(signature)
    body = source.index("{", start)
    depth = 1
    end = body + 1
    while depth:
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
        end += 1
    return source[start:end]


PRELUDE = r"""
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <unordered_set>

enum SDL_bool { SDL_FALSE, SDL_TRUE };
using SDL_Keycode = int32_t;
enum SDL_Scancode { SDL_SCANCODE_ESCAPE, SDL_SCANCODE_E, SDL_SCANCODE_R, SDL_SCANCODE_W,
    SDL_SCANCODE_A, SDL_SCANCODE_S, SDL_SCANCODE_D, SDL_SCANCODE_UP,
    SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT,
    SDL_SCANCODE_LSHIFT, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_F10 };
constexpr int SDL_BUTTON_LEFT = 1;
constexpr uint32_t KMOD_CTRL = 1, KMOD_ALT = 2, KMOD_GUI = 4;
uint32_t modifiers = 0;
uint32_t SDL_GetModState() { return modifiers; }
SDL_bool relativeMode = SDL_FALSE;
bool captureFails = false;
int mouseDelta = 0;
SDL_bool SDL_GetRelativeMouseMode() { return relativeMode; }
int SDL_SetRelativeMouseMode(SDL_bool mode) {
    if (captureFails) return -1;
    relativeMode = mode; return 0;
}
int SDL_GetRelativeMouseState(int*, int*) { mouseDelta = 0; return 0; }
SDL_Scancode SDL_GetScancodeFromKey(SDL_Keycode key) {
    // An alternate-layout keycode still identifies the physical W key.
    return key == 1000 ? SDL_SCANCODE_W : static_cast<SDL_Scancode>(key);
}
bool appFocus = true;
bool ContextHasFocus() { return appFocus; }
enum class DrawingEngine { openGL, software };
struct Context {
    DrawingEngine engine = DrawingEngine::openGL;
    DrawingEngine GetDrawingEngineType() { return engine; }
} context;
Context* GetContext() { return &context; }
int walks = 0, looks = 0, inspections = 0, interactions = 0, exits = 0, rideAudio = 0;
int consoleInputs = 0, chatInputs = 0, dialogInputs = 0;

namespace OpenRCT2::Paint {
void RequestFirstPersonInspectorPick() { ++inspections; }
void RequestFirstPersonInteractionPick() { ++interactions; }
}
namespace OpenRCT2::Ui {
enum class InputDeviceKind { mouse, keyboard, joyButton, joyHat, joyAxis };
enum class InputEventState { down, release };
struct InputEvent {
    InputDeviceKind deviceKind; uint32_t modifiers; uint32_t button;
    InputEventState state; int16_t axisValue{};
};
enum class WindowClass { textinput, loadsaveOverwritePrompt, loadsave };
struct Window {};
struct WindowManager {
    Window window;
    std::array<bool, 3> open{};
    Window* FindByClass(WindowClass cls) {
        return open[static_cast<size_t>(cls)] ? &window : nullptr;
    }
} windowManager;
WindowManager* GetWindowManager() { return &windowManager; }
bool gChatOpen = false;
namespace Windows {
bool widgetText = false;
bool IsUsingWidgetTextBox() { return widgetText; }
void WindowTextInputKey(Window*, uint32_t) { ++dialogInputs; }
void WindowLoadSaveOverwritePromptInputKey(Window*, uint32_t) { ++dialogInputs; }
void WindowLoadSaveInputKey(Window*, uint32_t) { ++dialogInputs; }
}
namespace ShortcutId { constexpr int kDebugToggleConsole = 1; }
struct Console { bool open = false; bool IsOpen() { return open; } } console;
Console& GetInGameConsole() { return console; }
struct Shortcuts {
    bool pending = false; int actions = 0;
    bool isPendingShortcutChange() { return pending; }
    void processEvent(const InputEvent& event) {
        if (event.state == InputEventState::down) ++actions;
    }
    bool processEventForSpecificShortcut(const InputEvent&, int) { return false; }
} shortcuts;
Shortcuts& GetShortcutManager() { return shortcuts; }
struct InputManager {
    std::unordered_set<uint32_t> _firstPersonKeys;
    bool hasUiInputFocus() const;
    void process(const InputEvent&);
    void processInGameConsole(const InputEvent&) { ++consoleInputs; }
    void processChat(const InputEvent&) { ++chatInputs; }
} inputManager;
InputManager& GetInputManager() { return inputManager; }
namespace FirstPerson {
enum class Mode { off, walking, rideAttached };
struct State {
    Mode mode = Mode::off;
    float headYaw = 0, headPitch = 0;
    std::chrono::steady_clock::time_point lastUpdate{};
    SDL_bool previousRelativeMouseMode = SDL_FALSE;
    bool ownsRelativeMouseMode = false;
    bool nativeUiInputSuspended = false;
} _state;
bool IsActive();
bool HasInputFocus();
void ReleaseMouse();
void Exit() { ReleaseMouse(); _state.mode = Mode::off; ++exits; }
void PublishRideAudioAttachment() { ++rideAudio; }
void PublishAudioListener() {}
void PublishTweenView() {}
void UpdateMouseLook(bool) { ++looks; mouseDelta = 0; }
void UpdateWalking() { ++walks; }
void UpdateRideAttached() {}
void ProcessPendingFirstPersonInteraction() {}
"""

MOUSE = r"""
namespace OpenRCT2 {
using namespace Ui;
struct ScreenCoordsXY { int x{}, y{}; };
enum class MouseState { released, leftPress, leftRelease, rightPress, rightRelease };
enum class InputState { normal };
enum class InputFlag { widgetPressed, leftMousePressed, rightMousePressed };
struct Flags {
    void unset(InputFlag) {}
    bool has(InputFlag) { return false; }
} gInputFlags;
void InputSetState(InputState) {}
struct RCTMouseData { int x{}, y{}; MouseState state{}; };
RCTMouseData _mouseInputQueue[64];
uint8_t _mouseInputQueueReadIndex = 0, _mouseInputQueueWriteIndex = 0;
int nativeClicks = 0, nativeHover = 0, nativeTools = 0;
void InvalidateAllWindowsAfterInput() {}
MouseState GameGetNextInput(ScreenCoordsXY& point) {
    if (_mouseInputQueueReadIndex == _mouseInputQueueWriteIndex)
        return MouseState::released;
    const auto entry = _mouseInputQueue[_mouseInputQueueReadIndex];
    _mouseInputQueueReadIndex = (_mouseInputQueueReadIndex + 1) % std::size(_mouseInputQueue);
    point = {entry.x, entry.y}; return entry.state;
}
void GameHandleInputMouse(const ScreenCoordsXY&, MouseState state) {
    if (state == MouseState::leftPress) ++nativeClicks;
}
void ProcessMouseOver(const ScreenCoordsXY&) { ++nativeHover; }
void ProcessMouseTool(const ScreenCoordsXY&) { ++nativeTools; }
int ContextGetWidth() { return 1024; }
int ContextGetHeight() { return 768; }
"""

TESTS = r"""
using namespace OpenRCT2;
using namespace OpenRCT2::Ui;
using namespace OpenRCT2::Ui::FirstPerson;
int main() {
    auto key = [](uint32_t code, InputEventState state = InputEventState::down) {
        inputManager.process({InputDeviceKind::keyboard, 0, code, state});
    };
    auto click = [](InputEventState state) {
        inputManager.process({InputDeviceKind::mouse, 0, SDL_BUTTON_LEFT, state});
    };
    _state.mode = Mode::walking;
    for (auto code : {SDL_SCANCODE_W, SDL_SCANCODE_A, SDL_SCANCODE_S, SDL_SCANCODE_D,
         SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT,
         SDL_SCANCODE_LSHIFT, SDL_SCANCODE_RSHIFT}) {
        key(code); key(code, InputEventState::release);
    }
    key(1000); key(1000, InputEventState::release);
    assert(shortcuts.actions == 0);
    key(SDL_SCANCODE_E);
    key(SDL_SCANCODE_E);
    key(SDL_SCANCODE_E, InputEventState::release);
    assert(interactions == 1 && shortcuts.actions == 0);
    // Ctrl+S remains a screenshot command and cannot also walk backwards.
    modifiers = KMOD_CTRL;
    inputManager.process({InputDeviceKind::keyboard, modifiers, SDL_SCANCODE_S, InputEventState::down});
    inputManager.process({InputDeviceKind::keyboard, modifiers, SDL_SCANCODE_S, InputEventState::release});
    Update();
    assert(shortcuts.actions == 1 && walks == 0);
    modifiers = 0; shortcuts.actions = 0;
    key(SDL_SCANCODE_F10);
    assert(shortcuts.actions == 1); // Unrelated shortcut remains usable.

    // A complete click between presentation frames is still one inspection.
    click(InputEventState::down); click(InputEventState::release);
    assert(inspections == 1 && shortcuts.actions == 1);
    StoreMouseInput(MouseState::leftPress, {10, 10});
    assert(_mouseInputQueueReadIndex == _mouseInputQueueWriteIndex);
    GameHandleInput();
    assert(nativeClicks == 0 && nativeHover == 0 && nativeTools == 0);

    CaptureMouse();
    assert(relativeMode == SDL_TRUE);
    console.open = true;
    key(SDL_SCANCODE_W);
    key(SDL_SCANCODE_ESCAPE);
    mouseDelta = 70;
    Update(); UpdatePresentationInput();
    assert(consoleInputs == 2 && IsActive());
    assert(walks == 0 && looks == 0 && relativeMode == SDL_FALSE && mouseDelta == 0);
    console.open = false;

    for (auto cls : {WindowClass::textinput, WindowClass::loadsave,
                    WindowClass::loadsaveOverwritePrompt}) {
        windowManager.open[static_cast<size_t>(cls)] = true;
        assert(!HasInputFocus());
        key(SDL_SCANCODE_W, InputEventState::release);
        Update(); UpdatePresentationInput();
        windowManager.open[static_cast<size_t>(cls)] = false;
    }
    assert(dialogInputs == 3 && walks == 0 && looks == 0);
    gChatOpen = true; key(SDL_SCANCODE_W); Update();
    assert(chatInputs == 1 && walks == 0);
    gChatOpen = false;
    Windows::widgetText = true; key(SDL_SCANCODE_W); Update();
    assert(shortcuts.actions == 1 && walks == 0);
    Windows::widgetText = false;
    shortcuts.pending = true; key(SDL_SCANCODE_W); Update();
    assert(shortcuts.actions == 2 && walks == 0);
    shortcuts.pending = false;

    appFocus = false; Update(); UpdatePresentationInput();
    assert(walks == 0 && looks == 0);
    appFocus = true;
    _state.nativeUiInputSuspended = true;
    ReleaseMouse();
    assert(!HasInputFocus() && relativeMode == SDL_FALSE);
    assert(ResumeInputFromMainViewport());
    assert(HasInputFocus() && relativeMode == SDL_TRUE);
    assert(!ResumeInputFromMainViewport());
    ReleaseMouse();
    _state.lastUpdate = std::chrono::steady_clock::now() - std::chrono::hours(1);
    mouseDelta = 999;
    assert(UpdateInputCapture());
    assert(relativeMode == SDL_TRUE && mouseDelta == 0 && DeltaSeconds() < 0.05f);
    Update(); UpdatePresentationInput();
    assert(walks == 1 && looks == 1);

    _state.mode = Mode::rideAttached;
    _state.headYaw = _state.headPitch = 1.0f;
    key(SDL_SCANCODE_R); key(SDL_SCANCODE_R, InputEventState::release);
    assert(_state.headYaw == 0.0f && _state.headPitch == 0.0f && rideAudio == 1);
    assert(shortcuts.actions == 2);
    console.open = true;
    _state.headYaw = 1.0f; key(SDL_SCANCODE_R); UpdatePresentationInput();
    assert(_state.headYaw == 1.0f);
    console.open = false;

    // Escape is handled even if pressed and released before the next update.
    key(SDL_SCANCODE_ESCAPE); key(SDL_SCANCODE_ESCAPE); // OS key repeat after exit.
    key(SDL_SCANCODE_ESCAPE, InputEventState::release);
    assert(!IsActive() && exits == 1 && shortcuts.actions == 2);
    key(SDL_SCANCODE_W); click(InputEventState::down);
    assert(shortcuts.actions == 4 && inspections == 1);
    StoreMouseInput(MouseState::leftPress, {10, 10}); GameHandleInput();
    assert(nativeClicks == 1 && nativeHover == 1 && nativeTools == 1);

    // A queued native click from entry must not replay when POV exits.
    StoreMouseInput(MouseState::leftPress, {10, 10});
    _state.mode = Mode::walking; GameHandleInput(); Exit(); GameHandleInput();
    assert(nativeClicks == 1);

    // Failed mouse capture does not claim ownership of SDL state.
    _state.mode = Mode::walking; captureFails = true; CaptureMouse();
    assert(!_state.ownsRelativeMouseMode);
    captureFails = false; CaptureMouse();
    assert(_state.ownsRelativeMouseMode); ReleaseMouse();
    assert(relativeMode == SDL_FALSE);
    relativeMode = SDL_TRUE; CaptureMouse(); ReleaseMouse();
    assert(relativeMode == SDL_TRUE); // Restore a pre-existing relative mode.
    context.engine = DrawingEngine::software; Update();
    assert(!IsActive());
    std::cout << "First-person input routing and focus regressions passed\n";
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    ui = root / "src/openrct2-ui"
    controller = (ui / "FirstPersonController.cpp").read_text(encoding="utf-8")
    manager = (ui / "input/InputManager.cpp").read_text(encoding="utf-8")
    mouse = (ui / "input/MouseInput.cpp").read_text(encoding="utf-8")
    signatures = [
        "bool IsActive()", "bool HasInputFocus()", "void CaptureMouse()", "void ReleaseMouse()",
        "bool UpdateInputCapture()", "bool ResumeInputFromMainViewport()",
        "bool HandleInput(const InputEvent& event)", "float DeltaSeconds()",
        "void UpdatePresentationInput()", "void Update()",
    ]
    source = PRELUDE + "\n".join(function(controller, s) for s in signatures) + "\n}}\n"
    source += "using namespace OpenRCT2::Ui;\n"
    source += function(manager, "bool InputManager::hasUiInputFocus() const") + "\n"
    source += function(manager, "void InputManager::process(const InputEvent& e)") + "\n"
    source += MOUSE
    source += function(mouse, "void StoreMouseInput(MouseState state, const ScreenCoordsXY& screenCoords)") + "\n"
    source += function(mouse, "void GameHandleInput()") + "\n}\n" + TESTS
    with tempfile.TemporaryDirectory(prefix="openrct2-fp-input-") as temporary:
        source_path = Path(temporary) / "input.cpp"
        binary_path = Path(temporary) / "input-tests"
        source_path.write_text(source, encoding="utf-8")
        subprocess.run([args.compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        str(source_path), "-o", str(binary_path)], check=True)
        subprocess.run([str(binary_path)], check=True)


if __name__ == "__main__":
    main()
