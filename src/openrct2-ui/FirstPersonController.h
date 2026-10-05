/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <openrct2/Identifiers.h>

namespace OpenRCT2::Drawing
{
    struct RenderTarget;
}

namespace OpenRCT2::Ui
{
    struct InputEvent;
}

namespace OpenRCT2::Ui::FirstPerson
{
    enum class Mode
    {
        off,
        walking,
        rideAttached,
    };

    [[nodiscard]] Mode GetMode();
    [[nodiscard]] bool IsActive();
    // Shared ownership gate for camera controls, shortcuts and native mouse tools.
    [[nodiscard]] bool HasInputFocus();
    bool HandleInput(const InputEvent& event);

    void ToggleWalking();
    bool EnterWalking();
    bool EnterRide(EntityId vehicleId);
    void Exit();

    void Update();
    void Render(Drawing::RenderTarget& rt);
} // namespace OpenRCT2::Ui::FirstPerson


