/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace OpenRCT2::Paint
{
    // Native artwork arrival may advance a region's general invalidation
    // serial each frame. Only a genuine map edit may discard an in-progress
    // last-known-good region. The first-ever packet can finish and publish
    // dirty even across edits; it has no old geometry to preserve.
    [[nodiscard]] inline bool FirstPersonStagedPacketNeedsSourceRestart(
        uint64_t stagedInvalidationSerial, bool havePublishedPacket,
        uint64_t stagedMapEditSerial, uint64_t liveMapEditSerial)
    {
        return stagedInvalidationSerial
                == std::numeric_limits<uint64_t>::max()
            || (havePublishedPacket
                && stagedMapEditSerial != liveMapEditSerial);
    }

    // Tracks independent stages without ever exposing an incomplete packet.
    struct FirstPersonPacketBuildProgress
    {
        uint64_t invalidationSerial = std::numeric_limits<uint64_t>::max();
        uint64_t selectionSerial = 0;
        size_t nextTile = 0;
        bool groupsAdded = false;
        bool relationshipsResolved = false;

        [[nodiscard]] bool needsRestart(uint64_t invalidation, uint64_t selection) const
        {
            return invalidationSerial != invalidation || selectionSerial != selection;
        }

        void restart(uint64_t invalidation, uint64_t selection)
        {
            *this = {};
            invalidationSerial = invalidation;
            selectionSerial = selection;
        }

        [[nodiscard]] bool tilesDone(size_t count) const { return nextTile == count; }
        [[nodiscard]] bool readyToPublish(size_t count) const
        {
            return tilesDone(count) && groupsAdded && relationshipsResolved;
        }
    };
} // namespace OpenRCT2::Paint
