/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace OpenRCT2::Ui
{
    // A global atlas-image invalidation does not imply that every static
    // region's source pixels changed. Keep exact dependency revisions and
    // consult them only when the global invalidation serial advances.
    class FirstPersonRegionTextureRevisions
    {
        uint64_t _checkedSerial = 0;
        std::vector<uint64_t> _revisions;
        bool _stale = false;

    public:
        template<typename Images, typename Revision>
        void capture(const Images& images, uint64_t serial, Revision&& revision)
        {
            _checkedSerial = serial;
            _stale = false;
            _revisions.clear();
            _revisions.reserve(images.size());
            for (const auto image : images)
                _revisions.push_back(revision(image));
        }

        template<typename Images, typename Revision>
        [[nodiscard]] bool current(const Images& images, uint64_t serial, Revision&& revision)
        {
            if (_stale)
                return false;
            if (_checkedSerial == serial)
                return true;
            if (_revisions.size() != images.size())
            {
                _stale = true;
                return false;
            }
            for (size_t i = 0; i < images.size(); ++i)
            {
                if (_revisions[i] != revision(images[i]))
                {
                    _stale = true;
                    return false;
                }
            }
            _checkedSerial = serial;
            return true;
        }
    };
} // namespace OpenRCT2::Ui
