/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAsync.h"
#include "../drawing/Drawing.Sprite.h"
#include <memory>
#include <unordered_map>

namespace OpenRCT2::Paint
{
    struct FirstPersonOwnedSprite
    {
        G1Element header{};
        std::vector<uint8_t> bytes;
    };
    // Captured on the game thread. A job receives this by value and only uses
    // get(); no sprite registry, object manager, tile or PaintStruct pointers.
    class FirstPersonSpriteSnapshot
    {
        std::unordered_map<ImageIndex, std::shared_ptr<const FirstPersonOwnedSprite>> _sprites;
    public:
        const G1Element* get(ImageIndex index) const
        {
            const auto it = _sprites.find(index);
            return it == _sprites.end() || !it->second ? nullptr : &it->second->header;
        }
        const G1Element* get(ImageId image) const { return image.HasValue() ? get(image.GetIndex()) : nullptr; }
        bool capture(ImageIndex index, const G1Element* source, FirstPersonFrameBudget& budget)
        {
            if (_sprites.contains(index)) return true;
            if (!budget.take()) return false;
            if (source == nullptr || source->offset == nullptr || source->width <= 0 || source->height <= 0
                || source->width > 512 || source->height > 512 || source->flags.has(G1Flag::isPalette))
            {
                _sprites.emplace(index, nullptr);
                return true;
            }
            auto owned = std::make_shared<FirstPersonOwnedSprite>();
            owned->header = *source;
            const auto size = G1CalculateDataSize(source);
            owned->bytes.assign(source->offset, source->offset + size);
            owned->header.offset = owned->bytes.data();
            _sprites.emplace(index, std::move(owned));
            return true;
        }
        bool capture(ImageIndex index, FirstPersonFrameBudget& budget)
        {
            if (_sprites.contains(index)) return true;
            if (!budget.available()) return false;
            return capture(index, GfxGetG1Element(index), budget);
        }
        bool capture(ImageId image, FirstPersonFrameBudget& budget)
        {
            return !image.HasValue() || capture(image.GetIndex(), budget);
        }
    };
}
