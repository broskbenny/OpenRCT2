/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonRenderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonSurfaceRelationshipKind : uint8_t
    {
        unresolved,
        internalOccupancyInterface,
        nativeOrderedLayer,
        semanticArtworkComposition,
        sameOwnerArtworkCarrier,
        equivalentPhysicalSurface,
        mutuallyExclusiveExterior,
    };

    struct FirstPersonCanonicalSurfaceQuad
    {
        std::array<std::array<int64_t, 3>, 4> points{};

        [[nodiscard]] bool operator==(
            const FirstPersonCanonicalSurfaceQuad&) const = default;
    };

    struct FirstPersonCanonicalSurfaceQuadHash
    {
        [[nodiscard]] size_t operator()(
            const FirstPersonCanonicalSurfaceQuad& key) const
        {
            uint64_t hash = 14695981039346656037ull;
            for (const auto& point : key.points)
            for (const auto coordinate : point)
            {
                hash ^= uint64_t(coordinate);
                hash *= 1099511628211ull;
            }
            return size_t(hash);
        }
    };

    [[nodiscard]] inline std::optional<
        FirstPersonCanonicalSurfaceQuad>
        FirstPersonCanonicalQuadForSurface(
            const FirstPersonSurface& surface)
    {
        constexpr double kQuantization = 256.0;
        std::array<std::array<int64_t, 3>, 4> unique{};
        size_t count = 0;
        for (const auto& vertex : surface.triangles)
        {
            const std::array<int64_t, 3> point{ {
                int64_t(std::llround(
                    double(vertex.world.x) * kQuantization)),
                int64_t(std::llround(
                    double(vertex.world.y) * kQuantization)),
                int64_t(std::llround(
                    double(vertex.world.z) * kQuantization)),
            } };
            bool duplicate = false;
            for (size_t i = 0; i < count; ++i)
            {
                if (unique[i] == point)
                {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate)
                continue;
            if (count >= unique.size())
                return std::nullopt;
            unique[count++] = point;
        }
        if (count != unique.size())
            return std::nullopt;
        std::sort(unique.begin(), unique.end());
        return FirstPersonCanonicalSurfaceQuad{ unique };
    }

    [[nodiscard]] inline bool FirstPersonSurfacesShareExactQuad(
        const FirstPersonSurface& a,
        const FirstPersonSurface& b)
    {
        const auto qa = FirstPersonCanonicalQuadForSurface(a);
        const auto qb = FirstPersonCanonicalQuadForSurface(b);
        return qa.has_value() && qb.has_value()
            && *qa == *qb;
    }

    [[nodiscard]] inline uint64_t
        FirstPersonSurfaceRelationshipOwnerKey(
            const FirstPersonSurface& surface)
    {
        uint64_t source = 0;
        uint64_t domain = 0;
        if (surface.reconstructionGroup != 0)
        {
            source = surface.reconstructionGroup;
            domain = 1;
        }
        else if (surface.diagnosticSemanticGroup != 0)
        {
            source = surface.diagnosticSemanticGroup;
            domain = 2;
        }
        else if (surface.sourceInstanceKey != 0)
        {
            source = surface.sourceInstanceKey;
            domain = 3;
        }
        if (source == 0)
            return 0;

        uint64_t result = 14695981039346656037ull;
        result ^= domain;
        result *= 1099511628211ull;
        result ^= source;
        result *= 1099511628211ull;
        return result == 0 ? 1 : result;
    }

    [[nodiscard]] inline bool
        FirstPersonSurfaceIsReconstructedOccupancyBoundary(
            const FirstPersonSurface& surface)
    {
        return surface.cameraIndependent
            && surface.reconstructedOccupancyBoundary
            && !surface.artworkCarrier;
    }

    [[nodiscard]] inline FirstPersonSurfaceRelationshipKind
        ClassifyFirstPersonSurfaceRelationship(
            const FirstPersonSurface& a,
            const FirstPersonSurface& b)
    {
        const uint64_t ownerA =
            FirstPersonSurfaceRelationshipOwnerKey(a);
        const uint64_t ownerB =
            FirstPersonSurfaceRelationshipOwnerKey(b);
        const bool exactQuad =
            FirstPersonSurfacesShareExactQuad(a, b);
        const bool opposed =
            FpDot(a.outwardNormal, b.outwardNormal)
                <= -0.999f;

        if (exactQuad && opposed
            && ownerA != 0 && ownerB != 0
            && ownerA != ownerB
            && FirstPersonSurfaceIsReconstructedOccupancyBoundary(a)
            && FirstPersonSurfaceIsReconstructedOccupancyBoundary(b))
        {
            return FirstPersonSurfaceRelationshipKind::
                internalOccupancyInterface;
        }

        if (exactQuad
            && a.artworkCarrier && b.artworkCarrier
            && ownerA != 0 && ownerA == ownerB)
        {
            return FirstPersonSurfaceRelationshipKind::
                sameOwnerArtworkCarrier;
        }

        if (a.coplanarOwner && b.coplanarOwner
            && a.nativePaintOrdinal != 0
            && b.nativePaintOrdinal != 0
            && a.diagnosticSemanticGroup != 0
            && b.diagnosticSemanticGroup != 0
            && a.diagnosticSourceArtworkGroup != 0
            && a.diagnosticSourceArtworkGroup
                == b.diagnosticSourceArtworkGroup
            && a.diagnosticSourceTile.x
                == b.diagnosticSourceTile.x
            && a.diagnosticSourceTile.y
                == b.diagnosticSourceTile.y)
        {
            return FirstPersonSurfaceRelationshipKind::
                semanticArtworkComposition;
        }

        if ((a.coplanarOwner && a.nativePaintOrdinal != 0)
            || (b.coplanarOwner && b.nativePaintOrdinal != 0))
        {
            return FirstPersonSurfaceRelationshipKind::
                nativeOrderedLayer;
        }

        if (a.physicalCoverage && b.physicalCoverage
            && a.solidColour != 0
            && a.solidColour == b.solidColour)
        {
            return FirstPersonSurfaceRelationshipKind::
                equivalentPhysicalSurface;
        }

        if (a.exteriorOnly && b.exteriorOnly && opposed)
        {
            return FirstPersonSurfaceRelationshipKind::
                mutuallyExclusiveExterior;
        }

        return FirstPersonSurfaceRelationshipKind::unresolved;
    }

    [[nodiscard]] inline bool
        FirstPersonSurfaceRelationshipExplainsCoplanarity(
            const FirstPersonSurface& a,
            const FirstPersonSurface& b)
    {
        return ClassifyFirstPersonSurfaceRelationship(a, b)
            != FirstPersonSurfaceRelationshipKind::unresolved;
    }

    struct FirstPersonInternalInterfaceResolution
    {
        size_t interfaces = 0;
        size_t suppressedFaces = 0;
        size_t contextMatches = 0;
    };

    [[nodiscard]] inline FirstPersonInternalInterfaceResolution
        ResolveFirstPersonInternalOccupancyInterfaces(
            std::vector<FirstPersonSurface>& surfaces,
            std::span<const FirstPersonSurface> context = {})
    {
        struct Entry
        {
            const FirstPersonSurface* surface = nullptr;
            size_t localIndex = 0;
            bool local = false;
        };

        std::unordered_map<
            FirstPersonCanonicalSurfaceQuad,
            std::vector<Entry>,
            FirstPersonCanonicalSurfaceQuadHash>
            buckets;
        buckets.reserve(
            (surfaces.size() + context.size()) / 2 + 1);

        const auto append =
            [&](const FirstPersonSurface& surface,
                size_t localIndex, bool local) {
                if (!FirstPersonSurfaceIsReconstructedOccupancyBoundary(
                        surface)
                    || FirstPersonSurfaceRelationshipOwnerKey(surface) == 0)
                    return;
                const auto quad =
                    FirstPersonCanonicalQuadForSurface(surface);
                if (!quad.has_value())
                    return;
                buckets[*quad].push_back({
                    &surface, localIndex, local
                });
            };

        for (size_t i = 0; i < surfaces.size(); ++i)
            append(surfaces[i], i, true);
        for (size_t i = 0; i < context.size(); ++i)
            append(context[i], i, false);

        std::vector<uint8_t> suppress(
            surfaces.size(), 0);
        FirstPersonInternalInterfaceResolution result{};
        for (const auto& [quad, entries] : buckets)
        {
            (void)quad;
            for (size_t i = 0; i < entries.size(); ++i)
            for (size_t j = i + 1; j < entries.size(); ++j)
            {
                const auto& a = entries[i];
                const auto& b = entries[j];
                if (!a.local && !b.local)
                    continue;
                if (ClassifyFirstPersonSurfaceRelationship(
                        *a.surface, *b.surface)
                    != FirstPersonSurfaceRelationshipKind::
                        internalOccupancyInterface)
                    continue;

                ++result.interfaces;
                if (a.local)
                    suppress[a.localIndex] = 1;
                if (b.local)
                    suppress[b.localIndex] = 1;
                if (a.local != b.local)
                    ++result.contextMatches;
            }
        }

        const size_t before = surfaces.size();
        size_t source = 0;
        size_t target = 0;
        while (source < surfaces.size())
        {
            if (suppress[source] == 0)
            {
                if (target != source)
                    surfaces[target] =
                        std::move(surfaces[source]);
                ++target;
            }
            ++source;
        }
        surfaces.resize(target);
        result.suppressedFaces = before - target;
        return result;
    }
} // namespace OpenRCT2::Paint
