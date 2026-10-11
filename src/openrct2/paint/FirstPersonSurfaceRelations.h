/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonRenderer.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
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
            uint64_t owner = 0;
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
                if (!FirstPersonSurfaceIsReconstructedOccupancyBoundary(surface))
                    return;
                const uint64_t owner = FirstPersonSurfaceRelationshipOwnerKey(surface);
                if (owner == 0)
                    return;
                const auto quad =
                    FirstPersonCanonicalQuadForSurface(surface);
                if (!quad.has_value())
                    return;
                buckets[*quad].push_back({
                    &surface, localIndex, local, owner
                });
            };

        for (size_t i = 0; i < surfaces.size(); ++i)
            append(surfaces[i], i, true);
        for (size_t i = 0; i < context.size(); ++i)
            append(context[i], i, false);

        std::vector<uint8_t> suppress(
            surfaces.size(), 0);
        FirstPersonInternalInterfaceResolution result{};
        struct Counts
        {
            size_t local = 0;
            size_t context = 0;
            size_t total() const { return local + context; }
        };
        struct Orientation
        {
            FirstPersonVec3 normal{};
            Counts count;
            std::unordered_map<uint64_t, Counts> owners;
            std::vector<Entry> entries;
        };
        while (!buckets.empty())
        {
            const auto bucket = buckets.begin();
            if (bucket->second.size() <= 8)
            {
                // Most interfaces have just two faces. Avoid constructing
                // orientation/owner maps for these ordinary contacts.
                const auto& entries = bucket->second;
                for (size_t i = 0; i < entries.size(); ++i)
                for (size_t j = i + 1; j < entries.size(); ++j)
                {
                    const auto& a = entries[i];
                    const auto& b = entries[j];
                    if ((!a.local && !b.local) || a.owner == b.owner
                        || !(FpDot(a.surface->outwardNormal, b.surface->outwardNormal) <= -0.999f))
                        continue;
                    ++result.interfaces;
                    if (a.local)
                        suppress[a.localIndex] = 1;
                    if (b.local)
                        suppress[b.localIndex] = 1;
                    if (a.local != b.local)
                        ++result.contextMatches;
                }
                buckets.erase(bucket);
                continue;
            }
            std::map<std::array<uint32_t, 3>, Orientation> orientations;
            for (const auto& entry : bucket->second)
            {
                const auto n = entry.surface->outwardNormal;
                // Use representation keys so non-finite normals never become
                // ordered floating-point keys. The dot predicate below retains
                // the classifier's exact semantics, including NaN rejection.
                const std::array<uint32_t, 3> normalKey{
                    std::bit_cast<uint32_t>(n.x), std::bit_cast<uint32_t>(n.y), std::bit_cast<uint32_t>(n.z)
                };
                auto& orientation = orientations[normalKey];
                orientation.normal = n;
                auto& owner = orientation.owners[entry.owner];
                if (entry.local)
                {
                    ++orientation.count.local;
                    ++owner.local;
                }
                else
                {
                    ++orientation.count.context;
                    ++owner.context;
                }
                orientation.entries.push_back(entry);
            }
            // A normal cannot oppose itself. Compare distinct orientations,
            // then account for all matching instances without repeating the
            // same geometric predicate for every pair of coincident faces.
            for (auto a = orientations.begin(); a != orientations.end(); ++a)
            for (auto b = std::next(a); b != orientations.end(); ++b)
            {
                if (!(FpDot(a->second.normal, b->second.normal) <= -0.999f))
                    continue;
                auto& left = a->second;
                auto& right = b->second;
                if (left.count.local == 0 && right.count.local == 0)
                    continue;
                size_t interfaces = left.count.local * right.count.total()
                    + left.count.context * right.count.local;
                size_t contextMatches = left.count.local * right.count.context
                    + left.count.context * right.count.local;
                // Identical orientations share the same dot-product decision.
                // Count distinct-owner pairs without comparing every instance.
                for (const auto& [owner, counts] : left.owners)
                {
                    const auto same = right.owners.find(owner);
                    if (same == right.owners.end())
                        continue;
                    interfaces -= counts.local * same->second.total() + counts.context * same->second.local;
                    contextMatches -= counts.local * same->second.context + counts.context * same->second.local;
                }
                result.interfaces += interfaces;
                result.contextMatches += contextMatches;
                for (const auto& entry : left.entries)
                {
                    if (!entry.local)
                        continue;
                    const auto same = right.owners.find(entry.owner);
                    if (same == right.owners.end() || same->second.total() < right.count.total())
                        suppress[entry.localIndex] = 1;
                }
                for (const auto& entry : right.entries)
                {
                    if (!entry.local)
                        continue;
                    const auto same = left.owners.find(entry.owner);
                    if (same == left.owners.end() || same->second.total() < left.count.total())
                        suppress[entry.localIndex] = 1;
                }
            }
            buckets.erase(bucket);
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
