// Compiled inside the exact production small-scenery material-bake harness.
// The input contains independently decoded legacy G1 indices and offsets.
// No replacement renderer, special object names, or synthetic silhouettes.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_set>

namespace
{
    template<typename T>
    bool ReadExact(std::istream& input, T& value)
    {
        return bool(input.read(reinterpret_cast<char*>(&value), sizeof(value)));
    }

    bool ReadAsset(
        const char* filename, SmallSceneryEntry& entry,
        FirstPersonSpriteSnapshot& snapshot,
        std::array<std::vector<uint8_t>, 4>& sourcePixels)
    {
        std::ifstream input(filename, std::ios::binary);
        char magic[4]{};
        uint32_t flags = 0;
        uint32_t height = 0;
        if (!input.read(magic, sizeof(magic))
            || std::string(magic, sizeof(magic)) != "FPS1"
            || !ReadExact(input, flags) || !ReadExact(input, height)
            || height == 0 || height > 255)
            return false;

        entry = {};
        entry.image = 100;
        entry.height = uint8_t(height);
        for (int bit = 0; bit < 28; ++bit)
            if (flags & (1u << bit))
                entry.flags.set(static_cast<SmallSceneryFlag>(bit));
        // Match SmallSceneryObject::ReadLegacy: tall legacy objects are
        // classified as trees even when the file does not carry that new flag.
        if (entry.height > 64)
            entry.flags.set(SmallSceneryFlag::isTree);

        FirstPersonFrameBudget budget(4, std::chrono::seconds(10));
        for (uint32_t rotation = 0; rotation < 4; ++rotation)
        {
            int16_t width = 0, spriteHeight = 0, originX = 0, originY = 0;
            uint32_t count = 0;
            if (!ReadExact(input, width)
                || !ReadExact(input, spriteHeight)
                || !ReadExact(input, originX)
                || !ReadExact(input, originY)
                || !ReadExact(input, count)
                || width <= 0 || spriteHeight <= 0
                || width > 512 || spriteHeight > 512
                || count != uint32_t(width) * uint32_t(spriteHeight))
                return false;

            auto& pixels = sourcePixels[rotation];
            pixels.resize(count);
            if (!input.read(reinterpret_cast<char*>(pixels.data()), count))
                return false;
            G1Element sprite{};
            sprite.width = width;
            sprite.height = spriteHeight;
            sprite.xOffset = originX;
            sprite.yOffset = originY;
            sprite.flags.set(G1Flag::hasTransparency);
            sprite.offset = pixels.data();
            if (!snapshot.capture(entry.image + rotation, &sprite, budget))
                return false;
        }
        return input.peek() == std::char_traits<char>::eof();
    }

    // Export actual emitted triangles, not just the selected hull.
    // FPM1 is an internal, little-endian indexed-material diagnostic format:
    // magic, count, then repeated width/height/length/u8 palette indices.
    bool ExportEmittedScene(
        const FirstPersonScene& scene, const std::string& folder, int index)
    {
        std::ostringstream stem;
        stem << folder << "/asset_" << std::setfill('0') << std::setw(4)
             << index;
        std::ofstream obj(stem.str() + ".obj");
        std::ofstream materials(stem.str() + ".fpmat", std::ios::binary);
        if (!obj || !materials)
            return false;
        const char magic[4] = { 'F', 'P', 'M', '1' };
        const uint32_t count = uint32_t(scene.surfaces.size());
        materials.write(magic, sizeof(magic));
        materials.write(reinterpret_cast<const char*>(&count), sizeof(count));
        obj << "# Exact OpenRCT2 emitted geometry; indexed materials in paired FPM1 file\n";
        obj << std::fixed << std::setprecision(6);
        size_t vertex = 1;
        for (size_t i = 0; i < scene.surfaces.size(); ++i)
        {
            const auto& surface = scene.surfaces[i];
            const uint32_t width = surface.immutableWidth > 0
                ? uint32_t(surface.immutableWidth) : 0;
            const uint32_t height = surface.immutableHeight > 0
                ? uint32_t(surface.immutableHeight) : 0;
            const uint32_t countBytes =
                surface.persistentPixels != nullptr
                ? uint32_t(surface.persistentPixels->size()) : 0;
            materials.write(reinterpret_cast<const char*>(&width), sizeof(width));
            materials.write(reinterpret_cast<const char*>(&height), sizeof(height));
            materials.write(reinterpret_cast<const char*>(&countBytes), sizeof(countBytes));
            if (countBytes)
                materials.write(reinterpret_cast<const char*>(
                    surface.persistentPixels->data()), countBytes);
            obj << "g surface_" << i << "\n";
            for (const auto& point : surface.triangles)
                obj << "v " << point.world.x << " " << point.world.y
                    << " " << point.world.z << "\n";
            for (const auto& point : surface.triangles)
                obj << "vt " << (width ? point.u / float(width) : 0.0f)
                    << " " << (height ? 1.0f - point.v / float(height) : 0.0f)
                    << "\n";
            for (size_t tri = 0; tri < 2; ++tri)
            {
                const size_t base = vertex + tri * 3;
                obj << "f " << base << "/" << base
                    << " " << base + 1 << "/" << base + 1
                    << " " << base + 2 << "/" << base + 2 << "\n";
            }
            vertex += 6;
        }
        return bool(obj) && bool(materials);
    }

    void EvaluateRealSmallScenery(int index, const char* path,
                                  const std::string& exportFolder)
    {
        SmallSceneryEntry entry{};
        FirstPersonSpriteSnapshot snapshot;
        std::array<std::vector<uint8_t>, 4> sourcePixels;
        if (!ReadAsset(path, entry, snapshot, sourcePixels))
        {
            std::cout << "{\"index\":" << index
                      << ",\"input_valid\":false}" << std::endl;
            return;
        }

        SmallSceneryElement element{};
        element.setOccupiedQuadrants(15);
        element.setDirection(0);
        std::unordered_set<uint8_t> observedPalette;
        size_t nativeOpaquePixels = 0;
        for (const auto& pixels : sourcePixels)
            for (uint8_t pixel : pixels)
                if (pixel)
                {
                    ++nativeOpaquePixels;
                    observedPalette.insert(pixel);
                }

        FirstPersonScene scene;
        FirstPersonVisualHullAttempt attempt{};
        SmallSceneryBuildEvidence evidence{};
        std::optional<FirstPersonSmallSceneryWalkingMask> walking;
        const auto started = std::chrono::steady_clock::now();
        const bool built = BuildSmallSceneryGeometry(
            scene, element, entry, ImageId(entry.image), snapshot,
            &attempt, &evidence, &walking);
        const auto elapsedMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();

        // Evaluate the same geometry decision the production appearance plan
        // uses, with the same source offsets and optional registration.
        // This second invocation is outside the production-cost timer.
        const auto hull = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
            entry, element,
            [&](ImageIndex id) { return snapshot.get(id); });
        std::vector<FirstPersonVisualHullFace> actualFaces;
        if (hull.valid)
        {
            const auto appearance =
                BuildFirstPersonSmallSceneryAppearancePlan(
                    entry, element, &hull);
            if (appearance.valid())
                actualFaces = appearance.faces;
        }
        std::vector<FirstPersonVisualHullView> observations;
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const G1Element* sprite = snapshot.get(entry.image + rotation);
            if (!sprite)
                continue;
            FirstPersonVisualHullView view{};
            view.imageDirection = rotation;
            view.image = entry.image + rotation;
            view.g1 = sprite;
            AppendFirstPersonG1OpaqueSilhouette(*sprite, view.observed);
            observations.push_back(std::move(view));
        }
        const auto project = [&](uint8_t rotation, FirstPersonVec3 point) {
            const CoordsXYZ integerPoint{
                int32_t(std::lround(point.x)),
                int32_t(std::lround(point.y)),
                int32_t(std::lround(point.z))
            };
            const auto projected =
                FirstPersonSmallSceneryArtworkPoint(
                    entry, rotation, integerPoint);
            const auto offset =
                FirstPersonSmallSceneryPaintOffset(entry, element, rotation);
            const auto origin =
                Translate3DTo2DWithZ(rotation, { offset, 0 });
            const auto correction = hull.viewPixelOffsets[rotation];
            return std::array<float, 2>{
                float(projected.x - origin.x + correction.x),
                float(projected.y - origin.y + correction.y)
            };
        };
        FirstPersonRoundTripCertificate certificate{};
        if (!actualFaces.empty() && observations.size() == 4)
            certificate = CertifyFirstPersonReconstruction(
                TriangulateFirstPersonReconstructionFaces(actualFaces),
                observations, project);

        size_t bakedTexels = 0;
        size_t opaqueBakedTexels = 0;
        size_t offPaletteTexels = 0;
        size_t surfacesWithoutPixels = 0;
        size_t degenerateFaces = 0;
        size_t outOfBoundsVertices = 0;
        for (const auto& surface : scene.surfaces)
        {
            if (!surface.persistentPixels)
            {
                ++surfacesWithoutPixels;
                continue;
            }
            const auto& pixels = *surface.persistentPixels;
            bakedTexels += pixels.size();
            for (uint8_t pixel : pixels)
                if (pixel)
                {
                    ++opaqueBakedTexels;
                    if (!observedPalette.contains(pixel))
                        ++offPaletteTexels;
                }
        }
        for (const auto& face : actualFaces)
        {
            const auto a = face.corners[0];
            const auto b = face.corners[1];
            const auto d = face.corners[3];
            const auto ux = b.x - a.x, uy = b.y - a.y, uz = b.z - a.z;
            const auto vx = d.x - a.x, vy = d.y - a.y, vz = d.z - a.z;
            const auto cx = uy * vz - uz * vy;
            const auto cy = uz * vx - ux * vz;
            const auto cz = ux * vy - uy * vx;
            if (cx * cx + cy * cy + cz * cz < 1e-8f)
                ++degenerateFaces;
            for (const auto& corner : face.corners)
                if (!std::isfinite(corner.x)
                    || !std::isfinite(corner.y)
                    || !std::isfinite(corner.z)
                    || corner.x < -0.05f || corner.x > 32.05f
                    || corner.y < -0.05f || corner.y > 32.05f
                    || corner.z < -0.05f
                    || corner.z > entry.height + 0.05f)
                    ++outOfBoundsVertices;
        }

        if (!exportFolder.empty()
            && !ExportEmittedScene(scene, exportFolder, index))
            throw std::runtime_error("Unable to export production scene");

        std::cout << std::fixed << std::setprecision(6)
                  << "{\"index\":" << index
                  << ",\"input_valid\":true"
                  << ",\"built\":" << (built ? "true" : "false")
                  << ",\"hull_valid\":" << (hull.valid ? "true" : "false")
                  << ",\"eligible_for_native_wire\":"
                  << (entry.flags.hasAll(SmallSceneryFlag::occupiesFullTile,
                      SmallSceneryFlag::isRotatable) ? "true" : "false")
                  << ",\"height\":" << unsigned(entry.height)
                  << ",\"native_opaque_pixels\":" << nativeOpaquePixels
                  << ",\"native_palette_entries\":" << observedPalette.size()
                  << ",\"attempt_stage\":" << unsigned(attempt.stage)
                  << ",\"attempt_views\":" << unsigned(attempt.nativeViews)
                  << ",\"hull_faces\":" << evidence.hullFaces
                  << ",\"actual_faces\":" << actualFaces.size()
                  << ",\"rendered_surfaces\":" << scene.surfaces.size()
                  << ",\"material_faces\":" << evidence.materialFaces
                  << ",\"no_source_material\":" << evidence.withoutSourceMaterial
                  << ",\"no_opaque_samples\":" << evidence.withoutOpaqueSamples
                  << ",\"unobserved_undersides\":" << evidence.unobservedUndersides
                  << ",\"direct_convex\":" << (evidence.directConvexSurface ? "true" : "false")
                  << ",\"structural_beam\":" << (evidence.structuralBeamSelected ? "true" : "false")
                  << ",\"native_wire\":" << (evidence.sourceTriangulatedWireSelected ? "true" : "false")
                  << ",\"baked_texels\":" << bakedTexels
                  << ",\"opaque_baked_texels\":" << opaqueBakedTexels
                  << ",\"off_source_palette_texels\":" << offPaletteTexels
                  << ",\"untextured_surfaces\":" << surfacesWithoutPixels
                  << ",\"degenerate_faces\":" << degenerateFaces
                  << ",\"out_of_bounds_vertices\":" << outOfBoundsVertices
                  << ",\"production_elapsed_ms\":" << elapsedMs
                  << ",\"round_trip_valid\":" << (certificate.valid ? "true" : "false")
                  << ",\"four_view_min_iou\":" << certificate.minimumIntersectionOverUnion
                  << ",\"four_view_min_candidate_coverage\":" << certificate.minimumCandidateCoverage
                  << ",\"four_view_min_observed_coverage\":" << certificate.minimumObservedCoverage
                  << ",\"four_view_max_edge_error\":" << certificate.maximumEdgeError
                  << ",\"four_view_disagreement_pixels\":" << certificate.symmetricDifferencePixels
                  << ",\"registered_offsets\":[";
        for (size_t rotation = 0; rotation < 4; ++rotation)
        {
            if (rotation) std::cout << ",";
            std::cout << "[" << hull.viewPixelOffsets[rotation].x
                      << "," << hull.viewPixelOffsets[rotation].y << "]";
        }
        std::cout << "],\"view_fits\":[";
        for (size_t i = 0; i < certificate.views.size(); ++i)
        {
            if (i) std::cout << ",";
            const auto& fit = certificate.views[i];
            std::cout << "{\"rotation\":" << unsigned(fit.imageDirection)
                      << ",\"iou\":" << fit.fit.intersectionOverUnion
                      << ",\"candidate_coverage\":" << fit.fit.candidateCoverage
                      << ",\"observed_coverage\":" << fit.fit.observedCoverage
                      << ",\"disagreement_pixels\":" << fit.symmetricDifferencePixels
                      << "}";
        }
        std::cout << "]}" << std::endl;
    }
}

int main(int argc, char** argv)
{
    int firstPacket = 1;
    std::string exportFolder;
    if (argc > 1 && std::string(argv[1]).rfind("export:", 0) == 0)
    {
        exportFolder = std::string(argv[1]).substr(7);
        firstPacket = 2;
    }
    if (argc <= firstPacket)
    {
        std::cerr << "Provide at least one decoded legacy DAT packet\n";
        return 2;
    }
    for (int i = firstPacket; i < argc; ++i)
        EvaluateRealSmallScenery(i - firstPacket, argv[i], exportFolder);
    return 0;
}
