/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <future>
#include <thread>
#include <openrct2/paint/FirstPersonAsync.h>
#include <openrct2/paint/FirstPersonSpriteSnapshot.h>
#include <algorithm>
#include <openrct2/paint/FirstPersonRenderer.h>
#include <openrct2/paint/FirstPersonAssetReconstruction.h>
#include <openrct2/paint/FirstPersonPeriodicPassengerMotion.h>
#include <openrct2/paint/FirstPersonPhysicalProxy.h>
#include <openrct2/paint/FirstPersonPathGeometry.h>
#include <openrct2/paint/FirstPersonTrackTrajectory.h>
#include <openrct2/paint/FirstPersonTunnelGeometry.h>
#include <openrct2/paint/FirstPersonSmallSceneryCollision.h>
#include <openrct2/paint/FirstPersonVehicleBodyHull.h>
#include <openrct2/paint/FirstPersonVisualHull.h>
#include <openrct2/paint/FirstPersonWalkingSemantics.h>
#include <openrct2/paint/Paint.h>
#include <openrct2/entity/EntityBase.h>
#include <openrct2/ride/Vehicle.h>
#include <openrct2/world/tile_element/Slope.h>
#include <openrct2/paint/FirstPersonVehiclePose.h>
#include <openrct2/paint/tile_element/Paint.Path.h>
#include <openrct2/paint/tile_element/Paint.TileElement.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/world/Wall.h>
#include <openrct2/world/tile_element/PathElement.h>

#include <cmath>

using namespace OpenRCT2::Paint;
using OpenRCT2::Translate3DTo2DWithZ;
using OpenRCT2::Vehicle;

namespace
{
    constexpr ScreenSize kScreen{ 1000, 600 };
    constexpr float kPi = 3.14159265358979323846f;

    FirstPersonSilhouette MakeSilhouetteRect(
        int32_t x0, int32_t y0, int32_t x1, int32_t y1)
    {
        FirstPersonSilhouette result{};
        for (int32_t y = y0; y < y1; ++y)
        for (int32_t x = x0; x < x1; ++x)
            result.add(x, y);
        return result;
    }
}

TEST(FirstPersonAssetReconstructionTest, PhysicalDepthDeterminesSpriteFaceOwnership)
{
    FirstPersonDepthOwnerMap owners{};
    const std::array<ScreenCoordsXY, 4> square{ {
        { 0, 0 }, { 4, 0 }, { 4, 4 }, { 0, 4 },
    } };
    const auto addSquare =
        [&](uint32_t owner, float depth) {
            AddFirstPersonDepthTriangle(
                owners, owner,
                { square[0], square[1], square[2] },
                { depth, depth, depth });
            AddFirstPersonDepthTriangle(
                owners, owner,
                { square[0], square[2], square[3] },
                { depth, depth, depth });
        };

    constexpr uint32_t kBackFace = 3;
    constexpr uint32_t kFrontFace = 7;
    addSquare(kBackFace, 10.0f);
    addSquare(kFrontFace, 20.0f);

    const auto silhouette =
        MakeSilhouetteRect(0, 0, 4, 4);
    EXPECT_FLOAT_EQ(
        FirstPersonDepthOwnerCoverage(
            owners, kFrontFace, silhouette),
        1.0f);
    EXPECT_FLOAT_EQ(
        FirstPersonDepthOwnerCoverage(
            owners, kBackFace, silhouette),
        0.0f);

    // A later draw of a physically farther face must not steal the pixels.
    addSquare(kBackFace, 5.0f);
    EXPECT_FLOAT_EQ(
        FirstPersonDepthOwnerCoverage(
            owners, kFrontFace, silhouette),
        1.0f);
}

TEST(FirstPersonAssetReconstructionTest, TextureReprojectionRequiresOwnershipAndSourceCoverage)
{
    EXPECT_TRUE(
        FirstPersonTextureReprojectionIsReliable(
            0.99f, 0.95f));
    EXPECT_FALSE(
        FirstPersonTextureReprojectionIsReliable(
            0.97f, 1.00f));
    EXPECT_FALSE(
        FirstPersonTextureReprojectionIsReliable(
            1.00f, 0.89f));

    const auto silhouette =
        MakeSilhouetteRect(0, 0, 4, 4);
    EXPECT_FLOAT_EQ(
        FirstPersonSilhouettePredicateCoverage(
            silhouette,
            [](int32_t x, int32_t) {
                return x < 2;
            }),
        0.5f);
    EXPECT_FLOAT_EQ(
        FirstPersonSilhouettePredicateCoverage(
            silhouette,
            [](int32_t, int32_t) {
                return true;
            },
            8),
        0.0f);
}

TEST(FirstPersonPathGeometryTest, FootprintPreservesMarginsEdgesAndCorners)
{
    const auto isolated =
        BuildFirstPersonPathFootprint(0, 0, false);
    ASSERT_EQ(isolated.size(), 1u);
    EXPECT_EQ(isolated[0].x0, 3);
    EXPECT_EQ(isolated[0].y0, 3);
    EXPECT_EQ(isolated[0].x1, 29);
    EXPECT_EQ(isolated[0].y1, 29);

    const auto complete =
        BuildFirstPersonPathFootprint(0x0F, 0x0F, false);
    ASSERT_EQ(complete.size(), 9u);
    int32_t area = 0;
    for (const auto& cell : complete)
        area += (cell.x1 - cell.x0)
            * (cell.y1 - cell.y0);
    EXPECT_EQ(area, 32 * 32);

    const auto queue =
        BuildFirstPersonPathFootprint(0, 0x0F, true);
    ASSERT_EQ(queue.size(), 1u);
}

TEST(FirstPersonVisualHullTest, SolidVolumeMergesToSixBoundaryFaces)
{
    FirstPersonVisualHull hull{};
    hull.valid = true;
    hull.step = 2.0f;
    hull.sizeForward = 4;
    hull.sizeRight = 3;
    hull.sizeUp = 5;
    hull.occupied.assign(
        size_t(hull.sizeForward)
            * size_t(hull.sizeRight)
            * size_t(hull.sizeUp),
        1);
    const auto faces =
        BuildFirstPersonVisualHullBoundaryFaces(hull);
    ASSERT_EQ(faces.size(), 6u);
    EXPECT_EQ(
        std::count_if(
            faces.begin(), faces.end(),
            [](const auto& face) {
                return face.kind
                    == FirstPersonVisualHullFaceKind::bottom;
            }),
        1);
}

TEST(FirstPersonAssetReconstructionTest, AdjacentQuarterCellsDropInternalWall)
{
    struct Cell
    {
        int32_t qx{};
        int32_t qy{};
        int32_t lowZ{};
        int32_t highZ{};
        uint16_t sequence{};
    };
    const std::vector<Cell> cells{
        { 0, 0, 0, 16, 0 },
        { 1, 0, 0, 16, 1 },
    };
    const auto faces =
        BuildFirstPersonQuarterCellOccupancyFaces(cells);
    EXPECT_EQ(
        std::count_if(
            faces.begin(), faces.end(),
            [](const auto& face) {
                return (
                    face.kind == FirstPersonOccupancyFaceKind::maxX
                    && face.corners[0].x == 16)
                    || (
                        face.kind == FirstPersonOccupancyFaceKind::minX
                        && face.corners[0].x == 16);
            }),
        0);
    EXPECT_EQ(
        std::count_if(
            faces.begin(), faces.end(),
            [](const auto& face) {
                return face.kind
                    == FirstPersonOccupancyFaceKind::bottom;
            }),
        2);
}

TEST(FirstPersonDiagnosticGeometryTest, SharedBoundsWithoutSharedAreaAreNotOverlaps)
{
    const std::array<FirstPersonVec3, 3> lowerLeft{ {
        { 0.0f, 0.0f, 10.0f },
        { 32.0f, 0.0f, 10.0f },
        { 0.0f, 32.0f, 10.0f },
    } };
    const std::array<FirstPersonVec3, 3> upperRight{ {
        { 32.0f, 32.0f, 10.0f },
        { 0.0f, 32.0f, 10.0f },
        { 32.0f, 0.0f, 10.0f },
    } };

    // Both triangles have the same 32x32 AABB, but meet only along the
    // diagonal. The old horizontal diagnostic reported this as a full overlap.
    EXPECT_NEAR(
        FirstPersonCoplanarTriangleOverlapAreaXY(
            lowerLeft, upperRight),
        0.0f, 0.001f);
}

TEST(FirstPersonDiagnosticGeometryTest, GenuineCoplanarTriangleOverlapRetainsArea)
{
    const std::array<FirstPersonVec3, 3> triangle{ {
        { 0.0f, 0.0f, 10.0f },
        { 32.0f, 0.0f, 10.0f },
        { 0.0f, 32.0f, 10.0f },
    } };
    const std::array<FirstPersonVec3, 3> reversed{ {
        { 0.0f, 32.0f, 10.0f },
        { 32.0f, 0.0f, 10.0f },
        { 0.0f, 0.0f, 10.0f },
    } };

    EXPECT_NEAR(
        FirstPersonCoplanarTriangleOverlapAreaXY(
            triangle, reversed),
        512.0f, 0.001f);
}

TEST(FirstPersonSourceRotationTest, CameraRelativePointOwnsNativeQuadrant)
{
    FirstPersonCamera camera{};
    // Both points lie inside the same 32x32 world tile, but their azimuths
    // from the camera straddle the native 45-degree source-view boundary.
    EXPECT_EQ(FirstPersonSourceRotationForPoint(camera, { 30.0f, 28.0f, 0.0f }), 0);
    EXPECT_EQ(FirstPersonSourceRotationForPoint(camera, { 1.0f, 31.0f, 0.0f }), 1);
}

TEST(FirstPersonViewportClipTest, DirtyStripDoesNotReplaceFullProjectionViewport)
{
    const ScreenRect viewport{ 100, 50, 1100, 650 };
    const ScreenRect dirtyStrip{ 420, 50, 440, 650 };
    const auto clip =
        IntersectFirstPersonScreenRects(viewport, dirtyStrip);

    EXPECT_EQ(viewport.getWidth(), 1000);
    EXPECT_EQ(viewport.getHeight(), 600);
    EXPECT_EQ(clip.getLeft(), 420);
    EXPECT_EQ(clip.getWidth(), 20);
    EXPECT_EQ(clip.getHeight(), 600);
}

TEST(FirstPersonSourceRotationTest, HysteresisBelongsToTheTrackedPoint)
{
    FirstPersonCamera camera{};
    EXPECT_EQ(
        FirstPersonSourceRotationForPoint(
            camera, { 100.0f, 111.0f, 0.0f }, uint8_t{ 0 }),
        0);
    EXPECT_EQ(
        FirstPersonSourceRotationForPoint(
            camera, { 100.0f, 123.0f, 0.0f }, uint8_t{ 0 }),
        1);
}

TEST(FirstPersonHullMaterialTest, MissingUndersideKeepsRaisedArtworkVisibleFromBelow)
{
    std::array<FirstPersonVisualHullFace, 6> faces{};
    faces[0].normal = { 1, 0, 0 };
    faces[1].normal = { -1, 0, 0 };
    faces[2].normal = { 0, 1, 0 };
    faces[3].normal = { 0, -1, 0 };
    faces[4].normal = { 0, 0, 1 };
    faces[5].normal = { 0, 0, -1 };
    FirstPersonHullMaterialCoverage coverage(faces);

    std::vector<FirstPersonSurface> surfaces(5);
    for (size_t i = 0; i < surfaces.size(); ++i)
    {
        coverage.recordFace(i, 32 * 48, 32 * 48);
        surfaces[i].outwardNormal = faces[i].normal;
    }
    auto& top = surfaces.back();
    top.triangles[0].world = { 4704, 2368, 216 };
    const FirstPersonVec3 towardEye{ 4895.187f - 4704, 2391.147f - 2368, 132 - 216 };
    ASSERT_LT(FpDot(top.outwardNormal, towardEye), 0.0f);

    coverage.applyTo(surfaces.begin(), surfaces.end());

    EXPECT_FALSE(coverage.isClosed());
    for (size_t i = 0; i < 4; ++i)
        EXPECT_TRUE(surfaces[i].exteriorOnly);
    EXPECT_FALSE(top.exteriorOnly);
    for (const auto& surface : surfaces)
    {
        EXPECT_EQ(surface.diagnosticHullBoundaryFaces, 6u);
        EXPECT_EQ(surface.diagnosticHullMaterialFaces, 5u);
        EXPECT_EQ(surface.diagnosticHullOpaqueFaces, 5u);
    }
    EXPECT_FALSE(top.exteriorOnly && FpDot(top.outwardNormal, towardEye) <= 0.0f);
}

TEST(FirstPersonHullMaterialTest, TransparentFaceExposesOnlyOpposingBoundaryWithoutFillingPixels)
{
    std::array<FirstPersonVisualHullFace, 6> faces{};
    faces[0].normal = { 1, 0, 0 };
    faces[1].normal = { -1, 0, 0 };
    faces[2].normal = { 0, 1, 0 };
    faces[3].normal = { 0, -1, 0 };
    faces[4].normal = { 0, 0, 1 };
    faces[5].normal = { 0, 0, -1 };
    FirstPersonHullMaterialCoverage coverage(faces);

    std::vector<FirstPersonSurface> surfaces(6);
    for (size_t i = 0; i < surfaces.size(); ++i)
    {
        auto& surface = surfaces[i];
        surface.outwardNormal = faces[i].normal;
        surface.immutablePixels = { 17, 17, 17, 17 };
        if (i == 0)
            surface.immutablePixels.back() = 0;
        coverage.recordFace(i, surface.immutablePixels.size(),
            size_t(std::count_if(surface.immutablePixels.begin(), surface.immutablePixels.end(),
                [](uint8_t pixel) { return pixel != 0; })));
    }
    coverage.applyTo(surfaces.begin(), surfaces.end());

    EXPECT_FALSE(coverage.isClosed());
    EXPECT_EQ(coverage.materialFaces(), 6u);
    EXPECT_EQ(coverage.opaqueFaces(), 5u);
    EXPECT_TRUE(surfaces[0].exteriorOnly);
    EXPECT_FALSE(surfaces[1].exteriorOnly);
    for (size_t i = 2; i < surfaces.size(); ++i)
        EXPECT_TRUE(surfaces[i].exteriorOnly);
    for (const auto& surface : surfaces)
    {
        EXPECT_FALSE(surface.physicalCoverage);
        EXPECT_FALSE(surface.viewFacing);
    }
    EXPECT_EQ(surfaces.front().immutablePixels.back(), 0);
}

TEST(FirstPersonHullMaterialTest, ClosedBoundaryRetainsSidednessAroundAGeometricOpening)
{
    FirstPersonVisualHull hull{};
    hull.valid = true;
    hull.step = 2.0f;
    hull.sizeForward = hull.sizeRight = 3;
    hull.sizeUp = 1;
    hull.occupied.assign(9, 1);
    hull.occupied[4] = 0; // A real through-opening, with its own inner boundary.
    const auto faces = BuildFirstPersonVisualHullBoundaryFaces(hull);
    ASSERT_GT(faces.size(), 6u);
    FirstPersonHullMaterialCoverage coverage(faces);
    std::vector<FirstPersonSurface> surfaces(faces.size());
    for (size_t i = 0; i < faces.size(); ++i)
    {
        surfaces[i].outwardNormal = faces[i].normal;
        coverage.recordFace(i, 4, 4);
    }
    coverage.applyTo(surfaces.begin(), surfaces.end());

    EXPECT_TRUE(coverage.isClosed());
    EXPECT_FALSE(hull.contains(1, 1, 0));
    for (size_t i = 0; i < faces.size(); ++i)
    {
        EXPECT_TRUE(surfaces[i].exteriorOnly);
        EXPECT_FLOAT_EQ(surfaces[i].outwardNormal.x, faces[i].normal.x);
        EXPECT_FLOAT_EQ(surfaces[i].outwardNormal.y, faces[i].normal.y);
        EXPECT_FLOAT_EQ(surfaces[i].outwardNormal.z, faces[i].normal.z);
    }
}

TEST(FirstPersonHullMaterialTest, AbsentFacesCannotBeCertifiedByDuplicateOrEmptySamples)
{
    std::array<FirstPersonVisualHullFace, 0> emptyFaces{};
    FirstPersonHullMaterialCoverage empty(emptyFaces);
    EXPECT_FALSE(empty.isClosed());

    std::array<FirstPersonVisualHullFace, 6> faces{};
    faces[0].normal = { 1, 0, 0 };
    faces[1].normal = { -1, 0, 0 };
    faces[2].normal = { 0, 1, 0 };
    faces[3].normal = { 0, -1, 0 };
    faces[4].normal = { 0, 0, 1 };
    faces[5].normal = { 0, 0, -1 };
    FirstPersonHullMaterialCoverage coverage(faces);
    for (size_t i = 0; i < 6; ++i)
        coverage.recordFace(0, 4, 4);
    EXPECT_EQ(coverage.materialFaces(), 1u);
    EXPECT_FALSE(coverage.isClosed());
    for (size_t i = 1; i < 6; ++i)
        coverage.recordFace(i, 4, 4);
    EXPECT_TRUE(coverage.isClosed());
    coverage.recordFace(5, 4, 0);
    EXPECT_FALSE(coverage.isClosed());
    coverage.recordFace(5, 0, 0);
    EXPECT_FALSE(coverage.isClosed());
    coverage.recordFace(5, 4, 5);
    EXPECT_FALSE(coverage.isClosed());
    coverage.recordFace(6, 4, 4);
    EXPECT_FALSE(coverage.isClosed());
}

TEST(FirstPersonHullMaterialTest, ObjectRangePreservesAuthoredWallAndDirectionalSidednessAcrossCacheCopy)
{
    std::vector<FirstPersonSurface> scene(3);
    scene.front().exteriorOnly = true; // Independently authored wall.
    scene.front().outwardNormal = { -1, 0, 0 };

    std::array<FirstPersonVisualHullFace, 6> faces{};
    faces[0].normal = { 1, 0, 0 };
    faces[1].normal = { -1, 0, 0 };
    faces[2].normal = { 0, 1, 0 };
    faces[3].normal = { 0, -1, 0 };
    faces[4].normal = { 0, 0, 1 };
    faces[5].normal = { 0, 0, -1 };
    scene[1].outwardNormal = faces[0].normal;
    scene[2].outwardNormal = faces[2].normal;

    FirstPersonHullMaterialCoverage coverage(faces);
    for (size_t i = 0; i < faces.size(); ++i)
        coverage.recordFace(i, 4, i == 1 ? 3 : 4);
    coverage.applyTo(scene.begin() + 1, scene.end());

    EXPECT_TRUE(scene.front().exteriorOnly);
    EXPECT_EQ(scene.front().diagnosticHullBoundaryFaces, 0u);

    const std::vector<FirstPersonSurface> cached(scene.begin() + 1, scene.end());
    ASSERT_EQ(cached.size(), 2u);
    EXPECT_FALSE(cached[0].exteriorOnly);
    EXPECT_TRUE(cached[1].exteriorOnly);
    for (const auto& surface : cached)
    {
        EXPECT_EQ(surface.diagnosticHullBoundaryFaces, 6u);
        EXPECT_EQ(surface.diagnosticHullMaterialFaces, 6u);
        EXPECT_EQ(surface.diagnosticHullOpaqueFaces, 5u);
    }
}

TEST(FirstPersonAssetReconstructionTest, VerticalCoverageCanSplitOneWallIntoSeveralStrips)
{
    const auto exposed = SubtractFirstPersonVerticalCoverage(
        { 0, 20 }, { { 5, 10 }, { 14, 16 } });
    ASSERT_EQ(exposed.size(), 3u);
    EXPECT_EQ(exposed[0].low, 0);
    EXPECT_EQ(exposed[0].high, 5);
    EXPECT_EQ(exposed[1].low, 10);
    EXPECT_EQ(exposed[1].high, 14);
    EXPECT_EQ(exposed[2].low, 16);
    EXPECT_EQ(exposed[2].high, 20);

    EXPECT_TRUE(FirstPersonVerticalPointCoveredAbove(
        7, { { 5, 10 }, { 14, 16 } }));
    EXPECT_FALSE(FirstPersonVerticalPointCoveredAbove(
        12, { { 5, 10 }, { 14, 16 } }));
}

TEST(FirstPersonWalkingSemanticsTest, StableDoorStatesHaveAuthoritativeCollision)
{
    EXPECT_TRUE(FirstPersonDoorBlocksWalking(0));
    EXPECT_FALSE(FirstPersonDoorBlocksWalking(5));
    EXPECT_TRUE(FirstPersonDoorBlocksWalking(1));
    EXPECT_TRUE(FirstPersonDoorBlocksWalking(6));
    EXPECT_TRUE(FirstPersonDoorBlocksWalking(15));
}

TEST(FirstPersonWalkingSemanticsTest, PathSlopeMappingMatchesNativePathDirections)
{
    EXPECT_EQ(
        FirstPersonPathLandSlope(0),
        kTileSlopeSWSideUp);
    EXPECT_EQ(
        FirstPersonPathLandSlope(1),
        kTileSlopeNWSideUp);
    EXPECT_EQ(
        FirstPersonPathLandSlope(2),
        kTileSlopeNESideUp);
    EXPECT_EQ(
        FirstPersonPathLandSlope(3),
        kTileSlopeSESideUp);
}

TEST(FirstPersonWalkingSemanticsTest, WidePathPublishesFullDeckWithoutChangingGraphEdges)
{
    OpenRCT2::PathElement path{};
    path.setWide(true);
    path.setSloped(false);
    path.setIsQueue(false);
    path.setEdges(0);
    path.setCorners(0);

    auto constraint =
        FirstPersonWalkabilityFromPath(
            CoordsXY{ 0, 0 }, path);
    EXPECT_EQ(
        constraint.kind,
        FirstPersonWalkabilityKind::path);
    EXPECT_TRUE(constraint.walkableFloor);
    EXPECT_TRUE(constraint.visualFullTileDeck);
    EXPECT_EQ(constraint.visualDeckEdgeMask, 0x0F);
    EXPECT_EQ(constraint.connectedSides, 0);
    EXPECT_FALSE(
        FirstPersonWalkabilityContainsPoint(
            constraint, CoordsXY{ 1, 1 }));
    EXPECT_TRUE(
        FirstPersonWalkabilityContainsPoint(
            constraint, CoordsXY{ 16, 16 }));

    path.setIsQueue(true);
    constraint =
        FirstPersonWalkabilityFromPath(
            CoordsXY{ 0, 0 }, path);
    EXPECT_EQ(
        constraint.kind,
        FirstPersonWalkabilityKind::queue);
    EXPECT_FALSE(constraint.visualFullTileDeck);
    EXPECT_EQ(constraint.visualDeckEdgeMask, 0);
}

TEST(FirstPersonWalkingSemanticsTest, ConnectedPathEdgesPublishMaterialContinuity)
{
    OpenRCT2::PathElement path{};
    path.setWide(false);
    path.setSloped(false);
    path.setIsQueue(false);
    path.setEdges(0x05);
    path.setCorners(0);

    const auto constraint =
        FirstPersonWalkabilityFromPath(
            CoordsXY{ 0, 0 }, path);
    EXPECT_FALSE(constraint.visualFullTileDeck);
    EXPECT_EQ(constraint.connectedSides, 0x05);
    EXPECT_EQ(constraint.visualDeckEdgeMask, 0x05);
}

TEST(FirstPersonWalkingSemanticsTest, ConnectionMasksRotateWithWorldOrientation)
{
    EXPECT_EQ(
        RotateFirstPersonConnectionMask(0x05, 0),
        0x05);
    EXPECT_EQ(
        RotateFirstPersonConnectionMask(0x05, 1),
        0x0A);
    EXPECT_EQ(
        RotateFirstPersonConnectionMask(0x05, 2),
        0x05);
    EXPECT_EQ(
        FirstPersonPassageAxisForConnections(0x05),
        FirstPersonPassageAxis::x);
    EXPECT_EQ(
        FirstPersonPassageAxisForConnections(0x0A),
        FirstPersonPassageAxis::y);
    EXPECT_EQ(
        FirstPersonPassageAxisForConnections(0x01),
        FirstPersonPassageAxis::none);
    EXPECT_EQ(
        FirstPersonPassageAxisForConnections(0x0F),
        FirstPersonPassageAxis::none);
}

TEST(FirstPersonWalkingSemanticsTest, ParkEntranceCentrePublishesRotatedThroughPassage)
{
    OpenRCT2::EntranceElement entrance{};
    entrance.setEntranceType(
        OpenRCT2::EntranceType::parkEntrance);
    entrance.setSequenceIndex(
        OpenRCT2::ParkEntranceSequence::centre);
    entrance.setBaseZ(96);
    entrance.setDirection(Direction{ 1 });

    const auto constraint =
        FirstPersonWalkabilityFromEntrance(
            CoordsXY{ 320, 640 },
            entrance);
    EXPECT_EQ(
        constraint.kind,
        FirstPersonWalkabilityKind::parkEntrance);
    EXPECT_EQ(constraint.baseZ, 96);
    EXPECT_EQ(constraint.connectedSides, 0x0A);
    EXPECT_TRUE(constraint.walkableFloor);
    EXPECT_TRUE(
        constraint.guaranteedThroughPassage);
    EXPECT_FALSE(
        constraint.visualFullTileDeck);
    EXPECT_EQ(
        constraint.visualDeckEdgeMask, 0x0A);

    entrance.setSequenceIndex(
        OpenRCT2::ParkEntranceSequence::left);
    const auto side =
        FirstPersonWalkabilityFromEntrance(
            CoordsXY{ 288, 640 },
            entrance);
    EXPECT_EQ(side.connectedSides, 0);
    EXPECT_FALSE(side.walkableFloor);
    EXPECT_FALSE(side.guaranteedThroughPassage);
}

TEST(FirstPersonWalkingSemanticsTest, RideEntrancesRemainGraphEndpoints)
{
    OpenRCT2::EntranceElement entrance{};
    entrance.setEntranceType(
        OpenRCT2::EntranceType::rideEntrance);
    entrance.setSequenceIndex(
        OpenRCT2::ParkEntranceSequence::centre);
    entrance.setBaseZ(80);
    entrance.setDirection(Direction{ 3 });

    auto constraint =
        FirstPersonWalkabilityFromEntrance(
            CoordsXY{ 64, 96 }, entrance);
    EXPECT_EQ(
        constraint.kind,
        FirstPersonWalkabilityKind::rideEntrance);
    EXPECT_EQ(constraint.connectedSides, 0x02);
    EXPECT_FALSE(constraint.walkableFloor);
    EXPECT_FALSE(
        constraint.guaranteedThroughPassage);
    EXPECT_EQ(constraint.visualDeckEdgeMask, 0);

    entrance.setEntranceType(
        OpenRCT2::EntranceType::rideExit);
    constraint =
        FirstPersonWalkabilityFromEntrance(
            CoordsXY{ 64, 96 }, entrance);
    EXPECT_EQ(
        constraint.kind,
        FirstPersonWalkabilityKind::rideExit);
    EXPECT_EQ(constraint.connectedSides, 0x02);
    EXPECT_FALSE(constraint.walkableFloor);
}

TEST(FirstPersonWalkingSemanticsTest, GuaranteedPassageDefinesOnlyTheWalkableCorridor)
{
    FirstPersonWalkabilityConstraint entrance{};
    entrance.kind =
        FirstPersonWalkabilityKind::parkEntrance;
    entrance.tile = CoordsXY{ 320, 640 };
    entrance.baseZ = 96;
    entrance.connectedSides = 0x05;
    entrance.walkableFloor = true;
    entrance.guaranteedThroughPassage = true;
    entrance.visualDeckEdgeMask = 0x05;

    EXPECT_TRUE(
        FirstPersonWalkabilityContainsPoint(
            entrance,
            CoordsXY{ 321, 656 }));
    EXPECT_TRUE(
        FirstPersonWalkabilityContainsPoint(
            entrance,
            CoordsXY{ 351, 656 }));
    EXPECT_FALSE(
        FirstPersonWalkabilityContainsPoint(
            entrance,
            CoordsXY{ 336, 647 }));
    // A walking body radius narrows the permitted centre line rather than
    // widening the carved opening.
    EXPECT_FALSE(
        FirstPersonWalkabilityContainsPoint(
            entrance,
            CoordsXY{ 336, 649 },
            2.0f));
    EXPECT_TRUE(
        FirstPersonWalkabilityContainsPoint(
            entrance,
            CoordsXY{ 336, 650 },
            2.0f));
}

TEST(FirstPersonWalkingSemanticsTest, PathAndParkEntranceShareGraphEdges)
{
    FirstPersonWalkabilityConstraint path{};
    path.kind = FirstPersonWalkabilityKind::path;
    path.tile = CoordsXY{ 0, 0 };
    path.baseZ = 80;
    path.connectedSides = 1u << 2;
    path.walkableFloor = true;

    FirstPersonWalkabilityConstraint entrance{};
    entrance.kind =
        FirstPersonWalkabilityKind::parkEntrance;
    entrance.tile =
        CoordsXY{ kCoordsXYStep, 0 };
    entrance.baseZ = 80;
    entrance.connectedSides = 0x05;
    entrance.walkableFloor = true;
    entrance.guaranteedThroughPassage = true;

    EXPECT_TRUE(
        FirstPersonWalkabilitySupportsConnect(
            path, entrance));
    entrance.connectedSides = 1u << 2;
    EXPECT_FALSE(
        FirstPersonWalkabilitySupportsConnect(
            path, entrance));
}

TEST(FirstPersonWalkingSemanticsTest, WideVisualDeckCanReachTileBoundaryWithoutInventingEdges)
{
    FirstPersonWalkabilityConstraint wide{};
    wide.kind = FirstPersonWalkabilityKind::path;
    wide.tile = CoordsXY{ 0, 0 };
    wide.walkableFloor = true;
    wide.wide = true;
    wide.visualFullTileDeck = true;
    wide.visualDeckEdgeMask = 0x0F;
    wide.connectedSides = 0;

    EXPECT_FALSE(
        FirstPersonWalkabilityContainsPoint(
            wide, CoordsXY{ 1, 1 }));
    EXPECT_FALSE(
        FirstPersonWalkabilityContainsPoint(
            wide, CoordsXY{ 31, 31 }));
    EXPECT_TRUE(
        FirstPersonWalkabilityContainsPoint(
            wide, CoordsXY{ 16, 16 }));
    // Visual continuity does not fabricate graph connectivity.
    FirstPersonWalkabilityConstraint neighbour = wide;
    neighbour.tile =
        CoordsXY{ kCoordsXYStep, 0 };
    EXPECT_FALSE(
        FirstPersonWalkabilitySupportsConnect(
            wide, neighbour));
}

TEST(FirstPersonSmallSceneryCollisionTest, PartialWalkingMaskCannotReplaceTallArtwork)
{
    EXPECT_TRUE(
        FirstPersonSmallSceneryVisualReconstructionCoversHeight(
            20, 20));
    EXPECT_FALSE(
        FirstPersonSmallSceneryVisualReconstructionCoversHeight(
            20, 18));
    EXPECT_FALSE(
        FirstPersonSmallSceneryVisualReconstructionCoversHeight(
            64, 20));
}

TEST(FirstPersonVisualHullTest, AuthoritativeOccupancySurvivesMissingSilhouetteEvidence)
{
    FirstPersonVisualHullBounds bounds{};
    bounds.maxForward = 32.0f;
    bounds.maxRight = 32.0f;
    bounds.maxUp = 24.0f;
    bounds.step = 4.0f;
    FirstPersonVisualHullConfig config{};
    config.maximumAxisCells = 16;
    config.maximumGridCells = 4096;
    config.maximumOccupiedCells = 4096;

    const std::vector<FirstPersonVisualHullTextureView> views{
        { 0, 100 }, { 1, 101 }, { 2, 102 }, { 3, 103 }
    };
    const auto hull = BuildFirstPersonOccupancyHull(
        bounds, config, views,
        [](FirstPersonVec3 point) {
            return point.x < 16.0f
                && point.y < 16.0f;
        });

    ASSERT_TRUE(hull.valid);
    EXPECT_EQ(hull.textureViews.size(), 4u);
    EXPECT_TRUE(hull.containsPoint({ 4.0f, 4.0f, 4.0f }));
    EXPECT_TRUE(hull.containsPoint({ 12.0f, 12.0f, 20.0f }));
    EXPECT_FALSE(hull.containsPoint({ 20.0f, 4.0f, 4.0f }));
    EXPECT_FALSE(hull.containsPoint({ 4.0f, 20.0f, 4.0f }));
}

TEST(FirstPersonAssetReconstructionTest, KnownSpriteBoundsClassifyOutsideAsTransparent)
{
    EXPECT_EQ(
        FirstPersonArtworkSampleCoverageForPoint(
            false, 0, 0, 8, 8),
        FirstPersonArtworkSampleCoverage::unknown);
    EXPECT_EQ(
        FirstPersonArtworkSampleCoverageForPoint(
            true, -1, 4, 8, 8),
        FirstPersonArtworkSampleCoverage::transparent);
    EXPECT_EQ(
        FirstPersonArtworkSampleCoverageForPoint(
            true, 8, 4, 8, 8),
        FirstPersonArtworkSampleCoverage::transparent);
    EXPECT_EQ(
        FirstPersonArtworkSampleCoverageForPoint(
            true, 7, 7, 8, 8),
        FirstPersonArtworkSampleCoverage::raster);
}

TEST(FirstPersonSmallSceneryCollisionTest, QuarterMappingMatchesNativeConstructionQuadrants)
{
    EXPECT_EQ(FirstPersonSmallSceneryQuarterForPoint(24, 24), 0);
    EXPECT_EQ(FirstPersonSmallSceneryQuarterForPoint(24, 8), 1);
    EXPECT_EQ(FirstPersonSmallSceneryQuarterForPoint(8, 8), 2);
    EXPECT_EQ(FirstPersonSmallSceneryQuarterForPoint(8, 24), 3);

    FirstPersonSmallSceneryWalkingMask mask{};
    mask.layerCount = 2;
    mask.layerLowZ[0] = 0;
    mask.layerHighZ[0] = 8;
    mask.layerLowZ[1] = 8;
    mask.layerHighZ[1] = 20;
    mask.add(0, 2, 3);
    mask.add(1, 3, 2);

    EXPECT_TRUE(mask.valid);
    EXPECT_TRUE(mask.contains(0, 2, 3));
    EXPECT_FALSE(mask.contains(0, 3, 2));
    EXPECT_TRUE(mask.contains(1, 3, 2));
    EXPECT_FALSE(mask.contains(1, 2, 3));
}

TEST(FirstPersonVehicleBodyHullTest, NativeYawProjectionUsesCarLocalForwardAndRight)
{
    constexpr FirstPersonVec3 local{ 6.0f, 2.0f, 4.0f };
    const auto yaw0 =
        ProjectFirstPersonVehicleLocalPoint(0, local);
    EXPECT_NEAR(yaw0[0], 4.0f, 0.0001f);
    EXPECT_NEAR(yaw0[1], -8.0f, 0.0001f);

    const auto yaw8 =
        ProjectFirstPersonVehicleLocalPoint(8, local);
    EXPECT_NEAR(yaw8[0], 8.0f, 0.0001f);
    EXPECT_NEAR(yaw8[1], -2.0f, 0.0001f);

    FirstPersonVehicleBodyHull hull{};
    hull.step = 4.0f;
    hull.minForward = -4;
    hull.minRight = -4;
    hull.minUp = 0;
    hull.sizeForward = 2;
    hull.sizeRight = 2;
    hull.sizeUp = 2;
    hull.occupied.assign(8, 0);
    hull.occupied[7] = 1;
    EXPECT_TRUE(hull.contains(1, 1, 1));
    EXPECT_TRUE(hull.containsPoint({ 2.0f, 2.0f, 6.0f }));
    EXPECT_FALSE(hull.containsPoint({ -2.0f, -2.0f, 2.0f }));
}

TEST(FirstPersonAssetReconstructionTest, QuarterCellFallbackKeepsExactOccupancyFaces)
{
    struct Cell
    {
        int32_t qx{}, qy{}, lowZ{}, highZ{};
        uint16_t sequence{};
    };
    const std::vector<Cell> cells{
        { 2, 3, 8, 28, 7 }
    };
    const auto faces =
        BuildFirstPersonQuarterCellOccupancyFaces(cells);

    ASSERT_EQ(faces.size(), 6u);
    EXPECT_EQ(faces[0].sequence, 7u);
    EXPECT_EQ(faces[0].corners[0].x, 32);
    EXPECT_EQ(faces[0].corners[0].y, 48);
    EXPECT_EQ(faces[0].corners[0].z, 8);
    EXPECT_EQ(faces[4].kind, FirstPersonOccupancyFaceKind::top);
    EXPECT_EQ(faces[5].kind, FirstPersonOccupancyFaceKind::bottom);
    EXPECT_EQ(faces.back().corners[0].z, 28);

    const auto source =
        ChooseFirstPersonOccupancyFaceSourceDirection(
            FirstPersonOccupancyFaceKind::minX,
            [](uint8_t direction) {
                return direction == 1 || direction == 3;
            });
    ASSERT_TRUE(source.has_value());
    EXPECT_EQ(*source, 1);
}

TEST(FirstPersonAssetReconstructionTest, NativeViewRotationInvertsPainterDirection)
{
    for (uint8_t objectDirection = 0; objectDirection < 4; ++objectDirection)
    for (uint8_t nativeDirection = 0; nativeDirection < 4; ++nativeDirection)
    {
        const auto viewportRotation = FirstPersonViewportRotationForNativeView(
            objectDirection, nativeDirection);
        EXPECT_EQ(
            uint8_t((objectDirection + viewportRotation) & 3u),
            nativeDirection);
    }
}

TEST(FirstPersonAssetReconstructionTest, FourMatchingViewsPassReliabilityGate)
{
    std::array<FirstPersonSilhouette, 4> observed{};
    std::array<FirstPersonSilhouette, 4> candidate{};
    for (size_t i = 0; i < observed.size(); ++i)
    {
        observed[i] = MakeSilhouetteRect(-20, -40, 21, 1);
        candidate[i] = observed[i];
    }

    const auto fit = CompareFirstPersonMultiViewSilhouettes(observed, candidate);
    ASSERT_TRUE(fit.valid);
    EXPECT_FLOAT_EQ(fit.minimumIntersectionOverUnion, 1.0f);
    EXPECT_EQ(fit.maximumEdgeError, 0);
    EXPECT_TRUE(IsFirstPersonMultiViewFitReliable(fit));
}

TEST(FirstPersonVisualHullTest, EveryAdmittedViewCarvesAuthoritativeOccupancy)
{
    std::vector<FirstPersonVisualHullView> views(4);
    for (uint8_t direction = 0; direction < 4; ++direction)
    {
        views[direction].imageDirection = direction;
        views[direction].observed =
            MakeSilhouetteRect(-4, -4, 9, 9);
    }

    FirstPersonVisualHullBounds bounds{};
    bounds.minForward = 0.0f;
    bounds.maxForward = 4.0f;
    bounds.minRight = 0.0f;
    bounds.maxRight = 4.0f;
    bounds.minUp = 0.0f;
    bounds.maxUp = 4.0f;
    bounds.step = 2.0f;

    FirstPersonVisualHullConfig config{};
    config.minimumViews = 4;
    config.minimumOccupiedCells = 1;
    config.maximumOccupiedCells = 8;
    config.maximumGridCells = 8;
    config.maximumAxisCells = 2;
    config.minimumCandidateCoverage = 0.0f;
    config.minimumObservedCoverage = 0.0f;
    config.maximumEdgeError = 20;

    const auto hull = BuildFirstPersonVisualHull(
        views, bounds, config,
        [](uint8_t, FirstPersonVec3 point) {
            return std::array<float, 2>{
                point.x, point.z
            };
        },
        [](FirstPersonVec3 point) {
            return point.y < 2.0f;
        },
        [](const FirstPersonVisualHullView& view,
            FirstPersonVec3 point) {
            return view.imageDirection != 3
                || point.x < 2.0f;
        });

    ASSERT_TRUE(hull.valid);
    EXPECT_TRUE(hull.contains(0, 0, 0));
    EXPECT_TRUE(hull.contains(0, 0, 1));
    EXPECT_FALSE(hull.contains(1, 0, 0));
    EXPECT_FALSE(hull.contains(0, 1, 0));
}

TEST(FirstPersonAssetReconstructionTest, OneContradictoryViewForcesFallback)
{
    std::array<FirstPersonSilhouette, 4> observed{};
    std::array<FirstPersonSilhouette, 4> candidate{};
    for (size_t i = 0; i < observed.size(); ++i)
    {
        observed[i] = MakeSilhouetteRect(-20, -40, 21, 1);
        candidate[i] = observed[i];
    }
    candidate[2] = MakeSilhouetteRect(-8, -40, 33, 1);

    const auto fit = CompareFirstPersonMultiViewSilhouettes(observed, candidate);
    ASSERT_TRUE(fit.valid);
    EXPECT_GT(fit.maximumEdgeError, 8);
    EXPECT_FALSE(IsFirstPersonMultiViewFitReliable(fit));
}

TEST(FirstPersonAssetReconstructionTest, ProjectedQuadRasterizesAreaNotJustEdges)
{
    FirstPersonSilhouette silhouette{};
    AddFirstPersonSilhouetteQuad(
        silhouette,
        { {
            { 0, 0 }, { 16, 8 }, { 0, 16 }, { -16, 8 },
        } });
    EXPECT_GT(silhouette.size(), 100u);
    EXPECT_TRUE(silhouette.contains(0, 8));
    EXPECT_FALSE(silhouette.contains(20, 8));
}

TEST(FirstPersonAssetReconstructionTest, OversizedProjectedFaceForcesFallback)
{
    auto observed = MakeSilhouetteRect(-20, -40, 21, 1);
    FirstPersonSilhouette candidate{};
    AddFirstPersonSilhouetteQuad(
        candidate,
        { {
            { -1000000, 0 }, { 1000000, 0 },
            { 1000000, 1000000 }, { -1000000, 1000000 },
        } });

    EXPECT_TRUE(candidate.overflowed);
    const auto fit = CompareFirstPersonSilhouettes(observed, candidate);
    EXPECT_FALSE(fit.valid);
}


TEST(FirstPersonAssetReconstructionTest, RotatedFullTileKeepsNativePlacementPivot)
{
    constexpr std::array<CoordsXY, 4> corners{ {
        { 0, 0 }, { 32, 0 }, { 32, 32 }, { 0, 32 },
    } };
    for (uint8_t direction = 0; direction < 4; ++direction)
    {
        int32_t minX = 1000, minY = 1000, maxX = -1000, maxY = -1000;
        for (const auto corner : corners)
        {
            const auto placed = FirstPersonLargeSceneryPlacedPoint(
                { 0, 0 }, corner, direction);
            minX = std::min(minX, placed.x);
            minY = std::min(minY, placed.y);
            maxX = std::max(maxX, placed.x);
            maxY = std::max(maxY, placed.y);
        }
        EXPECT_EQ(minX, 0);
        EXPECT_EQ(minY, 0);
        EXPECT_EQ(maxX, 32);
        EXPECT_EQ(maxY, 32);
    }

    // A second source tile rotates as a tile offset, while its internal
    // coordinates still rotate about that destination tile's centre.
    const auto p0 = FirstPersonLargeSceneryPlacedPoint(
        { 32, 0 }, { 32, 0 }, 1);
    const auto p1 = FirstPersonLargeSceneryPlacedPoint(
        { 32, 0 }, { 64, 32 }, 1);
    EXPECT_EQ(p0.x, 0);
    EXPECT_EQ(p0.y, 0);
    EXPECT_EQ(p1.x, 32);
    EXPECT_EQ(p1.y, -32);
}

TEST(FirstPersonAssetReconstructionTest, DepthOwnerRejectsOccludedTextureSource)
{
    const std::array<ScreenCoordsXY, 3> triangle{ {
        { 0, 0 }, { 16, 0 }, { 0, 16 },
    } };
    FirstPersonSilhouette silhouette{};
    AddFirstPersonSilhouetteTriangle(
        silhouette, triangle[0], triangle[1], triangle[2]);

    FirstPersonDepthOwnerMap owners{};
    AddFirstPersonDepthTriangle(
        owners, 1, triangle, { 1.0f, 1.0f, 1.0f });
    AddFirstPersonDepthTriangle(
        owners, 2, triangle, { 2.0f, 2.0f, 2.0f });

    EXPECT_LT(FirstPersonDepthOwnerCoverage(owners, 1, silhouette), 0.01f);
    EXPECT_GT(FirstPersonDepthOwnerCoverage(owners, 2, silhouette), 0.99f);
}


TEST(FirstPersonAssetReconstructionTest, NativeFacingAndDepthOwnershipAgreeForAsymmetricBox)
{
    struct Face
    {
        CoordsXY normal{};
        std::array<CoordsXYZ, 4> corners{};
        uint32_t owner{};
        bool top = false;
    };

    const std::array<Face, 5> faces{ {
        { { -1, 0 }, { { { 0, 0, 0 }, { 0, 24, 0 }, { 0, 24, 32 }, { 0, 0, 32 } } }, 0, false },
        { { 1, 0 }, { { { 16, 24, 0 }, { 16, 0, 0 }, { 16, 0, 32 }, { 16, 24, 32 } } }, 1, false },
        { { 0, -1 }, { { { 16, 0, 0 }, { 0, 0, 0 }, { 0, 0, 32 }, { 16, 0, 32 } } }, 2, false },
        { { 0, 1 }, { { { 0, 24, 0 }, { 16, 24, 0 }, { 16, 24, 32 }, { 0, 24, 32 } } }, 3, false },
        { { 0, 0 }, { { { 0, 0, 32 }, { 16, 0, 32 }, { 16, 24, 32 }, { 0, 24, 32 } } }, 4, true },
    } };

    for (uint8_t rotation = 0; rotation < 4; ++rotation)
    {
        FirstPersonDepthOwnerMap owners{};
        std::array<FirstPersonSilhouette, 5> projected{};
        for (const auto& face : faces)
        {
            if (!face.top
                && !FirstPersonFaceVisibleFromNativeView(
                    face.normal, rotation))
                continue;

            std::array<ScreenCoordsXY, 4> screen{};
            std::array<float, 4> depth{};
            for (size_t i = 0; i < 4; ++i)
            {
                screen[i] =
                    Translate3DTo2DWithZ(rotation, face.corners[i]);
                depth[i] =
                    FirstPersonIsoDepth(rotation, face.corners[i]);
            }
            AddFirstPersonSilhouetteQuad(
                projected[face.owner], screen);
            AddFirstPersonDepthTriangle(
                owners, face.owner,
                { screen[0], screen[1], screen[2] },
                { depth[0], depth[1], depth[2] });
            AddFirstPersonDepthTriangle(
                owners, face.owner,
                { screen[0], screen[2], screen[3] },
                { depth[0], depth[2], depth[3] });
        }

        const auto view = FirstPersonNativeViewDirection(rotation);
        const uint32_t xOwner = view.x > 0 ? 1u : 0u;
        const uint32_t yOwner = view.y > 0 ? 3u : 2u;
        ASSERT_FALSE(projected[xOwner].empty());
        ASSERT_FALSE(projected[yOwner].empty());
        EXPECT_GT(
            FirstPersonDepthOwnerCoverage(
                owners, xOwner, projected[xOwner]),
            0.97f);
        EXPECT_GT(
            FirstPersonDepthOwnerCoverage(
                owners, yOwner, projected[yOwner]),
            0.97f);

        const uint32_t backX = view.x > 0 ? 0u : 1u;
        const uint32_t backY = view.y > 0 ? 2u : 3u;
        EXPECT_TRUE(projected[backX].empty());
        EXPECT_TRUE(projected[backY].empty());
    }
}

// Coplanar ordering is now material/physical-depth based, with no semantic
// role whitelist. The old whitelist test referenced an API removed at HEAD.

TEST(FirstPersonTrackTrajectoryTest, MotionTemplateIsIndependentOfRailCrossSection)
{
    const auto* a = GetFirstPersonTrackTrajectoryTemplate(
        OpenRCT2::TrackElemType::flat, 0);
    const auto* b = GetFirstPersonTrackTrajectoryTemplate(
        OpenRCT2::TrackElemType::flat, 0);
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a, b);
    ASSERT_FALSE(a->points.empty());
    EXPECT_TRUE(
        FirstPersonTrackTrajectoryTemplateSamplesContinuous(
            OpenRCT2::TrackElemType::flat, 0));

    const FirstPersonTrackRailProfile profile{};
    EXPECT_EQ(profile.railCount, 2);
    EXPECT_FLOAT_EQ(profile.halfGauge, 0.0f);
    EXPECT_FLOAT_EQ(profile.halfWidth, 0.0f);
    EXPECT_FLOAT_EQ(profile.halfHeight, 0.0f);
}

TEST(FirstPersonTrackTrajectoryTest, StyleBaselineAlwaysGeneratesStableGeometry)
{
    FirstPersonTrackTrajectory trajectory{};
    FirstPersonBasis basis{};
    basis.forward = { 1.0f, 0.0f, 0.0f };
    basis.right = { 0.0f, 1.0f, 0.0f };
    basis.up = { 0.0f, 0.0f, 1.0f };
    trajectory.points = {
        { { 0.0f, 0.0f, 0.0f }, basis, 0 },
        { { 12.0f, 0.0f, 0.0f }, basis, 1 },
    };

    const auto twin =
        FirstPersonDefaultTrackRailProfile(
            TrackStyle::corkscrewRollerCoaster);
    EXPECT_EQ(twin.railCount, 2);
    EXPECT_EQ(
        BuildFirstPersonRailProxySegments(
            trajectory, twin).size(),
        2u);

    const auto single =
        FirstPersonDefaultTrackRailProfile(
            TrackStyle::singleRailRollerCoaster);
    EXPECT_EQ(single.railCount, 1);
    const auto singleRail =
        BuildFirstPersonRailProxySegments(
            trajectory, single);
    ASSERT_EQ(singleRail.size(), 1u);
    EXPECT_FLOAT_EQ(singleRail[0].a.y, 0.0f);
    EXPECT_EQ(
        singleRail[0].provenance,
        FirstPersonPhysicalProxyProvenance::
            authoritativeTrackTrajectory);
}

TEST(FirstPersonTrackTrajectoryTest, StandardSamplesMatchVehicleMotionSource)
{
    constexpr FirstPersonVec3 origin{ 320.0f, 640.0f, 80.0f };
    const auto trajectory = BuildFirstPersonTrackTrajectory(
        OpenRCT2::TrackElemType::flat, 0, origin);
    ASSERT_TRUE(trajectory.has_value());

    const size_t index = size_t(EnumValue(OpenRCT2::TrackElemType::flat))
        * kNumOrthogonalDirections;
    const auto* list = gTrackVehicleInfo[
        EnumValue(VehicleTrackSubposition::standard)][index];
    ASSERT_NE(list, nullptr);
    ASSERT_EQ(trajectory->points.size(), list->size);

    for (const size_t sampleIndex : {
             size_t{ 0 }, trajectory->points.size() / 2,
             trajectory->points.size() - 1 })
    {
        const auto& source = list->info[sampleIndex];
        const auto& point = trajectory->points[sampleIndex];
        EXPECT_FLOAT_EQ(point.position.x, origin.x + source.x);
        EXPECT_FLOAT_EQ(point.position.y, origin.y + source.y);
        EXPECT_FLOAT_EQ(point.position.z, origin.z + source.z);
        EXPECT_EQ(point.progress, sampleIndex);
    }
    EXPECT_TRUE(FirstPersonTrackTrajectorySamplesContinuous(*trajectory));
}

TEST(FirstPersonTrackTrajectoryTest, EndpointGapMeasuresPhysicalDiscontinuity)
{
    FirstPersonTrackTrajectory first{};
    first.points = {
        { { 0.0f, 0.0f, 0.0f }, {}, 0 },
        { { 4.0f, 0.0f, 0.0f }, {}, 1 },
    };
    FirstPersonTrackTrajectory next{};
    next.points = {
        { { 5.5f, 0.0f, 0.0f }, {}, 0 },
        { { 8.0f, 0.0f, 0.0f }, {}, 1 },
    };
    EXPECT_FLOAT_EQ(FirstPersonTrackTrajectoryEndpointGap(first, next), 1.5f);
    EXPECT_TRUE(FirstPersonTrackTrajectorySamplesContinuous(first, 4.0f));
    EXPECT_TRUE(FirstPersonTrackTrajectorySamplesContinuous(next, 4.0f));
    EXPECT_FALSE(FirstPersonTrackTrajectorySamplesContinuous(next, 2.0f));
}

TEST(FirstPersonVehiclePresentationTest, HalfTweenOwnsOneSharedCarriageTransform)
{
    OpenRCT2::Vehicle car{};
    car.x = 100;
    car.y = 200;
    car.z = 30;
    car.orientation = 0;
    car.pitch = VehiclePitch::up25;
    car.roll = VehicleRoll::right45;
    // Avoid any ride-object lookup; cable-lift visual state is a static native
    // CarEntry and is sufficient for testing the presentation clock itself.
    car.ride_subtype = 0xFFFF;
    car.ride = RideId::GetNull();

    OpenRCT2::FirstPersonTrackedVehicleVisuals tracked{};
    tracked.yawBefore = 0;
    tracked.yawAfter = 8;
    tracked.pitchBefore =
        static_cast<uint8_t>(VehiclePitch::flat);
    tracked.pitchAfter =
        static_cast<uint8_t>(VehiclePitch::up25);
    tracked.rollBefore =
        static_cast<uint8_t>(VehicleRoll::unbanked);
    tracked.rollAfter =
        static_cast<uint8_t>(VehicleRoll::right45);
    tracked.alpha = 0.5f;

    const auto state =
        BuildFirstPersonVehiclePresentationState(
            car, tracked);
    const float yaw =
        FirstPersonLerpAngle(
            FirstPersonVehicleYawRadians(tracked.yawBefore),
            FirstPersonVehicleYawRadians(tracked.yawAfter),
            0.5f);
    const float pitch =
        FirstPersonLerpAngle(
            FirstPersonVehiclePitchRadians(
                VehiclePitch::flat),
            FirstPersonVehiclePitchRadians(
                VehiclePitch::up25),
            0.5f);
    const float roll =
        FirstPersonLerpAngle(
            FirstPersonVehicleRollRadians(
                VehicleRoll::unbanked),
            FirstPersonVehicleRollRadians(
                VehicleRoll::right45),
            0.5f);
    const auto expected =
        BuildFirstPersonCarriageTransform(
            car,
            FirstPersonVehicleTrackBasis(
                car, yaw, pitch, roll),
            0.0f, 0.0f, 0.0f);

    EXPECT_NEAR(
        state.carriage.basis.forward.x,
        expected.basis.forward.x, 1e-5f);
    EXPECT_NEAR(
        state.carriage.basis.forward.y,
        expected.basis.forward.y, 1e-5f);
    EXPECT_NEAR(
        state.carriage.basis.forward.z,
        expected.basis.forward.z, 1e-5f);
    EXPECT_NEAR(
        state.carriage.basis.up.x,
        expected.basis.up.x, 1e-5f);
    EXPECT_NEAR(
        state.carriage.basis.up.y,
        expected.basis.up.y, 1e-5f);
    EXPECT_NEAR(
        state.carriage.basis.up.z,
        expected.basis.up.z, 1e-5f);
}

TEST(FirstPersonSemanticComponentTest, OrientedBeamBoundsPreserveDiagonalGeometry)
{
    FirstPersonPaintSemanticComponent component{};
    component.geometry.kind =
        FirstPersonPaintSemanticPrimitiveKind::beam;
    component.geometry.points[0] = { 0.0f, 0.0f, 0.0f };
    component.geometry.points[1] = { 10.0f, 0.0f, 0.0f };
    component.geometry.pointCount = 2;
    component.geometry.halfWidth = 1.0f;
    component.geometry.halfHeight = 1.0f;
    component.transform.origin = { 100.0f, 200.0f, 10.0f };
    component.transform.axisX = { 0.0f, 1.0f, 0.0f };
    component.transform.axisY = { -1.0f, 0.0f, 0.0f };
    component.transform.axisZ = { 0.0f, 0.0f, 1.0f };

    FirstPersonVec3 low{}, high{};
    ASSERT_TRUE(
        FirstPersonSemanticComponentBounds(
            component, low, high));
    EXPECT_NEAR(low.x, 99.0f, 1e-5f);
    EXPECT_NEAR(high.x, 101.0f, 1e-5f);
    EXPECT_NEAR(low.y, 199.0f, 1e-5f);
    EXPECT_NEAR(high.y, 211.0f, 1e-5f);
    EXPECT_NEAR(low.z, 9.0f, 1e-5f);
    EXPECT_NEAR(high.z, 11.0f, 1e-5f);
}

TEST(FirstPersonSemanticComponentTest, TileRegistryPublishesOneGenericContract)
{
    ClearFirstPersonSemanticComponents();
    const CoordsXY tile{ 64, 96 };

    FirstPersonPaintSemanticComponent component{};
    component.role = FirstPersonPaintSemanticRole::stationFence;
    component.geometry.kind =
        FirstPersonPaintSemanticPrimitiveKind::box;
    component.geometry.points[0] = { 0.0f, 0.0f, 0.0f };
    component.geometry.points[1] = { 32.0f, 1.0f, 7.0f };
    component.geometry.pointCount = 2;
    component.transform.origin = {
        float(tile.x), float(tile.y), 0.0f
    };

    PublishFirstPersonSemanticComponents(
        tile, { component });
    const auto* published =
        GetFirstPersonSemanticComponents(tile);
    ASSERT_NE(published, nullptr);
    ASSERT_EQ(published->size(), 1u);
    EXPECT_EQ(
        published->front().role,
        FirstPersonPaintSemanticRole::stationFence);

    ClearFirstPersonSemanticComponents();
    EXPECT_EQ(
        GetFirstPersonSemanticComponents(tile),
        nullptr);
}

TEST(FirstPersonPhysicalProxyTest, BankedRectangularRailRotatesWidthIntoVerticalExtent)
{
    const FirstPersonBasis banked{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
        { 0.0f, -1.0f, 0.0f },
    };
    const FirstPersonRailProxySegment rail{
        { 0.0f, 0.0f, 10.0f },
        { 10.0f, 0.0f, 10.0f },
        banked,
        banked,
        2.0f,
        0.25f,
        FirstPersonPhysicalProxyProvenance::
            authoritativeTrackTrajectory,
    };

    // At 90 degrees of bank, the 2-unit local width is vertical. Treating
    // halfHeight as a world-Z extent would incorrectly miss this contact.
    EXPECT_TRUE(FirstPersonRailProxyIntersectsWalkStep(
        rail,
        { 5.0f, -2.0f, 11.5f },
        { 5.0f, 0.0f, 11.5f },
        0.2f, 0.0f));
    EXPECT_FALSE(FirstPersonRailProxyIntersectsWalkStep(
        rail,
        { 5.0f, -2.0f, 12.2f },
        { 5.0f, 0.0f, 12.2f },
        0.2f, 0.0f));
}

TEST(FirstPersonPhysicalProxyTest, LargeSceneryCollisionUsesRecoveredFacesNotReservedVolume)
{
    ClearFirstPersonLargeSceneryPhysicalProxies();
    const uint64_t groupKey = 17;
    const std::vector<FirstPersonPhysicalBoxProxy> faces{
        {
            { 10.0f, 0.0f, 0.0f },
            { 11.0f, 32.0f, 32.0f },
            FirstPersonPhysicalProxyProvenance::
                authoritativeLargeSceneryOccupancy,
            static_cast<uint8_t>(
                FirstPersonPhysicalProxyCapability::collide),
            groupKey,
        },
    };
    PublishFirstPersonLargeSceneryPhysicalProxies(
        groupKey, faces);

    EXPECT_TRUE(
        FirstPersonLargeSceneryProxyGroupIntersectsWalkStep(
            groupKey,
            { 0.0f, 16.0f, 0.0f },
            { 20.0f, 16.0f, 0.0f }));
    // Space inside the construction footprint but away from the recovered
    // material face remains traversable.
    EXPECT_FALSE(
        FirstPersonLargeSceneryProxyGroupIntersectsWalkStep(
            groupKey,
            { 20.0f, 4.0f, 0.0f },
            { 24.0f, 4.0f, 0.0f }));

    ClearFirstPersonLargeSceneryPhysicalProxies();
}

TEST(FirstPersonPhysicalProxyTest, AuthoritativeRailGeometryFeedsWalkingCollision)
{
    FirstPersonTrackTrajectory trajectory{};
    const FirstPersonBasis basis{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };
    trajectory.points = {
        { { 0.0f, 0.0f, 10.0f }, basis, 0 },
        { { 12.0f, 0.0f, 10.0f }, basis, 1 },
    };

    FirstPersonTrackRailProfile profile{};
    profile.halfGauge = 2.0f;
    profile.halfWidth = 0.5f;
    profile.halfHeight = 0.5f;

    const auto rails =
        BuildFirstPersonRailProxySegments(trajectory, profile);
    ASSERT_EQ(rails.size(), 2u);
    EXPECT_FLOAT_EQ(rails[0].a.y, -2.0f);
    EXPECT_FLOAT_EQ(rails[1].a.y, 2.0f);

    EXPECT_TRUE(FirstPersonRailProxyIntersectsWalkStep(
        rails[0],
        { 6.0f, -5.0f, 0.0f },
        { 6.0f, 0.0f, 0.0f },
        20.0f));
    EXPECT_FALSE(FirstPersonRailProxyIntersectsWalkStep(
        rails[0],
        { 6.0f, -5.0f, -30.0f },
        { 6.0f, 0.0f, -30.0f },
        10.0f));

    // Starting inside the collision radius must not trap the walker.
    EXPECT_FALSE(FirstPersonRailProxyIntersectsWalkStep(
        rails[0],
        { 6.0f, -2.0f, 0.0f },
        { 6.0f, -7.0f, 0.0f },
        20.0f));
}


TEST(FirstPersonPaintProvenanceTest, InteractionOwnerDoesNotMakeTileArtworkDynamic)
{
    OpenRCT2::EntityBase owner{};
    PaintStruct root{};
    root.Entity = &owner;

    root.Source = PaintStructSource::tile;
    EXPECT_FALSE(IsFirstPersonEntityPaintRoot(root));

    root.Source = PaintStructSource::entity;
    EXPECT_TRUE(IsFirstPersonEntityPaintRoot(root));
}

TEST(FirstPersonTerrainTextureTest, ChosenSourceViewKeepsKnownDegenerateSlopesTwoDimensional)
{
    const auto minimumTriangleArea = [](uint8_t slope, uint8_t rotation) {
        const auto corners = OpenRCT2::GetSlopeCornerHeights(0, slope);
        const std::array<CoordsXYZ, 4> world{ {
            { 0, 0, corners.south },
            { kCoordsXYStep, 0, corners.east },
            { kCoordsXYStep, kCoordsXYStep, corners.north },
            { 0, kCoordsXYStep, corners.west },
        } };
        std::array<ScreenCoordsXY, 4> projected{};
        for (size_t i = 0; i < world.size(); ++i)
            projected[i] = Translate3DTo2DWithZ(rotation, world[i]);

        const auto area = [&](size_t a, size_t b, size_t c) {
            const int64_t abx = int64_t(projected[b].x) - projected[a].x;
            const int64_t aby = int64_t(projected[b].y) - projected[a].y;
            const int64_t acx = int64_t(projected[c].x) - projected[a].x;
            const int64_t acy = int64_t(projected[c].y) - projected[a].y;
            const int64_t value = abx * acy - aby * acx;
            return uint64_t(value < 0 ? -value : value);
        };
        return std::min(area(0, 1, 3), area(1, 2, 3));
    };

    for (const uint8_t slope : { uint8_t{ 1 }, uint8_t{ 27 } })
    {
        EXPECT_EQ(minimumTriangleArea(slope, 0), 0u);
        const auto rotation = GetFirstPersonTerrainSourceRotation(slope);
        EXPECT_GT(minimumTriangleArea(slope, rotation), 0u);
    }
}

TEST(FirstPersonTunnelGeometryTest, NativeLeftAndRightEdgesRotateIntoWorldEdges)
{
    EXPECT_EQ(
        FirstPersonLeftTunnelWorldEdge(0),
        FirstPersonTunnelEdge::xMax);
    EXPECT_EQ(
        FirstPersonLeftTunnelWorldEdge(1),
        FirstPersonTunnelEdge::yMax);
    EXPECT_EQ(
        FirstPersonLeftTunnelWorldEdge(2),
        FirstPersonTunnelEdge::xMin);
    EXPECT_EQ(
        FirstPersonLeftTunnelWorldEdge(3),
        FirstPersonTunnelEdge::yMin);

    EXPECT_EQ(
        FirstPersonRightTunnelWorldEdge(0),
        FirstPersonTunnelEdge::yMax);
    EXPECT_EQ(
        FirstPersonRightTunnelWorldEdge(1),
        FirstPersonTunnelEdge::xMin);
    EXPECT_EQ(
        FirstPersonRightTunnelWorldEdge(2),
        FirstPersonTunnelEdge::yMin);
    EXPECT_EQ(
        FirstPersonRightTunnelWorldEdge(3),
        FirstPersonTunnelEdge::xMax);
}

TEST(FirstPersonTunnelGeometryTest, PortalUsesSharedNativeDescriptorHeight)
{
    const TunnelEntry entry{
        5, TunnelType::pathAndMiniGolf
    };
    const auto portal =
        BuildFirstPersonTunnelPortal(
            { 64, 96 },
            FirstPersonTunnelEdge::xMin,
            entry);
    const auto& descriptor =
        GetTunnelDescriptor(
            TunnelType::pathAndMiniGolf);
    EXPECT_EQ(portal.lowZ, 5 * kCoordsZPerTinyZ);
    EXPECT_EQ(
        portal.highZ - portal.lowZ,
        int32_t(descriptor.height)
            * kCoordsZPerTinyZ);
    EXPECT_EQ(
        portal.boundBoxZOffset,
        descriptor.boundBoxZOffset);
    EXPECT_EQ(
        portal.boundBoxLength,
        descriptor.boundBoxLength);
}

TEST(FirstPersonTunnelGeometryTest, TerrainWallLeavesPortalApertureEmpty)
{
    FirstPersonTunnelPortal portal{};
    portal.tile = { 0, 0 };
    portal.edge = FirstPersonTunnelEdge::xMin;
    portal.lowZ = 16;
    portal.highZ = 48;

    const auto quads =
        BuildFirstPersonTunnelPortalWall(
            portal, 0.0f, 0.0f,
            64.0f, 64.0f);
    ASSERT_EQ(quads.size(), 2u);
    for (const auto& corner : quads[0].corners)
        EXPECT_LE(corner.z, 16.0f);
    for (const auto& corner : quads[1].corners)
        EXPECT_GE(corner.z, 48.0f);
}

TEST(FirstPersonTunnelTest, VerticalTunnelMarkerCutsMatchingTerrainOnly)
{
    EXPECT_TRUE(FirstPersonVerticalTunnelCutsTerrain(80, 5));
    EXPECT_FALSE(FirstPersonVerticalTunnelCutsTerrain(96, 5));
    EXPECT_FALSE(FirstPersonVerticalTunnelCutsTerrain(80, 0xFF));
}

TEST(FirstPersonWalkingTest, NativeStepAllowedButCliffRejected)
{
    EXPECT_TRUE(FirstPersonWalkingHeightTransitionAllowed(100.0f, 108.0f));
    EXPECT_TRUE(FirstPersonWalkingHeightTransitionAllowed(108.0f, 100.0f));
    EXPECT_FALSE(FirstPersonWalkingHeightTransitionAllowed(100.0f, 116.0f));
    EXPECT_FALSE(FirstPersonWalkingHeightTransitionAllowed(116.0f, 100.0f));
}

TEST(FirstPersonTerrainCacheTest, DirtyRevalidationRestoresPacketMembership)
{
    EXPECT_TRUE(
        FirstPersonTerrainRevalidationNeedsRegionRebuild(
            true, false));
    EXPECT_TRUE(
        FirstPersonTerrainRevalidationNeedsRegionRebuild(
            false, true));
    EXPECT_FALSE(
        FirstPersonTerrainRevalidationNeedsRegionRebuild(
            false, false));
}

TEST(FirstPersonProjectionTest, ForwardPointProjectsToCentre)
{
    FirstPersonCamera camera{};
    const auto result = ProjectFirstPersonPoint(camera, { 100.0f, 0.0f, 0.0f }, kScreen);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x, 500.0f, 0.01f);
    EXPECT_NEAR(result->y, 300.0f, 0.01f);
}

TEST(FirstPersonProjectionTest, RightAndUpProjectInExpectedDirections)
{
    FirstPersonCamera camera{};

    const auto right = ProjectFirstPersonPoint(camera, { 100.0f, 10.0f, 0.0f }, kScreen);
    const auto up = ProjectFirstPersonPoint(camera, { 100.0f, 0.0f, 10.0f }, kScreen);

    ASSERT_TRUE(right.has_value());
    ASSERT_TRUE(up.has_value());
    EXPECT_GT(right->x, 500.0f);
    EXPECT_LT(up->y, 300.0f);
}

TEST(FirstPersonProjectionTest, RejectsNearOrBehindCamera)
{
    FirstPersonCamera camera{};

    EXPECT_FALSE(ProjectFirstPersonPoint(camera, { -10.0f, 0.0f, 0.0f }, kScreen).has_value());
    EXPECT_FALSE(ProjectFirstPersonPoint(camera, { 2.0f, 0.0f, 0.0f }, kScreen, 70.0f, 4.0f).has_value());
}

TEST(FirstPersonProjectionTest, DoubleDepthHalvesScale)
{
    FirstPersonCamera camera{};

    const auto near = ProjectFirstPersonPoint(camera, { 100.0f, 0.0f, 0.0f }, kScreen);
    const auto far = ProjectFirstPersonPoint(camera, { 200.0f, 0.0f, 0.0f }, kScreen);

    ASSERT_TRUE(near.has_value());
    ASSERT_TRUE(far.has_value());
    EXPECT_NEAR(far->scale, near->scale * 0.5f, 0.0001f);
}

TEST(FirstPersonProjectionTest, YawRotatesViewBasis)
{
    FirstPersonCamera camera{};
    camera.yaw = kPi * 0.5f;

    const auto result = ProjectFirstPersonPoint(camera, { 0.0f, 100.0f, 0.0f }, kScreen);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->x, 500.0f, 0.01f);
    EXPECT_NEAR(result->y, 300.0f, 0.01f);
}

TEST(FirstPersonStreamingTest, ResolvedViewUsesCompleteParkFarPlane)
{
    FirstPersonCamera camera{};
    const auto view = ResolveFirstPersonView(
        camera, 1600, 900, 1024, 1024, 70.0f, 2.0f, 32768.0f);

    EXPECT_NEAR(view.aspect, 1600.0f / 900.0f, 0.000001f);
    EXPECT_FLOAT_EQ(view.nearClip, 2.0f);
    EXPECT_GT(view.farClip, 46000.0f);
    EXPECT_FLOAT_EQ(
        view.farClip,
        CompleteParkFarClip(camera.position, 1024, 1024, 32768.0f));
}

TEST(FirstPersonProjectionTest, PassengerHeadYawTurnsRelativeToCar)
{
    FirstPersonCamera car{};
    car.yaw = kPi * 0.5f;
    const auto b = GetFirstPersonBasis(car);
    const auto head = GetPassengerHeadBasis(b, kPi * 0.5f, 0.0f);
    EXPECT_NEAR(head.forward.x, -1.0f, 0.0001f);
    EXPECT_NEAR(head.forward.y, 0.0f, 0.0001f);
    EXPECT_NEAR(FpDot(head.forward, head.right), 0.0f, 0.0001f);
}

TEST(FirstPersonProjectionTest, InversionAndHeadMovementDoNotLevelHorizon)
{
    FirstPersonCamera inverted{};
    inverted.roll = kPi;
    const auto car = GetFirstPersonBasis(inverted);
    const auto head = GetPassengerHeadBasis(car, kPi * 0.5f, 0.25f);
    EXPECT_LT(head.forward.y, 0.0f);
    EXPECT_LT(head.up.z, 0.0f);
    EXPECT_NEAR(FpDot(head.forward, head.right), 0.0f, 0.0001f);
    EXPECT_NEAR(FpDot(head.forward, head.up), 0.0f, 0.0001f);
    EXPECT_NEAR(FpDot(head.right, head.up), 0.0f, 0.0001f);
}

TEST(FirstPersonProjectionTest, HeadResetMatchesCarPose)
{
    FirstPersonCamera car{};
    car.yaw = 1.0f;
    car.pitch = -0.7f;
    car.roll = 1.9f;
    const auto b = GetFirstPersonBasis(car);
    const auto h = GetPassengerHeadBasis(b, 0.0f, 0.0f);
    EXPECT_NEAR(FpDot(h.forward, b.forward), 1.0f, 0.0001f);
    EXPECT_NEAR(FpDot(h.right, b.right), 1.0f, 0.0001f);
    EXPECT_NEAR(FpDot(h.up, b.up), 1.0f, 0.0001f);
}

TEST(FirstPersonProjectionTest, SpinnerUsesEightBitFullTurn)
{
    EXPECT_NEAR(SpinSpriteYawRadians(0), 0.0f, 0.0001f);
    EXPECT_NEAR(SpinSpriteYawRadians(64), kPi * 0.5f, 0.0001f);
    EXPECT_NEAR(SpinSpriteYawRadians(128), kPi, 0.0001f);
    EXPECT_NEAR(SpinSpriteYawRadians(192), kPi * 1.5f, 0.0001f);
}

TEST(FirstPersonVisibilityTest, ElevatedEntityIsVisibleEvenWhenUnderlyingTerrainIsNot)
{
    FirstPersonCamera camera{};
    camera.position = { 0.0f, 0.0f, 1000.0f };

    EXPECT_TRUE(FirstPersonSphereVisible(
        camera, { 100.0f, 0.0f, 1000.0f }, 64.0f,
        70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
    EXPECT_FALSE(FirstPersonSphereVisible(
        camera, { 100.0f, 0.0f, 0.0f }, 304.0f,
        70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
}

TEST(FirstPersonVisibilityTest, TallLandmarkNotCulledWhenBaseIsOutOfFrame)
{
    FirstPersonCamera camera{};
    camera.pitch = kPi / 4.0f;
    // Its base is below the camera's upward-facing field of view, while
    // its upper structure remains visible. Test the WHOLE bounding volume.
    EXPECT_TRUE(FirstPersonSphereVisible(camera, { 100.0f, 0.0f, 100.0f },
                                          110.0f, 70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
    EXPECT_FALSE(FirstPersonSphereVisible(camera, { -500.0f, 0.0f, -500.0f },
                                           20.0f, 70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
}

TEST(FirstPersonVisibilityTest, GroundRemainsVisibleWhenLookingStraightDownOrInverted)
{
    FirstPersonCamera camera{};
    camera.position = { 0.0f, 0.0f, 400.0f };
    camera.pitch = -kPi / 2.0f;
    EXPECT_TRUE(FirstPersonSphereVisible(camera, { 0.0f, 0.0f, 0.0f },
                                          24.0f, 70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
    camera.roll = kPi;
    EXPECT_TRUE(FirstPersonSphereVisible(camera, { 0.0f, 0.0f, 0.0f },
                                          24.0f, 70.0f, 1000.0f / 600.0f, 2.0f, 8192.0f));
}

TEST(FirstPersonVisibilityTest, ObjectsJustCrossingNearPlaneRemainEligible)
{
    FirstPersonCamera camera{};
    EXPECT_TRUE(FirstPersonSphereVisible(camera, { 0.0f, 0.0f, 0.0f },
                                          12.0f, 70.0f, 1.5f, 2.0f, 8192.0f));
    EXPECT_FALSE(FirstPersonSphereVisible(camera, { -30.0f, 0.0f, 0.0f },
                                           4.0f, 70.0f, 1.5f, 2.0f, 8192.0f));
}

TEST(FirstPersonWallGeometryTest, SharedSemanticCornersMatchWorldPlane)
{
    const CoordsXY origin{ 96, 160 };
    constexpr int32_t baseZ = 80;
    constexpr int32_t height = 48;
    constexpr uint8_t direction = 1;
    constexpr uint8_t slope = EDGE_SLOPE_UPWARDS;

    const auto local =
        FirstPersonWallSemanticCorners(
            direction, slope, height);
    const auto world =
        BuildFirstPersonWallPlane(
            origin, baseZ, direction, slope, height);

    for (size_t i = 0; i < local.size(); ++i)
    {
        EXPECT_FLOAT_EQ(
            world.corners[i].x,
            float(origin.x) + local[i].x);
        EXPECT_FLOAT_EQ(
            world.corners[i].y,
            float(origin.y) + local[i].y);
        EXPECT_FLOAT_EQ(
            world.corners[i].z,
            float(baseZ) + local[i].z);
    }
}

TEST(FirstPersonWallGeometryTest, UsesCompleteTileEdgesWithoutPainterInsets)
{
    const CoordsXY origin{ 64, 96 };
    const auto westEdge = BuildFirstPersonWallPlane(origin, 80, 0, 0, 40);
    EXPECT_FLOAT_EQ(westEdge.corners[0].x, 64.0f);
    EXPECT_FLOAT_EQ(westEdge.corners[0].y, 96.0f);
    EXPECT_FLOAT_EQ(westEdge.corners[1].x, 64.0f);
    EXPECT_FLOAT_EQ(westEdge.corners[1].y, 128.0f);
    EXPECT_FLOAT_EQ(westEdge.corners[0].z, 80.0f);
    EXPECT_FLOAT_EQ(westEdge.corners[2].z, 120.0f);

    const auto northEdge = BuildFirstPersonWallPlane(origin, 80, 1, 0, 40);
    EXPECT_FLOAT_EQ(northEdge.corners[0].x, 64.0f);
    EXPECT_FLOAT_EQ(northEdge.corners[0].y, 128.0f);
    EXPECT_FLOAT_EQ(northEdge.corners[1].x, 96.0f);
    EXPECT_FLOAT_EQ(northEdge.corners[1].y, 128.0f);
    // The adjacent edges meet at the exact tile corner.
    EXPECT_FLOAT_EQ(westEdge.corners[1].x, northEdge.corners[0].x);
    EXPECT_FLOAT_EQ(westEdge.corners[1].y, northEdge.corners[0].y);
}

TEST(FirstPersonWallGeometryTest, NativeSlopeRaisesTheCorrectEndpoint)
{
    const CoordsXY origin{ 32, 64 };
    const auto upwards = BuildFirstPersonWallPlane(origin, 100, 1, EDGE_SLOPE_UPWARDS, 48);
    EXPECT_FLOAT_EQ(upwards.corners[0].z, 100.0f);
    EXPECT_FLOAT_EQ(upwards.corners[1].z, 116.0f);
    EXPECT_FLOAT_EQ(upwards.corners[3].z, 148.0f);
    EXPECT_FLOAT_EQ(upwards.corners[2].z, 164.0f);

    const auto downwards = BuildFirstPersonWallPlane(origin, 100, 1, EDGE_SLOPE_DOWNWARDS, 48);
    EXPECT_FLOAT_EQ(downwards.corners[0].z, 116.0f);
    EXPECT_FLOAT_EQ(downwards.corners[1].z, 100.0f);
    EXPECT_FLOAT_EQ(downwards.corners[3].z, 164.0f);
    EXPECT_FLOAT_EQ(downwards.corners[2].z, 148.0f);
}

TEST(FirstPersonWallGeometryTest, SlopedCollisionUsesHeightAtActualContactPoint)
{
    const FirstPersonVec3 wallA{ 0.0f, 0.0f, 100.0f };
    const FirstPersonVec3 wallB{ 32.0f, 0.0f, 116.0f };

    // At x=8 the sloped wall base is z=104. An eye ending exactly at
    // z=104 does not overlap the wall and must be allowed underneath it.
    EXPECT_FALSE(FirstPersonSlopedWallIntersectsWalkStep(
        { 8.0f, -4.0f, 84.0f }, { 8.0f, 4.0f, 84.0f },
        wallA, wallB, 48.0f, 20.0f, 2.0f));

    // One unit higher produces a real vertical overlap at the same XY contact.
    EXPECT_TRUE(FirstPersonSlopedWallIntersectsWalkStep(
        { 8.0f, -4.0f, 85.0f }, { 8.0f, 4.0f, 85.0f },
        wallA, wallB, 48.0f, 20.0f, 2.0f));
}

TEST(FirstPersonPathArtworkTest, SemanticDeckUsesNativeRotationAdjustedSpriteOrigin)
{
    const CoordsXY origin{ 320, 640 };
    constexpr int32_t z = 80;
    const std::array<ScreenCoordsXY, 4> expectedWrongMinusNative{ {
        { 0, 0 },
        { -32, -16 },
        { 0, -32 },
        { 32, -16 },
    } };

    for (uint8_t rotation = 0; rotation < 4; ++rotation)
    {
        const auto adjusted = GetTileElementPaintSpritePosition(origin, rotation);
        const auto wrong = Translate3DTo2DWithZ(rotation, { origin, z });
        const auto native = Translate3DTo2DWithZ(rotation, { adjusted, z });
        EXPECT_EQ(wrong.x - native.x, expectedWrongMinusNative[rotation].x);
        EXPECT_EQ(wrong.y - native.y, expectedWrongMinusNative[rotation].y);
    }
}

TEST(FirstPersonPathArtworkTest, SharedNativeSurfaceSelectionRotatesConsistently)
{
    OpenRCT2::PathElement path{};
    path.setEdges(0b0001);
    path.setCorners(0);
    path.setIsQueue(false);
    path.setSloped(false);

    EXPECT_EQ(GetPathSurfaceImageOffset(path, 0), 1);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 1), 2);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 2), 4);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 3), 8);

    path.setSloped(true);
    path.setSlopeDirection(2);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 0), 18);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 1), 19);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 2), 16);
    EXPECT_EQ(GetPathSurfaceImageOffset(path, 3), 17);
}

TEST(FirstPersonVehiclePoseTest, NativeYawMatchesMovementConventionWithoutQuantising)
{
    constexpr std::array<uint8_t, 4> orientations{ 0, 8, 16, 24 };
    constexpr float kExpectedX[4]{ -1.0f, 0.0f, 1.0f, 0.0f };
    constexpr float kExpectedY[4]{ 0.0f, 1.0f, 0.0f, -1.0f };

    for (size_t i = 0; i < orientations.size(); ++i)
    {
        const auto orientation = orientations[i];
        const auto movement = OpenRCT2::RideVehicle::Geometry::getFreeroamVehicleMovementData(orientation);
        const auto yaw = FirstPersonVehicleYawRadians(orientation);
        const FirstPersonCamera camera{ {}, yaw, 0.0f, 0.0f };
        const auto basis = GetFirstPersonBasis(camera);

        const float movementLength = std::hypot(float(movement.x), float(movement.y));
        ASSERT_GT(movementLength, 0.0f);
        EXPECT_NEAR(basis.forward.x, float(movement.x) / movementLength, 0.000001f);
        EXPECT_NEAR(basis.forward.y, float(movement.y) / movementLength, 0.000001f);
        EXPECT_NEAR(basis.forward.x, kExpectedX[i], 0.000001f);
        EXPECT_NEAR(basis.forward.y, kExpectedY[i], 0.000001f);
    }

    // Orientation 3 is between the free-roam table's 8-way headings. The
    // camera must retain the native 32-step angle instead of snapping to 45°.
    const auto yaw3 = FirstPersonVehicleYawRadians(3);
    const auto basis3 = GetFirstPersonBasis(FirstPersonCamera{ {}, yaw3, 0.0f, 0.0f });
    const float theta3 = 3.0f * (2.0f * kPi / 32.0f);
    EXPECT_NEAR(basis3.forward.x, -std::cos(theta3), 0.000001f);
    EXPECT_NEAR(basis3.forward.y, std::sin(theta3), 0.000001f);
}

TEST(FirstPersonVehiclePoseTest, SpecialPitchStatesUseAuthoritativeGeometry)
{
    for (const auto pitch : {
             VehiclePitch::corkscrewUpRight0,
             VehiclePitch::corkscrewUpRight2,
             VehiclePitch::upHalfHelixLarge,
             VehiclePitch::downQuarterHelix,
             VehiclePitch::uninvertingDown25,
             VehiclePitch::curvedLiftHillUp,
         })
    {
        const auto direction = OpenRCT2::RideVehicle::Geometry::getPitchVector32(pitch);
        const auto expected = std::atan2(static_cast<float>(direction.y), static_cast<float>(direction.x));
        EXPECT_NEAR(FirstPersonVehiclePitchRadians(pitch), expected, 0.000001f);
    }
    EXPECT_NE(FirstPersonVehiclePitchRadians(VehiclePitch::corkscrewUpRight0), 0.0f);
    EXPECT_NE(FirstPersonVehiclePitchRadians(VehiclePitch::curvedLiftHillUp), 0.0f);
}

TEST(FirstPersonVehiclePoseTest, UninvertingRollStatesPreservePhysicalBank)
{
    EXPECT_NEAR(FirstPersonVehicleRollRadians(VehicleRoll::uninvertingLeft22), -kPi / 8.0f, 0.000001f);
    EXPECT_NEAR(FirstPersonVehicleRollRadians(VehicleRoll::uninvertingLeft45), -kPi / 4.0f, 0.000001f);
    EXPECT_NEAR(FirstPersonVehicleRollRadians(VehicleRoll::uninvertingRight22), kPi / 8.0f, 0.000001f);
    EXPECT_NEAR(FirstPersonVehicleRollRadians(VehicleRoll::uninvertingRight45), kPi / 4.0f, 0.000001f);
}


TEST(FirstPersonVehiclePoseTest, LocalSpinUsesCarriageUpAxisOnIncline)
{
    FirstPersonCamera track{};
    track.pitch = kPi / 6.0f;
    const auto basis = GetFirstPersonBasis(track);
    const auto spun = FirstPersonRotateLocalYaw(basis, kPi / 2.0f);

    EXPECT_NEAR(spun.forward.x, 0.0f, 0.000001f);
    EXPECT_NEAR(spun.forward.y, 1.0f, 0.000001f);
    EXPECT_NEAR(spun.forward.z, 0.0f, 0.000001f);
    EXPECT_NEAR(spun.up.x, basis.up.x, 0.000001f);
    EXPECT_NEAR(spun.up.y, basis.up.y, 0.000001f);
    EXPECT_NEAR(spun.up.z, basis.up.z, 0.000001f);
}

TEST(FirstPersonVehiclePoseTest, ReversalIsLocalHalfTurnNotWorldEulerHack)
{
    FirstPersonCamera track{};
    track.yaw = 0.4f;
    track.pitch = 0.3f;
    track.roll = -0.2f;
    const auto basis = GetFirstPersonBasis(track);
    const auto reversed = FirstPersonRotateLocalYaw(basis, kPi);

    EXPECT_NEAR(reversed.forward.x, -basis.forward.x, 0.000001f);
    EXPECT_NEAR(reversed.forward.y, -basis.forward.y, 0.000001f);
    EXPECT_NEAR(reversed.forward.z, -basis.forward.z, 0.000001f);
    EXPECT_NEAR(reversed.up.x, basis.up.x, 0.000001f);
    EXPECT_NEAR(reversed.up.y, basis.up.y, 0.000001f);
    EXPECT_NEAR(reversed.up.z, basis.up.z, 0.000001f);
}


TEST(FirstPersonVehiclePoseTest, MultiDimensionSeatUsesNativeFrameMapAndLocalPitchAxis)
{
    EXPECT_EQ(FirstPersonMultiDimensionAnimationFrame(4, 8), 0);
    EXPECT_EQ(FirstPersonMultiDimensionAnimationFrame(5, 8), 1);
    EXPECT_EQ(FirstPersonMultiDimensionAnimationFrame(3, 8), 7);
    EXPECT_EQ(FirstPersonMultiDimensionAnimationFrame(12, 8), 0);

    EXPECT_NEAR(
        FirstPersonMultiDimensionSeatAngle(6, 2, 8),
        kPi / 2.0f, 0.000001f);
    // If semantic and rendered state momentarily disagree, the visible native
    // animation frame wins the camera pose.
    EXPECT_NEAR(
        FirstPersonMultiDimensionSeatAngle(6, 3, 8),
        3.0f * kPi / 4.0f, 0.000001f);

    FirstPersonCamera track{};
    track.yaw = 0.25f;
    track.pitch = 0.35f;
    const auto basis = GetFirstPersonBasis(track);
    const auto rotated =
        FirstPersonRotateLocalPitch(basis, kPi / 2.0f);
    EXPECT_NEAR(rotated.forward.x, basis.up.x, 0.000001f);
    EXPECT_NEAR(rotated.forward.y, basis.up.y, 0.000001f);
    EXPECT_NEAR(rotated.forward.z, basis.up.z, 0.000001f);
    EXPECT_NEAR(rotated.right.x, basis.right.x, 0.000001f);
    EXPECT_NEAR(rotated.right.y, basis.right.y, 0.000001f);
    EXPECT_NEAR(rotated.right.z, basis.right.z, 0.000001f);
}

TEST(FirstPersonVehiclePoseTest, MissingSeatPivotKeepsPassengerPositionNeutral)
{
    FirstPersonCamera camera{};
    const auto neutral = GetFirstPersonBasis(camera);
    FirstPersonCarriageTransform carriage{};
    carriage.positionBasis = neutral;
    carriage.basis =
        FirstPersonRotateLocalPitch(neutral, kPi / 2.0f);
    carriage.orientationOnlySeatRotation = true;

    constexpr FirstPersonVec3 origin{ 10.0f, 20.0f, 30.0f };
    constexpr FirstPersonVec3 eye{ 5.0f, -2.0f, 9.0f };
    const auto safe = FirstPersonPassengerEyeForCarriage(
        origin, carriage, eye, false);
    const auto expected =
        FirstPersonPassengerEye(origin, neutral, eye);
    EXPECT_NEAR(safe.x, expected.x, 0.0001f);
    EXPECT_NEAR(safe.y, expected.y, 0.0001f);
    EXPECT_NEAR(safe.z, expected.z, 0.0001f);

    carriage.orientationOnlySeatRotation = false;
    const auto rotated = FirstPersonPassengerEyeForCarriage(
        origin, carriage, eye, false);
    EXPECT_GT(
        std::hypot(rotated.x - expected.x, rotated.z - expected.z),
        1.0f);
}

TEST(FirstPersonVehiclePoseTest, NativePassengerAnchorOwnsPositionWithoutRideName)
{
    Vehicle vehicle{};
    vehicle.num_seats = 2;

    PassengerPaintAnchor anchor{};
    anchor.Entity = &vehicle;
    anchor.seatMask = 1u;
    anchor.x = 100.0f;
    anchor.y = 200.0f;
    anchor.z = 300.0f;
    anchor.hasEyeOffset = true;
    anchor.eyeForward = 2.0f;
    anchor.eyeRight = -3.0f;
    anchor.eyeUp = 10.0f;

    FirstPersonCamera camera{};
    const auto pose =
        BuildFirstPersonPassengerPose(
            vehicle,
            { 1.0f, 2.0f, 3.0f },
            GetFirstPersonBasis(camera),
            &anchor, 0);

    ASSERT_TRUE(pose.supported);
    EXPECT_TRUE(pose.rideSpecificTransform);
    EXPECT_FLOAT_EQ(pose.position.x, 102.0f);
    EXPECT_FLOAT_EQ(pose.position.y, 197.0f);
    EXPECT_FLOAT_EQ(pose.position.z, 310.0f);
}

TEST(FirstPersonVehiclePoseTest, MissingAnchorAndSeatCalibrationIsUnsupported)
{
    Vehicle vehicle{};
    vehicle.num_seats = 1;
    FirstPersonCamera camera{};

    const auto pose =
        BuildFirstPersonPassengerPose(
            vehicle,
            { 1.0f, 2.0f, 3.0f },
            GetFirstPersonBasis(camera),
            nullptr, 0);
    EXPECT_FALSE(pose.supported);
}

TEST(FirstPersonVehiclePoseTest, RideSpecificTranslationSurvivesCarriageFallback)
{
    FirstPersonCamera camera{};
    const auto neutral = GetFirstPersonBasis(camera);
    FirstPersonCarriageTransform carriage{};
    carriage.positionBasis = neutral;
    carriage.basis =
        FirstPersonRotateLocalPitch(neutral, kPi / 2.0f);
    carriage.orientationOnlySeatRotation = true;

    FirstPersonPassengerPose pose{};
    pose.position = { 101.0f, 202.0f, 303.0f };
    pose.localEyeOffset = { 4.0f, -2.0f, 9.0f };
    pose.rideSpecificTransform = true;
    const auto completed = pose.position;

    ApplyFirstPersonPassengerCarriagePositionFallback(
        pose, { 10.0f, 20.0f, 30.0f }, carriage);
    EXPECT_FLOAT_EQ(pose.position.x, completed.x);
    EXPECT_FLOAT_EQ(pose.position.y, completed.y);
    EXPECT_FLOAT_EQ(pose.position.z, completed.z);

    pose.rideSpecificTransform = false;
    ApplyFirstPersonPassengerCarriagePositionFallback(
        pose, { 10.0f, 20.0f, 30.0f }, carriage);
    const auto expected = FirstPersonPassengerEye(
        { 10.0f, 20.0f, 30.0f },
        carriage.positionBasis, pose.localEyeOffset);
    EXPECT_NEAR(pose.position.x, expected.x, 0.0001f);
    EXPECT_NEAR(pose.position.y, expected.y, 0.0001f);
    EXPECT_NEAR(pose.position.z, expected.z, 0.0001f);
}

TEST(FirstPersonVehiclePoseTest, SwingCalibrationFitsOneRigidFourViewOrbit)
{
    std::array<FirstPersonVec3, 13> points{};
    constexpr FirstPersonVec3 pivot{ 3.0f, -2.0f, 21.0f };
    constexpr float radius = 17.0f;
    constexpr std::array<float, 3> angles{ 0.12f, 0.29f, 0.51f };

    const auto pointAt = [&](float angle) {
        return FirstPersonVec3{
            pivot.x,
            pivot.y + radius * std::sin(angle),
            pivot.z - radius * std::cos(angle),
        };
    };
    points[0] = pointAt(0.0f);
    for (size_t level = 0; level < angles.size(); ++level)
    {
        points[1 + level * 2] = pointAt(-angles[level]);
        points[2 + level * 2] = pointAt(angles[level]);
    }

    const auto calibration = FitFirstPersonSwingCalibration(
        points, 3, FirstPersonAssetMarkerEvidence::riderPrimary, 0.2f);
    ASSERT_TRUE(calibration.valid);
    EXPECT_EQ(calibration.pairCount, 3);
    EXPECT_NEAR(calibration.pivotLocal.x, pivot.x, 0.0001f);
    EXPECT_NEAR(calibration.pivotLocal.y, pivot.y, 0.0001f);
    EXPECT_NEAR(calibration.pivotLocal.z, pivot.z, 0.0001f);
    EXPECT_NEAR(calibration.radius, radius, 0.0001f);
    for (size_t level = 0; level < angles.size(); ++level)
    {
        EXPECT_NEAR(
            calibration.positiveAngles[level + 1],
            angles[level], 0.0001f);
    }

    // A moving forward coordinate is not a roll-axis pendulum, even though
    // individual 2-D sprite pairs could still look symmetric.
    points[4].x += 8.0f;
    EXPECT_FALSE(
        FitFirstPersonSwingCalibration(
            points, 3,
            FirstPersonAssetMarkerEvidence::riderPrimary).valid);
}

TEST(FirstPersonPassengerAssetCalibrationTest, OppositeViewPairsActAsHoldoutValidation)
{
    constexpr FirstPersonVec3 nativePoint{ -6.0f, 4.0f, 13.0f };
    std::array<FirstPersonRiderChannelObservation, 4> views{};
    for (uint8_t direction = 0; direction < 4; ++direction)
    {
        const auto projected =
            ProjectFirstPersonLocalIso(direction, nativePoint);
        views[direction] = {
            true, projected[0], projected[1], 4.0f, 6.0f
        };
    }
    const auto pairA =
        RecoverFirstPersonLocalPointFromOppositeViews(views, 0);
    const auto pairB =
        RecoverFirstPersonLocalPointFromOppositeViews(views, 1);
    ASSERT_TRUE(pairA.has_value());
    ASSERT_TRUE(pairB.has_value());
    EXPECT_NEAR(pairA->x, pairB->x, 0.0001f);
    EXPECT_NEAR(pairA->y, pairB->y, 0.0001f);
    EXPECT_NEAR(pairA->z, pairB->z, 0.0001f);

    views[3].x += 7.0f;
    const auto contradicted =
        RecoverFirstPersonLocalPointFromOppositeViews(views, 1);
    ASSERT_TRUE(contradicted.has_value());
    EXPECT_GT(
        std::hypot(
            contradicted->x - pairA->x,
            contradicted->y - pairA->y),
        3.0f);
}

TEST(FirstPersonVehiclePoseTest, SwingCalibrationUsesNativeThresholdBands)
{
    FirstPersonSwingCalibration calibration{};
    calibration.valid = true;
    calibration.pairCount = 6;
    calibration.positiveAngles = {
        0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f
    };
    calibration.radius = 20.0f;

    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(910), 0);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(911), 1);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(2730), 1);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(2731), 2);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(4550), 2);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(4551), 3);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(5460), 3);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(6371), 4);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(8191), 5);
    EXPECT_EQ(OpenRCT2::VehicleSwingLevelForPosition(10011), 6);
    EXPECT_EQ(OpenRCT2::VehicleSwingSpriteForPosition(-911), 1);
    EXPECT_EQ(OpenRCT2::VehicleSwingSpriteForPosition(911), 2);
    EXPECT_EQ(OpenRCT2::VehicleSwingSpriteForPosition(-5460), 5);
    EXPECT_EQ(OpenRCT2::VehicleSwingSpriteForPosition(5460), 6);

    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 1820.0f),
        0.1f, 0.000001f);
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 3640.0f),
        0.2f, 0.000001f);
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, -5460.0f),
        -0.3f, 0.000001f);
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 10920.0f),
        0.6f, 0.000001f);

    // At native sprite boundaries, continuous SwingPosition lies halfway
    // between the adjacent calibrated category centres.
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 910.0f),
        0.05f, 0.000001f);
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 2730.0f),
        0.15f, 0.000001f);
    EXPECT_NEAR(
        FirstPersonSwingAngleForPosition(calibration, 4550.0f),
        0.25f, 0.000001f);
}


TEST(FirstPersonVehiclePoseTest, CyclicPassengerFrameInterpolationTakesShortestWrap)
{
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(47.0f, 0.0f, 0.5f, 48.0f), 47.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(0.0f, 47.0f, 0.5f, 48.0f), 47.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(15.0f, 0.0f, 0.5f, 16.0f), 15.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(0.0f, 1.0f, 0.5f, 16.0f), 0.5f, 0.0001f);
}

TEST(FirstPersonFerrisVisibilityTest, SelectedTileRiderSuppressesOnlyItself)
{
    EXPECT_EQ(
        FirstPersonHiddenComponentPolicy(true, false, true),
        FirstPersonHiddenComponentDisposition::suppressSelfContinueChain);
    EXPECT_EQ(
        FirstPersonHiddenComponentPolicy(true, true, false),
        FirstPersonHiddenComponentDisposition::suppressSubtree);
    EXPECT_EQ(
        FirstPersonHiddenComponentPolicy(true, false, false),
        FirstPersonHiddenComponentDisposition::emit);

    // Early and middle rider pairs are both component suppressions, never
    // subtree suppressions; later gondolas/supports must remain traversable.
    const auto early =
        FirstPersonFerrisWheelRiderImageIndex(1000, 0, 20, 0);
    const auto middle =
        FirstPersonFerrisWheelRiderImageIndex(1000, 0, 20, 8);
    EXPECT_NE(early, middle);
    EXPECT_TRUE(
        FirstPersonFerrisWheelImageMatchesSeatPair(
            1000, early, 20, 0));
    EXPECT_FALSE(
        FirstPersonFerrisWheelImageMatchesSeatPair(
            1000, middle, 20, 0));
    EXPECT_TRUE(
        FirstPersonFerrisWheelImageMatchesSeatPair(
            1000, middle, 20, 8));
}

TEST(FirstPersonPassengerAssetCalibrationTest, FourRiderViewsRecoverCarLocalEyePoint)
{
    constexpr FirstPersonVec3 worldLocal{ -7.0f, 3.0f, 11.0f };
    std::array<FirstPersonRiderChannelObservation, 4> views{};
    for (uint8_t direction = 0; direction < 4; ++direction)
    {
        const auto projected =
            ProjectFirstPersonLocalIso(direction, worldLocal);
        views[direction] = {
            true, projected[0], projected[1], 5.0f, 8.0f
        };
    }

    float rmse = -1.0f;
    const auto recovered =
        RecoverFirstPersonLocalPointFromFourViews(views, &rmse);
    ASSERT_TRUE(recovered.has_value());
    EXPECT_NEAR(recovered->x, worldLocal.x, 0.000001f);
    EXPECT_NEAR(recovered->y, worldLocal.y, 0.000001f);
    EXPECT_NEAR(recovered->z, worldLocal.z, 0.000001f);
    EXPECT_NEAR(rmse, 0.0f, 0.000001f);
}

TEST(FirstPersonPassengerAssetCalibrationTest, RiderColourChannelsAreDistinctEvidence)
{
    EXPECT_TRUE(FirstPersonRiderPixelUsesPrimaryRemap(
        static_cast<uint8_t>(OpenRCT2::Drawing::PaletteIndex::primaryRemap5)));
    EXPECT_FALSE(FirstPersonRiderPixelUsesPrimaryRemap(
        static_cast<uint8_t>(OpenRCT2::Drawing::PaletteIndex::secondaryRemap5)));
    EXPECT_TRUE(FirstPersonRiderPixelUsesSecondaryRemap(
        static_cast<uint8_t>(OpenRCT2::Drawing::PaletteIndex::secondaryRemap5)));
    EXPECT_FALSE(FirstPersonRiderPixelUsesSecondaryRemap(
        static_cast<uint8_t>(OpenRCT2::Drawing::PaletteIndex::primaryRemap5)));
}


TEST(FirstPersonPassengerAssetCalibrationTest, MultiDimensionArtworkRecoversRightAxisPivotAndWinding)
{
    std::array<FirstPersonVec3, 16> points{};
    constexpr uint8_t frames = 8;
    constexpr FirstPersonVec3 pivot{ 3.0f, -4.0f, 7.0f };
    constexpr float radius = 12.0f;
    for (uint8_t frame = 0; frame < frames; ++frame)
    {
        const float phase =
            float(frame) * (2.0f * kPi / float(frames));
        points[frame] = {
            pivot.x + radius * std::cos(phase),
            pivot.y,
            pivot.z + radius * std::sin(phase),
        };
    }

    const auto calibration =
        FitFirstPersonMultiDimensionArtworkCalibration(
            points, frames, 0.5f);
    ASSERT_TRUE(calibration.valid);
    EXPECT_EQ(calibration.angleSign, 1);
    EXPECT_NEAR(calibration.pivotLocal.x, pivot.x, 0.0001f);
    EXPECT_NEAR(calibration.pivotLocal.y, pivot.y, 0.0001f);
    EXPECT_NEAR(calibration.pivotLocal.z, pivot.z, 0.0001f);
    EXPECT_NEAR(calibration.radius, radius, 0.0001f);
    EXPECT_GE(calibration.uncertainty, 0.5f);

    // Lateral movement means the artwork does not describe rotation about the
    // carriage's local right axis, so the calibration must be rejected.
    points[3].y += 8.0f;
    EXPECT_FALSE(
        FitFirstPersonMultiDimensionArtworkCalibration(
            points, frames).valid);
}

TEST(FirstPersonPeriodicPassengerMotionTest, FerrisRiderPhaseMatchesNativePairStride)
{
    EXPECT_FLOAT_EQ(FirstPersonFerrisWheelRiderPhase(7.0f, 0), 7.0f);
    EXPECT_FLOAT_EQ(FirstPersonFerrisWheelRiderPhase(7.0f, 1), 7.0f);
    EXPECT_FLOAT_EQ(FirstPersonFerrisWheelRiderPhase(7.0f, 2), 15.0f);
    EXPECT_FLOAT_EQ(FirstPersonFerrisWheelRiderPhase(127.0f, 2), 7.0f);
    EXPECT_EQ(
        FirstPersonFerrisWheelRiderImageIndex(1000, 2, 127, 2),
        1000u + 32u + 2u * 128u + 7u);
    EXPECT_EQ(
        FirstPersonFerrisWheelRiderImageIndex(1000, 2, 127, 3),
        FirstPersonFerrisWheelRiderImageIndex(1000, 2, 127, 2));
    EXPECT_NEAR(
        FirstPersonLerpCyclicFrame(127.0f, 0.0f, 0.5f, 128.0f),
        127.5f, 0.0001f);
}

TEST(FirstPersonPeriodicPassengerMotionTest, FerrisPainterOriginStaysAlignedForEveryRideDirection)
{
    constexpr FirstPersonPeriodicOrbitPoint painterOriginSeat{ 0.0f, 0.0f, 0.0f };
    constexpr std::array<FirstPersonPeriodicOrbitPoint, 4> expected{ {
        { -32.0f, -16.0f, 4.0f },
        { -16.0f, 0.0f, 4.0f },
        { 0.0f, -16.0f, 4.0f },
        { -16.0f, -32.0f, 4.0f },
    } };

    for (uint8_t direction = 0; direction < 4; ++direction)
    {
        const auto offset = FirstPersonFerrisWheelSeatBaseOffsetFromVehicle(
            painterOriginSeat, direction, 3.0f);
        EXPECT_FLOAT_EQ(offset.x, expected[direction].x);
        EXPECT_FLOAT_EQ(offset.y, expected[direction].y);
        EXPECT_FLOAT_EQ(offset.z, expected[direction].z);
    }
}

TEST(FirstPersonPeriodicPassengerMotionTest, FourViewsRecoverConstrainedFerrisOrbit)
{
    FirstPersonFerrisWheelObservationSet observations{};
    constexpr float radius = 42.0f;
    constexpr float centreX = 6.0f;
    constexpr float centreY = -3.0f;
    constexpr float centreZ = 61.0f;
    constexpr float phaseOffset = 0.61f;
    const float cosX = radius * std::cos(phaseOffset);
    const float cosZ = radius * std::sin(phaseOffset);
    const float sinX = -radius * std::sin(phaseOffset);
    const float sinZ = radius * std::cos(phaseOffset);

    for (uint8_t direction = 0; direction < 4; ++direction)
    for (size_t sample = 0; sample < kFirstPersonFerrisWheelSampleCount; ++sample)
    {
        const float phase = float(sample * kFirstPersonFerrisWheelSampleStride)
            * (2.0f * kPi / float(kFirstPersonFerrisWheelFrameCount));
        const float x = centreX + cosX * std::cos(phase) + sinX * std::sin(phase);
        const float y = centreY;
        const float z = centreZ + cosZ * std::cos(phase) + sinZ * std::sin(phase);

        float sx = 0.0f;
        float sy = 0.0f;
        switch (direction)
        {
            case 0:
                sx = y - x;
                sy = 0.5f * (x + y) - z;
                break;
            case 1:
                sx = -x - y;
                sy = 0.5f * (y - x) - z;
                break;
            case 2:
                sx = x - y;
                sy = -0.5f * (x + y) - z;
                break;
            case 3:
                sx = x + y;
                sy = 0.5f * (x - y) - z;
                break;
        }
        observations[direction][sample] = {
            true, sx, sy,
            12.0f + 0.4f * std::cos(2.0f * phase),
            16.0f + 0.5f * std::sin(2.0f * phase),
        };
    }

    const auto calibration = FitFirstPersonFerrisWheelOrbit(observations);
    ASSERT_TRUE(calibration.valid);
    EXPECT_NEAR(calibration.centerX, centreX, 0.001f);
    EXPECT_NEAR(calibration.centerY, centreY, 0.001f);
    EXPECT_NEAR(calibration.centerZ, centreZ, 0.001f);
    EXPECT_NEAR(calibration.radius, radius, 0.001f);
    EXPECT_LT(calibration.trainingRmse, 0.001f);
    EXPECT_LT(calibration.validationRmse, 0.001f);

    const auto phaseZero = SampleFirstPersonPeriodicOrbit(calibration, 0.0f);
    EXPECT_NEAR(phaseZero.x, centreX + cosX, 0.001f);
    EXPECT_NEAR(phaseZero.z, centreZ + cosZ, 0.001f);
    EXPECT_GT(calibration.eyeHeight, 6.0f);
    EXPECT_GT(calibration.eyeHeightUncertainty, 0.0f);
}

TEST(FirstPersonPeriodicPassengerMotionTest, ContradictoryHeldOutViewRejectsOrbit)
{
    FirstPersonFerrisWheelObservationSet observations{};
    constexpr float radius = 40.0f;
    for (uint8_t direction = 0; direction < 4; ++direction)
    for (size_t sample = 0; sample < kFirstPersonFerrisWheelSampleCount; ++sample)
    {
        float phase = float(sample * kFirstPersonFerrisWheelSampleStride)
            * (2.0f * kPi / float(kFirstPersonFerrisWheelFrameCount));
        if (direction == 3)
            phase += 0.5f * kPi;
        const float x = radius * std::cos(phase);
        const float y = 0.0f;
        const float z = 64.0f + radius * std::sin(phase);

        float sx = 0.0f;
        float sy = 0.0f;
        switch (direction)
        {
            case 0: sx = y - x; sy = 0.5f * (x + y) - z; break;
            case 1: sx = -x - y; sy = 0.5f * (y - x) - z; break;
            case 2: sx = x - y; sy = -0.5f * (x + y) - z; break;
            case 3: sx = x + y; sy = 0.5f * (x - y) - z; break;
        }
        observations[direction][sample] = { true, sx, sy, 12.0f, 16.0f };
    }

    const auto calibration = FitFirstPersonFerrisWheelOrbit(observations);
    EXPECT_FALSE(calibration.valid);
}


TEST(FirstPersonPeriodicPassengerMotionTest, HeldOutSecondHarmonicCannotHideInCoefficientFit)
{
    FirstPersonFerrisWheelObservationSet observations{};
    constexpr float radius = 40.0f;
    for (uint8_t direction = 0; direction < 4; ++direction)
    for (size_t sample = 0; sample < kFirstPersonFerrisWheelSampleCount; ++sample)
    {
        const float phase = float(sample * kFirstPersonFerrisWheelSampleStride)
            * (2.0f * kPi / float(kFirstPersonFerrisWheelFrameCount));
        const float x = radius * std::cos(phase);
        const float y = 0.0f;
        const float z = 64.0f + radius * std::sin(phase);

        float sx = 0.0f;
        float sy = 0.0f;
        switch (direction)
        {
            case 0: sx = y - x; sy = 0.5f * (x + y) - z; break;
            case 1: sx = -x - y; sy = 0.5f * (y - x) - z; break;
            case 2: sx = x - y; sy = -0.5f * (x + y) - z; break;
            case 3: sx = x + y; sy = 0.5f * (x - y) - z; break;
        }
        if (direction >= 2)
            sx += 40.0f * std::cos(2.0f * phase);
        observations[direction][sample] = { true, sx, sy, 12.0f, 16.0f };
    }

    const auto calibration = FitFirstPersonFerrisWheelOrbit(observations);
    EXPECT_FALSE(calibration.valid);
    EXPECT_GT(calibration.validationRmse, 10.0f);
}


namespace
{
    // The worker is deliberately blocked while the render-side APIs run. A
    // synchronous reconstruction/join regression makes this test time out.
    struct FirstPersonWorkerGate
    {
        std::promise<void> release;
        std::shared_future<void> ready = release.get_future().share();
        ~FirstPersonWorkerGate() { release.set_value(); }
    };
    template<typename Predicate>
    bool AwaitFirstPersonResult(FirstPersonWorkQueue& queue, Predicate&& done)
    {
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!done() && std::chrono::steady_clock::now() < end)
        {
            FirstPersonFrameBudget budget(2, std::chrono::milliseconds(1));
            queue.publish(budget);
            std::this_thread::yield();
        }
        return done();
    }
    bool SubmitFirstPersonTestWork(FirstPersonWorkQueue& queue, FirstPersonWorkQueue::Work work)
    {
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!queue.submit(work))
        {
            if (std::chrono::steady_clock::now() >= end) return false;
            std::this_thread::yield();
        }
        return true;
    }
}

TEST(FirstPersonAsyncTest, BlockedWorkerDoesNotBlockPublicationOrParkReset)
{
    FirstPersonWorkQueue queue;
    std::promise<void> started;
    auto start = started.get_future();
    bool committed = false;
    {
        FirstPersonWorkerGate gate;
        ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [ready = gate.ready, &started, &committed] {
            started.set_value();
            ready.wait();
            return [&committed] { committed = true; };
        }));
        ASSERT_EQ(start.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        FirstPersonFrameBudget budget(2, std::chrono::milliseconds(1));
        queue.publish(budget);
        EXPECT_FALSE(committed);
        const auto epoch = queue.epoch();
        queue.cancel();
        EXPECT_NE(queue.epoch(), epoch);
        EXPECT_FALSE(committed);
    }
    bool nextParkCommitted = false;
    ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [&] { return [&] { nextParkCommitted = true; }; }));
    EXPECT_TRUE(AwaitFirstPersonResult(queue, [&] { return nextParkCommitted; }));
    EXPECT_FALSE(committed);
}

TEST(FirstPersonAsyncTest, SnapshotOutlivesSourceAndPublishesOnlyOnCallerThread)
{
    FirstPersonSpriteSnapshot snapshot;
    FirstPersonFrameBudget capture(4, std::chrono::seconds(5));
    std::vector<uint8_t> pixels{1, 2, 3, 4};
    OpenRCT2::G1Element source{};
    source.width = source.height = 2;
    source.xOffset = -1;
    source.offset = pixels.data();
    ASSERT_TRUE(snapshot.capture(17, &source, capture));
    pixels.assign(4, 99);
    source.width = 1;
    EXPECT_EQ(snapshot.get(17)->width, 2);
    EXPECT_EQ(snapshot.get(17)->offset[3], 4);
    EXPECT_EQ(snapshot.get(17)->xOffset, -1);
    EXPECT_EQ(snapshot.get(18), nullptr); // no fallback to live G1 state

    FirstPersonWorkQueue queue;
    const auto renderThread = std::this_thread::get_id();
    std::thread::id workerThread, publicationThread;
    bool published = false;
    ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [snapshot, &workerThread, &publicationThread, &published] {
        workerThread = std::this_thread::get_id();
        const auto value = snapshot.get(17)->offset[3];
        return [value, &publicationThread, &published] {
            EXPECT_EQ(value, 4);
            publicationThread = std::this_thread::get_id();
            published = true;
        };
    }));
    EXPECT_TRUE(AwaitFirstPersonResult(queue, [&] { return published; }));
    EXPECT_NE(workerThread, renderThread);
    EXPECT_EQ(publicationThread, renderThread);
}

TEST(FirstPersonAsyncTest, BackpressureDoesNotExecuteRejectedWorkInline)
{
    FirstPersonWorkQueue queue;
    FirstPersonWorkerGate gate;
    std::promise<void> started;
    auto start = started.get_future();
    std::atomic<size_t> executed{0};
    ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [ready = gate.ready, &started] {
        started.set_value();
        ready.wait();
        return [] {};
    }));
    ASSERT_EQ(start.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    for (size_t i = 0; i < 7; ++i)
        ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [&] { ++executed; return [] {}; }));
    EXPECT_FALSE(queue.hasCapacity());
    EXPECT_FALSE(queue.submit([&] { ++executed; return [] {}; }));
    EXPECT_EQ(executed.load(), 0u);
    queue.cancel();
}

TEST(FirstPersonAsyncTest, BudgetedPublicationPreservesRemainingCompletions)
{
    FirstPersonWorkQueue queue;
    size_t completed = 0;
    ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [&] { return [&] { ++completed; }; }));
    ASSERT_TRUE(SubmitFirstPersonTestWork(queue, [&] { return [&] { ++completed; }; }));
    FirstPersonFrameBudget noUploads(0, std::chrono::seconds(5));
    queue.publish(noUploads);
    EXPECT_EQ(completed, 0u);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (completed == 0 && std::chrono::steady_clock::now() < end)
    {
        FirstPersonFrameBudget one(1, std::chrono::seconds(5));
        queue.publish(one);
        std::this_thread::yield();
    }
    EXPECT_EQ(completed, 1u);
    EXPECT_TRUE(AwaitFirstPersonResult(queue, [&] { return completed == 2; }));
}

TEST(FirstPersonAsyncTest, ExpiredCaptureBudgetLeavesSnapshotRetryable)
{
    FirstPersonSpriteSnapshot snapshot;
    uint8_t pixel = 42;
    OpenRCT2::G1Element source{};
    source.width = source.height = 1;
    source.offset = &pixel;
    FirstPersonFrameBudget exhausted(0, std::chrono::seconds(5));
    EXPECT_FALSE(snapshot.capture(10, &source, exhausted));
    EXPECT_EQ(snapshot.get(10), nullptr);
    FirstPersonFrameBudget nextFrame(1, std::chrono::seconds(5));
    EXPECT_TRUE(snapshot.capture(10, &source, nextFrame));
    EXPECT_EQ(snapshot.get(10)->offset[0], 42);
    EXPECT_TRUE(snapshot.capture(10, &source, exhausted)); // already owned
}
