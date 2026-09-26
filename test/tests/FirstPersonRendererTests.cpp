/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <algorithm>
#include <openrct2/paint/FirstPersonRenderer.h>
#include <openrct2/paint/FirstPersonTrackTrajectory.h>
#include <openrct2/paint/Paint.h>
#include <openrct2/entity/EntityBase.h>
#include <openrct2/world/tile_element/Slope.h>
#include <openrct2/paint/FirstPersonVehiclePose.h>
#include <openrct2/paint/tile_element/Paint.Path.h>
#include <openrct2/paint/tile_element/Paint.TileElement.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/world/Wall.h>
#include <openrct2/world/tile_element/PathElement.h>

#include <cmath>

using namespace OpenRCT2::Paint;

namespace
{
    constexpr ScreenSize kScreen{ 1000, 600 };
    constexpr float kPi = 3.14159265358979323846f;
}

TEST(FirstPersonSourceRotationTest, CameraRelativePointOwnsNativeQuadrant)
{
    FirstPersonCamera camera{};
    // Both points lie inside the same 32x32 world tile, but their azimuths
    // from the camera straddle the native 45-degree source-view boundary.
    EXPECT_EQ(FirstPersonSourceRotationForPoint(camera, { 30.0f, 28.0f, 0.0f }), 0);
    EXPECT_EQ(FirstPersonSourceRotationForPoint(camera, { 1.0f, 31.0f, 0.0f }), 1);
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

TEST(FirstPersonTrackTrajectoryTest, StandardSamplesMatchVehicleMotionSource)
{
    constexpr FirstPersonVec3 origin{ 320.0f, 640.0f, 80.0f };
    const auto trajectory = BuildFirstPersonTrackTrajectory(
        OpenRCT2::TrackElemType::flat, 0, origin);
    ASSERT_TRUE(trajectory.has_value());

    const size_t index = size_t(EnumValue(OpenRCT2::TrackElemType::flat))
        * kNumOrthogonalDirections;
    const auto* list = gTrackVehicleInfo[
        EnumValue(OpenRCT2::VehicleTrackSubposition::standard)][index];
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


TEST(FirstPersonVehiclePoseTest, CyclicPassengerFrameInterpolationTakesShortestWrap)
{
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(47.0f, 0.0f, 0.5f, 48.0f), 47.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(0.0f, 47.0f, 0.5f, 48.0f), 47.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(15.0f, 0.0f, 0.5f, 16.0f), 15.5f, 0.0001f);
    EXPECT_NEAR(FirstPersonLerpCyclicFrame(0.0f, 1.0f, 0.5f, 16.0f), 0.5f, 0.0001f);
}
