/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <openrct2/paint/FirstPersonRenderer.h>
#include <openrct2/paint/FirstPersonVehiclePose.h>
#include <openrct2/paint/tile_element/Paint.Path.h>
#include <openrct2/world/Wall.h>
#include <openrct2/world/tile_element/PathElement.h>

#include <cmath>

using namespace OpenRCT2::Paint;

namespace
{
    constexpr ScreenSize kScreen{ 1000, 600 };
    constexpr float kPi = 3.14159265358979323846f;
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
    const OpenRCT2::CoordsXY origin{ 64, 96 };
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
    const OpenRCT2::CoordsXY origin{ 32, 64 };
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

