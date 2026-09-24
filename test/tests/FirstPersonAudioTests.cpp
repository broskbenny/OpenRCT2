/****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 ****************************************************************************/
#include <gtest/gtest.h>
#include <openrct2/audio/FirstPersonSpatialAudio.h>

using namespace OpenRCT2::Audio;

namespace
{
    FirstPersonAudioListener Ear() { return { { 0, 0, 0 }, { 0, 1, 0 } }; }
}

TEST(FirstPersonAudioTest, NoNearFieldStereoSnap)
{
    auto ear = Ear();
    auto left = CalculateFirstPersonSpatialParams(ear, { 0, -0.001f, 0 });
    auto right = CalculateFirstPersonSpatialParams(ear, { 0, 0.001f, 0 });
    EXPECT_LT(std::abs(left.pan), 10);
    EXPECT_EQ(left.pan, -right.pan);
    EXPECT_EQ(CalculateFirstPersonSpatialParams(ear, { 0, 0, 0 }).pan, 0);
    EXPECT_EQ(CalculateFirstPersonSpatialParams(ear, { 0, 16, 0 }).pan, 10000);
}

TEST(FirstPersonAudioTest, RejectSilentVehiclesBeforeSoundSlotAllocation)
{
    auto ear = Ear();
    const auto close = CalculateFirstPersonSpatialParams(ear, { 0, 256, 0 });
    const auto distant = CalculateFirstPersonSpatialParams(ear, { 0, 1280, 0 });
    ASSERT_GT(close.vehicleVolume, 0);
    ASSERT_TRUE(distant.inRange);
    EXPECT_EQ(distant.vehicleVolume, 0);
    EXPECT_GT(FirstPersonVehiclePriority(1000, false, close.vehicleVolume),
              FirstPersonVehiclePriority(1000000, true, distant.vehicleVolume));
}

TEST(FirstPersonAudioTest, RideMusicFadesBeyondLegacyHardCutoff)
{
    FirstPersonSpatialParams params{};
    params.inRange = true;
    params.volume = -4500;
    EXPECT_TRUE(IsFirstPersonRideMusicAudible(params));
    params.volume = -8000;
    EXPECT_FALSE(IsFirstPersonRideMusicAudible(params));
}

TEST(FirstPersonAudioTest, WeightedCrowdRespectsQueueAndDistance)
{
    EXPECT_EQ(FirstPersonCrowdWeight(255, false), 510);
    EXPECT_EQ(FirstPersonCrowdWeight(255, true), 255);
    EXPECT_EQ(FirstPersonCrowdWeight(0, false), 0);
    EXPECT_LT(FirstPersonCrowdWeight(23, false), FirstPersonCrowdWeight(255, true));
}

