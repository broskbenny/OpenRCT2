/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Audio.h"

#include "../Context.h"
#include "../OpenRCT2.h"
#include "../PlatformEnvironment.h"
#include "../config/Config.h"
#include "../core/EnumUtils.hpp"
#include "../core/File.h"
#include "../core/FileStream.h"
#include "../core/String.hpp"
#include "../entity/Peep.h"
#include "../interface/Viewport.h"
#include "../localisation/Language.h"
#include "../localisation/StringIds.h"
#include "../object/AudioObject.h"
#include "../object/ObjectManager.h"
#include "../ride/Ride.h"
#include "../ride/RideAudio.h"
#include "../scenes/intro/IntroScene.h"
#include "../ui/WindowManager.h"
#include "../util/Util.h"
#include "../world/Map.h"
#include "../world/Weather.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../paint/FirstPersonVehiclePose.h"
#include "../GameState.h"
#include "../ride/Vehicle.h"
#include "AudioChannel.h"
#include "AudioContext.h"
#include "AudioMixer.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace OpenRCT2::Audio
{
    struct AudioParams
    {
        bool in_range;
        int32_t volume;
        int32_t pan;
    };

    static std::vector<std::string> _audioDevices;
    static int32_t _currentAudioDevice = -1;
    static ObjectEntryIndex _soundsAudioObjectEntryIndex = kObjectEntryIndexNull;
    static ObjectEntryIndex _soundsAdditionalAudioObjectEntryIndex = kObjectEntryIndexNull;
    static ObjectEntryIndex _titleAudioObjectEntryIndex = kObjectEntryIndexNull;

    bool gGameSoundsOff = false;
    int32_t gVolumeAdjustZoom = 0;

    // Audio runs on the game thread. Walking owns a fixed listener, while ride
    // mode owns a vehicle attachment that is resolved from current simulation
    // state for every spatial query. This keeps emitters/listener in one time
    // domain even when the renderer temporarily tweens vehicles for display.
    struct FirstPersonActiveEffect
    {
        CoordsXYZ emitter;
        int32_t sampleModifier;
        std::shared_ptr<IAudioChannel> channel;
    };
    struct FirstPersonRideListenerAttachment
    {
        EntityId vehicleId = EntityId::GetNull();
        RideId rideId = RideId::GetNull();
        uint8_t seatIndex = 0;
        float headYaw = 0.0f;
        float headPitch = 0.0f;
    };
    static std::vector<FirstPersonActiveEffect> _firstPersonActiveEffects;
    static std::optional<FirstPersonAudioListener> _firstPersonListener;
    static std::optional<FirstPersonRideListenerAttachment> _firstPersonRideListener;

    static std::optional<FirstPersonAudioListener> ResolveFirstPersonAudioListener()
    {
        if (_firstPersonRideListener)
        {
            const auto& attachment = *_firstPersonRideListener;
            auto* vehicle = getGameState().entities.getEntity<Vehicle>(attachment.vehicleId);
            if (vehicle == nullptr || vehicle->ride != attachment.rideId)
                return std::nullopt;
            const auto passenger = Paint::FirstPersonVehicleSimulationPassengerPose(
                *vehicle, attachment.seatIndex);
            const auto headBasis = Paint::GetPassengerHeadBasis(
                passenger.basis, attachment.headYaw, attachment.headPitch);
            return FirstPersonAudioListener{
                { passenger.position.x, passenger.position.y, passenger.position.z },
                { headBasis.right.x, headBasis.right.y, headBasis.right.z }
            };
        }
        return _firstPersonListener;
    }

    static int32_t ApplyWorldSoundEnvironment(const CoordsXYZ& location, int32_t listenerAttenuation)
    {
        const auto* element = MapGetSurfaceElementAt(location);
        const bool underground = element != nullptr && element->getBaseZ() - 5 > location.z;
        if (!underground)
            return listenerAttenuation;

        // Native positional audio applies a 10-bit attenuation expansion to an
        // already-negative viewport term (at zoom 0 that term is -1024). A
        // first-person source can otherwise have attenuation 0 at the listener,
        // so retain the native minimum baseline before applying the SAME rule.
        const int32_t environmentalBase = std::min(listenerAttenuation, -1024);
        return ((environmentalBase - 1) * (1 << 10)) + 1;
    }

    static void StopFirstPersonEffects()
    {
        for (auto& effect : _firstPersonActiveEffects)
            if (effect.channel != nullptr)
                effect.channel->Stop();
        _firstPersonActiveEffects.clear();
    }

    static void RefreshFirstPersonEffects()
    {
        const auto listener = ResolveFirstPersonAudioListener();
        if (!listener)
            return;
        std::erase_if(_firstPersonActiveEffects, [&listener](const FirstPersonActiveEffect& effect) {
            if (effect.channel == nullptr || effect.channel->IsDone())
                return true;
            const auto spatial = CalculateFirstPersonSpatialParams(
                *listener, { float(effect.emitter.x), float(effect.emitter.y), float(effect.emitter.z) });
            // Temporarily mute distant sources. Do not stop them: the player
            // can walk/ride back into range while the original sample is playing.
            const int32_t volume = spatial.inRange
                ? std::clamp(
                      ApplyWorldSoundEnvironment(effect.emitter, spatial.volume)
                          + effect.sampleModifier,
                      -10000, 0)
                : -10000;
            effect.channel->SetVolume(DStoMixerVolume(volume));
            effect.channel->SetPan(DStoMixerPan(spatial.inRange ? spatial.pan : 0));
            return false;
        });
    }

    void SetFirstPersonAudioListener(const FirstPersonAudioListener& listener)
    {
        const auto& p = listener.position;
        const auto& r = listener.right;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)
            || !std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z)
            || r.x * r.x + r.y * r.y + r.z * r.z < 1e-8f)
        {
            ClearFirstPersonAudioListener();
            return;
        }
        const bool wasActive = HasFirstPersonAudioListener();
        _firstPersonRideListener.reset();
        _firstPersonListener = listener;
        if (!wasActive)
            PeepStopCrowdNoise();
        RefreshFirstPersonEffects();
    }

    void SetFirstPersonRideAudioListener(
        EntityId vehicleId, RideId rideId, uint8_t seatIndex,
        float headYaw, float headPitch)
    {
        if (vehicleId.IsNull() || rideId.IsNull() || !std::isfinite(headYaw)
            || !std::isfinite(headPitch))
        {
            ClearFirstPersonAudioListener();
            return;
        }
        const bool wasActive = HasFirstPersonAudioListener();
        _firstPersonListener.reset();
        _firstPersonRideListener = FirstPersonRideListenerAttachment{
            vehicleId, rideId, seatIndex, headYaw, headPitch
        };
        if (!wasActive)
            PeepStopCrowdNoise();
        // Do not refresh channels here: this can be called from the render path
        // while EntityTweener has temporarily moved the car to presentation time.
    }

    void RefreshFirstPersonSpatialAudio()
    {
        RefreshFirstPersonEffects();
    }

    void ClearFirstPersonAudioListener()
    {
        if (HasFirstPersonAudioListener())
            PeepStopCrowdNoise();
        StopFirstPersonEffects();
        _firstPersonListener.reset();
        _firstPersonRideListener.reset();
    }

    bool HasFirstPersonAudioListener()
    {
        return _firstPersonListener.has_value() || _firstPersonRideListener.has_value();
    }

    std::optional<FirstPersonAudioListener> GetFirstPersonAudioListener()
    {
        return ResolveFirstPersonAudioListener();
    }

    FirstPersonSpatialParams GetFirstPersonSpatialParams(const CoordsXYZ& source)
    {
        const auto listener = ResolveFirstPersonAudioListener();
        if (!listener)
            return {};
        return CalculateFirstPersonSpatialParams(
            *listener, { float(source.x), float(source.y), float(source.z) });
    }

    static std::shared_ptr<IAudioChannel> _titleMusicChannel = nullptr;

    VehicleSound gVehicleSoundList[kMaxVehicleSounds];

    bool IsAvailable()
    {
        if (_currentAudioDevice == -1)
            return false;
        if (gGameSoundsOff)
            return false;
        if (!Config::Get().sound.soundEnabled)
            return false;
        if (gOpenRCT2Headless)
            return false;
        return true;
    }

    void Init()
    {
        auto& audioContext = GetContext()->GetAudioContext();
        if (Config::Get().sound.device.empty())
        {
            audioContext.SetOutputDevice("");
            _currentAudioDevice = 0;
        }
        else
        {
            audioContext.SetOutputDevice(Config::Get().sound.device);

            PopulateDevices();
            for (int32_t i = 0; i < GetDeviceCount(); i++)
            {
                if (_audioDevices[i] == Config::Get().sound.device)
                {
                    _currentAudioDevice = i;
                }
            }
        }
    }

    void LoadAudioObjects()
    {
        auto& objManager = GetContext()->GetObjectManager();

        Object* baseAudio = objManager.LoadObject(AudioObjectIdentifiers::kRCT2);
        if (baseAudio != nullptr)
        {
            _soundsAudioObjectEntryIndex = objManager.GetLoadedObjectEntryIndex(baseAudio);
        }

        objManager.LoadObject(AudioObjectIdentifiers::kOpenRCT2Additional);
        _soundsAdditionalAudioObjectEntryIndex = objManager.GetLoadedObjectEntryIndex(
            AudioObjectIdentifiers::kOpenRCT2Additional);
        objManager.LoadObject(AudioObjectIdentifiers::kRCT2Circus);
    }

    void PopulateDevices()
    {
        auto& audioContext = GetContext()->GetAudioContext();
        std::vector<std::string> devices = audioContext.GetOutputDevices();

        // Replace blanks with localised unknown string
        for (auto& device : devices)
        {
            if (device.empty())
            {
                device = LanguageGetString(STR_OPTIONS_SOUND_VALUE_DEFAULT);
            }
        }

        // The first device is always system default
        std::string defaultDevice = LanguageGetString(STR_OPTIONS_SOUND_VALUE_DEFAULT);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
        devices.insert(devices.begin(), defaultDevice);
#pragma GCC diagnostic pop

        _audioDevices = devices;
    }

    /**
     * Returns the audio parameters to use when playing the specified sound at a virtual location.
     * @param soundId The sound effect to be played.
     * @param location The location at which the sound effect is to be played.
     * @return The audio parameters to be used when playing this sound effect.
     */
    static AudioParams GetParametersFromLocation(AudioObject* obj, uint32_t sampleIndex, const CoordsXYZ& location)
    {
        AudioParams params;
        params.in_range = true;
        params.volume = 0;
        params.pan = 0;

        const auto sampleModifier = obj->GetSampleModifier(sampleIndex);
        if (HasFirstPersonAudioListener())
        {
            const auto spatial = GetFirstPersonSpatialParams(location);
            params.in_range = spatial.inRange;
            params.volume = spatial.inRange
                ? std::clamp(
                      ApplyWorldSoundEnvironment(location, spatial.volume) + sampleModifier,
                      -10000, 0)
                : -10000;
            params.pan = spatial.pan;
            return params;
        }

        uint8_t rotation = GetCurrentRotation();
        auto pos2 = Translate3DTo2DWithZ(rotation, location);

        const auto& activeViewports = GetAllViewports();
        for (const auto& viewport : activeViewports)
        {
            if (viewport.flags & VIEWPORT_FLAG_SOUND_ON)
            {
                int16_t vx = pos2.x - viewport.viewPos.x;
                params.pan = viewport.pos.x + viewport.zoom.ApplyInversedTo(vx);

                const auto viewModifier = ApplyWorldSoundEnvironment(
                    location, viewport.zoom.ApplyTo(-1024));
                params.volume = sampleModifier + viewModifier;

                if (!viewport.Contains(pos2) || params.volume < -10000)
                {
                    params.in_range = false;
                    return params;
                }
            }
        }

        return params;
    }

    static std::tuple<AudioObject*, uint32_t> GetAudioObjectAndSampleIndex(SoundId id)
    {
        auto& objManager = GetContext()->GetObjectManager();
        AudioObject* audioObject{};
        uint32_t sampleIndex = EnumValue(id);
        if (id >= SoundId::liftRMC)
        {
            audioObject = objManager.GetLoadedObject<AudioObject>(_soundsAdditionalAudioObjectEntryIndex);
            sampleIndex -= EnumValue(SoundId::liftRMC);
        }
        else
        {
            audioObject = objManager.GetLoadedObject<AudioObject>(_soundsAudioObjectEntryIndex);
        }
        return std::make_tuple(audioObject, sampleIndex);
    }

    static void Play(IAudioSource* audioSource, int32_t volume, int32_t pan)
    {
        int32_t mixerPan = 0;
        if (pan != kAudioPlayAtCentre)
        {
            int32_t x2 = pan << 16;
            uint16_t screenWidth = std::max<int32_t>(64, ContextGetWidth());
            mixerPan = ((x2 / screenWidth) - 0x8000) >> 4;
        }

        CreateAudioChannel(audioSource, MixerGroup::sound, false, DStoMixerVolume(volume), DStoMixerPan(mixerPan), 1, true);
    }

    void Play3D(SoundId soundId, const CoordsXYZ& loc)
    {
        if (!IsAvailable())
            return;

        // Get sound from base object
        auto [baseAudioObject, sampleIndex] = GetAudioObjectAndSampleIndex(soundId);
        if (baseAudioObject != nullptr)
        {
            auto params = GetParametersFromLocation(baseAudioObject, sampleIndex, loc);
            if (params.in_range)
            {
                auto source = baseAudioObject->GetSample(sampleIndex);
                if (source != nullptr)
                {
                    if (HasFirstPersonAudioListener())
                    {
                        // Hold the channel and emitter, not a one-time snapshot
                        // of the player's head orientation. Existing 3D effects
                        // update via SetFirstPersonAudioListener each frame.
                        constexpr size_t kMaxTrackedEffects = 128;
                        const bool trackEffect = _firstPersonActiveEffects.size() < kMaxTrackedEffects;
                        auto channel = CreateAudioChannel(source, MixerGroup::sound, false,
                            DStoMixerVolume(params.volume), DStoMixerPan(params.pan), 1, !trackEffect);
                        if (channel != nullptr && trackEffect)
                        {
                            // Saturation degrades only spatial FOLLOWING for the
                            // new one-shot. No existing voice is stopped or evicted;
                            // the mixer auto-reaps the untracked new channel.
                            _firstPersonActiveEffects.push_back(
                                { loc, baseAudioObject->GetSampleModifier(sampleIndex), std::move(channel) });
                        }
                    }
                    else
                    {
                        Play(source, params.volume, params.pan);
                    }
                }
            }
        }
    }

    void Play(SoundId soundId, int32_t volume, int32_t pan)
    {
        if (!IsAvailable())
            return;

        // Get sound from base object
        auto [baseAudioObject, sampleIndex] = GetAudioObjectAndSampleIndex(soundId);
        if (baseAudioObject != nullptr)
        {
            auto source = baseAudioObject->GetSample(sampleIndex);
            if (source != nullptr)
            {
                Play(source, volume, pan);
            }
        }
    }

    static bool IsRCT1TitleMusicAvailable()
    {
        auto& env = GetContext()->GetPlatformEnvironment();
        auto rct1path = env.GetDirectoryPath(DirBase::rct1);
        return !rct1path.empty();
    }

    static std::map<TitleMusicKind, std::string_view> GetAvailableMusicMap()
    {
        auto musicMap = std::map<TitleMusicKind, std::string_view>{
            { TitleMusicKind::OpenRCT2, AudioObjectIdentifiers::kOpenRCT2Title },
            { TitleMusicKind::RCT2, AudioObjectIdentifiers::kRCT2Title },
        };

        if (IsRCT1TitleMusicAvailable())
        {
            musicMap.emplace(TitleMusicKind::RCT1, AudioObjectIdentifiers::kRCT1Title);
        }

        return musicMap;
    }

    static ObjectEntryDescriptor GetTitleMusicDescriptor(TitleMusicKind musicKind)
    {
        auto musicMap = GetAvailableMusicMap();
        auto it = musicMap.find(musicKind);
        if (musicKind == TitleMusicKind::random)
        {
            it = std::next(musicMap.begin(), UtilRand() % musicMap.size());
        }

        if (it != musicMap.end())
        {
            return ObjectEntryDescriptor(ObjectType::audio, it->second);
        }

        // No music descriptor for the current setting, intentional for TitleMusicKind::none
        return {};
    }

    void PlayTitleMusic()
    {
        if (gGameSoundsOff || gLegacyScene != LegacyScene::titleSequence || IntroIsPlaying())
        {
            StopTitleMusic();
            return;
        }

        if (_titleMusicChannel != nullptr && !_titleMusicChannel->IsDone())
        {
            return;
        }

        // Load title sequence audio object
        auto descriptor = GetTitleMusicDescriptor(Config::Get().sound.titleMusic);
        auto& objManager = GetContext()->GetObjectManager();
        auto* audioObject = static_cast<AudioObject*>(objManager.LoadObject(descriptor));
        if (audioObject != nullptr)
        {
            _titleAudioObjectEntryIndex = objManager.GetLoadedObjectEntryIndex(audioObject);

            // Play first sample from object
            auto source = audioObject->GetSample(0);
            if (source != nullptr)
            {
                _titleMusicChannel = CreateAudioChannel(source, MixerGroup::titleMusic, true);
            }
        }
    }

    void StopSFX()
    {
        StopFirstPersonEffects();
        StopVehicleSounds();
        PeepStopCrowdNoise();
        Weather::stopWeatherSound();
    }

    void StopAll()
    {
        StopSFX();
        StopTitleMusic();
        RideAudio::StopAllChannels();
    }

    int32_t GetDeviceCount()
    {
        return static_cast<int32_t>(_audioDevices.size());
    }

    const std::string& GetDeviceName(int32_t index)
    {
        if (index < 0 || index >= GetDeviceCount())
        {
            static std::string InvalidDevice = "Invalid Device";
            return InvalidDevice;
        }
        return _audioDevices[index];
    }

    int32_t GetCurrentDeviceIndex()
    {
        return _currentAudioDevice;
    }

    void StopTitleMusic()
    {
        if (_titleMusicChannel != nullptr)
        {
            _titleMusicChannel->Stop();
            _titleMusicChannel = nullptr;
        }

        // Unload the audio object
        if (_titleAudioObjectEntryIndex != kObjectEntryIndexNull)
        {
            auto& objManager = GetContext()->GetObjectManager();
            auto* obj = objManager.GetLoadedObject<AudioObject>(_titleAudioObjectEntryIndex);
            if (obj != nullptr)
            {
                objManager.UnloadObjects({ obj->GetDescriptor() });
            }
            _titleAudioObjectEntryIndex = kObjectEntryIndexNull;
        }
    }

    void InitRideSoundsAndInfo()
    {
        InitRideSounds(0);
    }

    void InitRideSounds(int32_t device)
    {
        Close();
        for (auto& vehicleSound : gVehicleSoundList)
        {
            vehicleSound.id = kSoundIdNull;
        }

        _currentAudioDevice = device;
        Config::Save();
    }

    void Close()
    {
        StopFirstPersonEffects();
        PeepStopCrowdNoise();
        StopTitleMusic();
        RideAudio::StopAllChannels();
        Weather::stopWeatherSound();
        _currentAudioDevice = -1;
    }

    void ToggleAllSounds()
    {
        Config::Get().sound.masterSoundEnabled = !Config::Get().sound.masterSoundEnabled;
        if (Config::Get().sound.masterSoundEnabled)
        {
            Resume();
        }
        else
        {
            Pause();
        }

        auto* windowMgr = Ui::GetWindowManager();
        windowMgr->InvalidateByClass(WindowClass::options);
    }

    void Pause()
    {
        gGameSoundsOff = true;
        StopAll();
    }

    void Resume()
    {
        gGameSoundsOff = !Config::Get().sound.masterSoundEnabled;
        PlayTitleMusic();
    }

    void StopVehicleSounds()
    {
        if (!IsAvailable())
            return;

        for (auto& vehicleSound : gVehicleSoundList)
        {
            if (vehicleSound.id != kSoundIdNull)
            {
                vehicleSound.id = kSoundIdNull;
                if (vehicleSound.trackSound.id != SoundId::null)
                {
                    vehicleSound.trackSound.channel->Stop();
                }
                if (vehicleSound.otherSound.id != SoundId::null)
                {
                    vehicleSound.otherSound.channel->Stop();
                }
            }
        }
    }

    static IAudioMixer* GetMixer()
    {
        auto& audioContext = GetContext()->GetAudioContext();
        return audioContext.GetMixer();
    }

    std::shared_ptr<IAudioChannel> CreateAudioChannel(
        SoundId id, bool loop, int32_t volume, float pan, double rate, bool forget)
    {
        // Get sound from base object
        auto [baseAudioObject, sampleIndex] = GetAudioObjectAndSampleIndex(id);
        if (baseAudioObject != nullptr)
        {
            auto source = baseAudioObject->GetSample(sampleIndex);
            if (source != nullptr)
            {
                return CreateAudioChannel(source, MixerGroup::sound, loop, volume, pan, rate, forget);
            }
        }
        return nullptr;
    }

    std::shared_ptr<IAudioChannel> CreateAudioChannel(
        IAudioSource* source, MixerGroup group, bool loop, int32_t volume, float pan, double rate, bool forget)
    {
        auto* mixer = GetMixer();
        if (mixer == nullptr)
        {
            return nullptr;
        }

        mixer->Lock();
        auto channel = mixer->Play(source, loop ? kMixerLoopInfinite : kMixerLoopNone, forget);
        if (channel != nullptr)
        {
            channel->SetGroup(group);
            channel->SetVolume(volume);
            channel->SetPan(pan);
            channel->SetRate(rate);
            channel->UpdateOldVolume();
        }
        mixer->Unlock();
        return channel;
    }

    int32_t DStoMixerVolume(int32_t volume)
    {
        return static_cast<int32_t>(kMixerVolumeMax * (std::pow(10.0f, static_cast<float>(volume) / 2000)));
    }

    float DStoMixerPan(int32_t pan)
    {
        constexpr int32_t kDSBPanLeft = -10000;
        constexpr int32_t kDSBPanRight = 10000;
        return ((static_cast<float>(pan) + -kDSBPanLeft) / kDSBPanRight) / 2;
    }

    double DStoMixerRate(int32_t frequency)
    {
        return static_cast<double>(frequency) / 22050;
    }

} // namespace OpenRCT2::Audio
