#include "audio_output.h"

#define MINIAUDIO_IMPLEMENTATION
#define MA_NO_ENCODING
#define MA_NO_DECODING
#define MA_NO_GENERATION
#include "miniaudio.h"

#include <algorithm>
#include <cstring>

namespace dedective {

// Nested helper so the miniaudio C callback can reach private members of
// AudioOutput without exposing miniaudio types in the header.
struct AudioOutput::DeviceCallback {
    static void data(ma_device* device, void* output,
                     const void* /*input*/, ma_uint32 frame_count) {
        auto* self = static_cast<AudioOutput*>(device->pUserData);
        if (self)
            self->pull_samples(static_cast<int16_t*>(output), frame_count);
    }
};

AudioOutput::AudioOutput() = default;

AudioOutput::~AudioOutput() { stop(); }

bool AudioOutput::start() {
    if (running_.load()) return true;

    auto* device = new ma_device;
    ma_device_config config = ma_device_config_init(ma_device_type_playback);
    config.playback.format   = ma_format_s16;
    config.playback.channels = 1;
    config.sampleRate        = 8000;
    config.dataCallback      = &AudioOutput::DeviceCallback::data;
    config.pUserData         = this;

    if (ma_device_init(nullptr, &config, device) != MA_SUCCESS) {
        delete device;
        return false;
    }

    read_pos_  = 0;
    write_pos_ = 0;

    if (ma_device_start(device) != MA_SUCCESS) {
        ma_device_uninit(device);
        delete device;
        return false;
    }

    device_  = device;
    running_ = true;
    return true;
}

void AudioOutput::stop() {
    if (!running_.load() && !device_) return;
    running_ = false;
    if (device_) {
        auto* device = static_cast<ma_device*>(device_);
        ma_device_stop(device);
        ma_device_uninit(device);
        delete device;
        device_ = nullptr;
    }
}

void AudioOutput::write_samples(const int16_t* samples, size_t count) {
    if (!running_.load(std::memory_order_relaxed)) return;

    size_t wp = write_pos_.load(std::memory_order_relaxed);
    size_t rp = read_pos_.load(std::memory_order_acquire);

    // If ring would overflow, skip oldest data
    if (wp - rp + count > RING_SIZE)
        read_pos_.store(wp + count - RING_SIZE, std::memory_order_release);

    for (size_t i = 0; i < count; ++i)
        ring_[(wp + i) & RING_MASK] = samples[i];

    write_pos_.store(wp + count, std::memory_order_release);
}

void AudioOutput::set_muted(bool m)   { muted_ = m; }
void AudioOutput::set_volume(float v) { volume_ = std::clamp(v, 0.0f, 1.0f); }

void AudioOutput::pull_samples(int16_t* out, size_t frames) {
    if (!running_.load(std::memory_order_relaxed)) {
        std::memset(out, 0, frames * sizeof(int16_t));
        return;
    }

    const float vol = muted_.load(std::memory_order_relaxed)
                    ? 0.0f
                    : volume_.load(std::memory_order_relaxed);

    const size_t rp = read_pos_.load(std::memory_order_relaxed);
    const size_t wp = write_pos_.load(std::memory_order_acquire);
    const size_t avail = wp - rp;

    const size_t fill = std::min(avail, frames);
    for (size_t i = 0; i < fill; ++i) {
        const int32_t s = ring_[(rp + i) & RING_MASK];
        out[i] = static_cast<int16_t>(std::clamp(
            static_cast<int32_t>(s * vol), -32768, 32767));
    }
    // Underrun handling: fade the last sample down to silence instead of a
    // hard discontinuity (avoids clicks/pops).
    if (fill < frames) {
        const int16_t last = (fill > 0) ? out[fill - 1] : 0;
        const size_t missing = frames - fill;
        for (size_t i = 0; i < missing; ++i) {
            const float fade = 1.0f - (static_cast<float>(i) / static_cast<float>(missing));
            out[fill + i] = static_cast<int16_t>(last * fade);
        }
    }

    read_pos_.store(rp + fill, std::memory_order_release);
}

} // namespace dedective
