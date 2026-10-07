#pragma once
#include "dect_channels.h"
#include "audio_output.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dedective {

// Full-band sample rate used for HackRF-style wideband capture.
inline constexpr uint32_t WIDEBAND_SAMPLE_RATE = 18'432'000;
inline constexpr size_t DECT_SLOT_COUNT = 24;

struct WidebandChannelView {
    int      channel_number      = -1;
    uint64_t freq_hz             = 0;
    float    smoothed_power_db   = 0.0f;
    float    relative_power_db   = 0.0f;
    bool     active              = false;
    bool     voice_detected      = false;
    bool     qt_synced           = false;
    int      active_parts        = 0;
    uint64_t packets_seen        = 0;
    uint64_t voice_frames_ok     = 0;
    uint64_t voice_xcrc_fail     = 0;
    uint64_t voice_skipped       = 0;
};

struct WidebandSnapshot {
    bool ready = false;
    size_t buffered_samples = 0;
    float noise_floor_db = 0.0f;
    std::vector<float> fft_db;
    std::vector<float> waterfall_db;
    size_t waterfall_rows = 0;
    size_t waterfall_cols = 0;
    std::array<std::array<uint8_t, DECT_SLOT_COUNT>, NUM_DECT_CHANNELS> slot_state{};
    std::array<WidebandChannelView, NUM_DECT_CHANNELS> channels{};
};

// Common interface implemented by WidebandMonitor (full-band FFT capture) and
// HoppingScanner (channel-by-channel scan for bandwidth-limited devices).
class Scanner {
public:
    virtual ~Scanner() = default;

    virtual void set_band(DectBand band) = 0;
    virtual uint64_t center_freq() const = 0;

    virtual bool update_visuals() = 0;
    virtual WidebandSnapshot snapshot() const = 0;
    virtual void render_frame() = 0;

    virtual void set_dc_block(bool enabled) = 0;
    virtual bool dc_block_enabled() const = 0;

    // Feed wideband IQ (full-band scanners only).  Hopping scanners pull their
    // own samples and ignore this.
    virtual void ingest(const std::complex<float>* /*samples*/, size_t /*n*/) {}

    // Audio routing used by the CLI wideband monitor.
    virtual void set_audio_output(AudioOutput*) {}
    virtual void set_audio_channel(int) {}
    virtual int  audio_channel() const { return -1; }
};

} // namespace dedective
