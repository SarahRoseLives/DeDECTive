#pragma once
#include "scanner.h"
#include "iq_source.h"
#include "phase_diff.h"
#include "dc_blocker.h"
#include "packet_receiver.h"
#include "packet_decoder.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dedective {

// Channel-hopping scanner for devices that cannot capture the whole DECT band
// in one pass (e.g. SDRplay).  Tunes each channel in turn at SAMPLE_RATE,
// runs the narrowband decode pipeline, and aggregates the results into the
// same WidebandSnapshot used by the full-band WidebandMonitor.
class HoppingScanner : public Scanner {
public:
    explicit HoppingScanner(IqSource& source);
    ~HoppingScanner() override;

    void set_band(DectBand band) override;
    uint64_t center_freq() const override { return center_freq_hz_; }
    bool update_visuals() override;
    WidebandSnapshot snapshot() const override;
    void render_frame() override;
    void set_dc_block(bool enabled) override { dc_block_enabled_ = enabled; }
    bool dc_block_enabled() const override { return dc_block_enabled_; }

    void set_audio_output(AudioOutput* audio) override;
    void set_audio_channel(int channel_index) override;
    int  audio_channel() const override;

    bool start();
    void stop();
    bool is_running() const { return running_; }

    int  dwell_ms() const { return dwell_ms_.load(std::memory_order_relaxed); }
    void set_dwell_ms(int ms);

private:
    static constexpr size_t PLOT_BINS = 256;
    static constexpr size_t WATERFALL_ROWS = 120;

    struct ChannelStat {
        bool     active        = false;
        bool     voice         = false;
        bool     qt            = false;
        int      active_parts  = 0;
        uint64_t packets       = 0;
        uint64_t voice_ok      = 0;
        uint64_t xcrc_fail     = 0;
        uint64_t skipped       = 0;
        float    power_db      = -120.0f;
        std::array<uint8_t, DECT_SLOT_COUNT> slots{};
    };

    void handle_samples(const std::complex<float>* samples, size_t n);
    void build_pipeline_locked();
    void finalize_current_locked();
    void push_waterfall_row_locked();

    IqSource& source_;
    DectBand band_ = DectBand::US;
    const std::array<DectChannel, NUM_DECT_CHANNELS>* channels_ = nullptr;
    uint64_t center_freq_hz_ = 0;

    mutable std::mutex mutex_;               // guards pipeline, stats and visuals

    int current_ch_ = 0;
    PhaseDiff phase_diff_;
    DCBlocker dc_blocker_;
    std::unique_ptr<PacketReceiver> receiver_;
    std::unique_ptr<PacketDecoder>  decoder_;
    ChannelStat current_;
    double   power_accum_ = 0.0;
    uint64_t power_count_ = 0;

    std::array<ChannelStat, NUM_DECT_CHANNELS> stats_{};

    std::chrono::steady_clock::time_point hop_start_{};
    std::atomic<int> dwell_ms_{300};
    bool running_ = false;
    bool dc_block_enabled_ = true;
    bool have_history_ = false;
    float noise_floor_db_ = -110.0f;

    std::array<float, PLOT_BINS> fft_db_{};
    std::array<float, PLOT_BINS> smoothed_db_{};
    std::vector<float> waterfall_;
    size_t waterfall_head_ = 0;
    size_t waterfall_filled_ = 0;

    AudioOutput* audio_out_ = nullptr;
    int audio_channel_ = -1;   // -1 = auto
};

} // namespace dedective
