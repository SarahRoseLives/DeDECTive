#pragma once

#include "scanner.h"
#include "packet_decoder.h"
#include "packet_receiver.h"
#include "phase_diff.h"
#include "audio_output.h"
#include "dc_blocker.h"

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace dedective {

class WidebandMonitor : public Scanner {
public:
    static constexpr size_t FFT_SIZE = 16'384;
    static constexpr size_t FFT_PLOT_BINS = 256;
    static constexpr size_t WATERFALL_HISTORY = 160;

    WidebandMonitor();

    // Reconfigure for a different DECT band.  Must be called while
    // capture is stopped (before ingest()).
    void set_band(DectBand band) override;
    uint64_t center_freq() const override { return center_freq_hz_; }

    void ingest(const std::complex<float>* samples, size_t n) override;
    bool update_visuals() override;
    WidebandSnapshot snapshot() const override;
    void render_frame() override;

    // Audio output control
    void set_audio_output(AudioOutput* audio) override;
    void set_audio_channel(int channel_index) override;  // -1 = auto (first voice channel)
    int  audio_channel() const override;

    // DC offset correction
    void set_dc_block(bool enabled) override;
    bool dc_block_enabled() const override;

private:
    struct ChannelDecoderSnapshot {
        bool     voice_detected = false;
        bool     qt_synced      = false;
        int      active_parts   = 0;
        uint64_t packets_seen   = 0;
        uint64_t voice_frames_ok  = 0;
        uint64_t voice_xcrc_fail  = 0;
        uint64_t voice_skipped    = 0;
        std::array<uint8_t, DECT_SLOT_COUNT> slot_state{};
    };

    std::vector<std::complex<float>> ring_buffer_;
    std::array<float, NUM_DECT_CHANNELS> smoothed_power_db_;
    std::array<float, NUM_DECT_CHANNELS> peak_delta_db_;
    std::array<PhaseDiff, NUM_DECT_CHANNELS> phase_diff_;
    std::array<std::complex<float>, NUM_DECT_CHANNELS> mixer_;
    std::array<std::complex<float>, NUM_DECT_CHANNELS> mixer_step_;
    std::array<std::complex<float>, NUM_DECT_CHANNELS> decim_accum_;
    std::array<int, NUM_DECT_CHANNELS> decim_phase_;
    std::array<std::unique_ptr<PacketReceiver>, NUM_DECT_CHANNELS> receivers_;
    std::array<std::unique_ptr<PacketDecoder>, NUM_DECT_CHANNELS> decoders_;
    std::array<bool, NUM_DECT_CHANNELS> voice_detected_;
    std::array<int, NUM_DECT_CHANNELS> active_parts_;
    std::array<uint64_t, NUM_DECT_CHANNELS> packets_seen_;
    std::array<float, FFT_PLOT_BINS> fft_plot_db_;
    std::vector<float> waterfall_history_;
    std::array<WidebandChannelView, NUM_DECT_CHANNELS> channel_views_;
    std::array<std::array<uint8_t, DECT_SLOT_COUNT>, NUM_DECT_CHANNELS> slot_state_;
    size_t write_pos_;
    size_t buffered_samples_;
    uint64_t sample_counter_;
    bool have_history_;
    bool visual_ready_;
    size_t waterfall_head_;
    size_t waterfall_rows_filled_;
    float noise_floor_db_;
    mutable std::mutex ring_mutex_;
    mutable std::mutex status_mutex_;
    mutable std::mutex visual_mutex_;

    // Band configuration
    const std::array<DectChannel, NUM_DECT_CHANNELS>* channels_;
    uint64_t center_freq_hz_;

    // Audio routing
    AudioOutput* audio_out_ = nullptr;
    int audio_channel_ = -1;  // -1 = auto
    mutable std::mutex audio_mutex_;

    // DC offset correction
    DCBlocker dc_blocker_;
    bool dc_block_enabled_ = true;
    std::vector<std::complex<float>> dc_buf_;  // reusable scratch for filtered samples

    bool snapshot_latest_block(std::vector<std::complex<float>>& out, size_t& buffered_samples);
    void snapshot_channel_state(
        std::array<ChannelDecoderSnapshot, NUM_DECT_CHANNELS>& out) const;
    void configure_band();  // (re)init mixer + channel views from channels_/center_freq_hz_
    void render_waiting_screen(size_t buffered_samples) const;
    void process_sample(std::complex<float> sample) noexcept;
    void on_channel_update(size_t channel_index, const PartInfo parts[], int count) noexcept;
    void on_voice_packet(size_t channel_index, int rx_id,
                         const int16_t* pcm, size_t count) noexcept;

    static void fft_inplace(std::vector<std::complex<float>>& data);
};

} // namespace dedective
