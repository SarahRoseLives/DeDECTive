#pragma once
#include "iq_source.h"

#ifdef DEDECTIVE_HAVE_SDRPLAY

#include "sdrplay_api.h"

#include <atomic>
#include <string>
#include <vector>

namespace dedective {

// Wraps the SDRplay API (v3.x) for DECT reception.
//
// SDRplay RSP receivers top out around 10 MHz of instantaneous bandwidth, so
// `supports_full_wideband()` returns false and callers fall back to channel
// hopping.  The generic LNA/VGA gain model is mapped onto the tuner's LNA
// state and IF gain-reduction controls.
class SdrplaySource : public IqSource {
public:
    SdrplaySource();
    ~SdrplaySource() override;

    bool open(unsigned device_index = 0) override;
    void close() override;

    bool set_freq(uint64_t freq_hz) override;
    bool set_sample_rate(uint32_t sample_rate) override;
    bool set_lna_gain(uint32_t gain_db) override;
    bool set_vga_gain(uint32_t gain_db) override;
    bool set_amp_enable(bool enable) override;

    bool start(IQCallback cb) override;
    bool stop() override;
    bool is_streaming() const override { return streaming_.load(std::memory_order_relaxed); }

    const char* last_error() const override { return last_error_.c_str(); }
    const char* name() const override { return "SDRplay"; }
    uint32_t max_sample_rate() const override { return 10'660'000; }
    bool supports_full_wideband() const override { return false; }

private:
    static void stream_callback(short* xi, short* xq,
                                sdrplay_api_StreamCbParamsT* params,
                                unsigned int numSamples, unsigned int reset,
                                void* cb_context);
    static void event_callback(sdrplay_api_EventT event_id,
                               sdrplay_api_TunerSelectT tuner,
                               sdrplay_api_EventParamsT* params,
                               void* cb_context);

    sdrplay_api_RxChannelParamsT* rx_channel() const;
    void apply_gain_params();
    void apply_bandwidth();
    unsigned char lna_state_max() const;

    sdrplay_api_DeviceT  device_{};
    sdrplay_api_DeviceParamsT* params_ = nullptr;
    unsigned char        hw_ver_       = 0;

    bool api_open_    = false;
    bool selected_    = false;
    bool initialized_ = false;

    uint32_t sample_rate_ = 4'608'000;
    uint64_t freq_hz_     = 1'921'536'000ULL;
    uint32_t lna_gain_    = 32;
    uint32_t vga_gain_    = 20;
    bool     amp_         = false;

    IQCallback        callback_;
    std::atomic<bool> streaming_{false};
    std::string       last_error_;

    std::vector<std::complex<float>> conv_buf_;
};

} // namespace dedective

#endif // DEDECTIVE_HAVE_SDRPLAY
