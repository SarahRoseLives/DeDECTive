#pragma once
#include "iq_source.h"

#ifdef DEDECTIVE_HAVE_HACKRF

#include <hackrf.h>
#include <atomic>
#include <vector>

namespace dedective {

// Wraps libhackrf for simple blocking or async DECT reception.
//
// Usage:
//   HackrfSource src;
//   src.open();
//   src.set_freq(1921536000);
//   src.set_sample_rate(4608000);
//   src.set_lna_gain(32);       // 0..40 dB, 8 dB steps
//   src.set_vga_gain(20);       // 0..62 dB, 2 dB steps
//   src.set_amp_enable(false);  // external 14 dB amp
//   src.start(callback);
//   ...
//   src.stop();
//   src.close();

class HackrfSource : public IqSource {
public:
    HackrfSource();
    ~HackrfSource() override;

    // Device lifecycle
    bool open(unsigned device_index = 0) override;
    void close() override;

    // Configuration (must be called before start())
    bool set_freq(uint64_t freq_hz) override;
    bool set_sample_rate(uint32_t sample_rate) override;
    bool set_lna_gain(uint32_t gain_db) override;   // 0–40 dB, steps of 8
    bool set_vga_gain(uint32_t gain_db) override;   // 0–62 dB, steps of 2
    bool set_amp_enable(bool enable) override;

    // Streaming
    bool start(IQCallback cb) override;
    bool stop() override;

    bool is_streaming() const override;
    const char* last_error() const override { return last_error_; }

    const char* name() const override { return "HackRF"; }
    uint32_t max_sample_rate() const override { return 20'000'000; }
    bool supports_full_wideband() const override { return true; }

private:
    // libhackrf uses a C callback; we route it through a static trampoline
    static int rx_callback(hackrf_transfer* transfer);

    void* device_;                // hackrf_device*
    IQCallback callback_;
    std::atomic<bool> streaming_;
    const char* last_error_;

    // Conversion scratch buffer — reused across callbacks to avoid alloc
    std::vector<std::complex<float>> conv_buf_;
};

} // namespace dedective

#endif // DEDECTIVE_HAVE_HACKRF
