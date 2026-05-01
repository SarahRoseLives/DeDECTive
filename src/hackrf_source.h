#pragma once
#include "sdr_source.h"
#include <hackrf.h>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <array>
#include <atomic>
#include <thread>
#include <vector>
#include <complex>

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

class HackrfSource : public dedective::SdrSource {
public:
    HackrfSource();
    ~HackrfSource();

    // Not copyable
    HackrfSource(const HackrfSource&)            = delete;
    HackrfSource& operator=(const HackrfSource&) = delete;

    // Device lifecycle
    bool open(unsigned device_index = 0);
    void close();

    // Configuration (must be called before start())
    bool set_freq(uint64_t freq_hz);
    bool set_sample_rate(uint32_t sample_rate);
    bool set_lna_gain(uint32_t gain_db);   // 0–40 dB, steps of 8
    bool set_vga_gain(uint32_t gain_db);   // 0–62 dB, steps of 2
    bool set_amp_enable(bool enable);

    // Streaming
    bool start(dedective::SdrSource::IQCallback cb);
    bool stop();

    bool is_streaming() const;
    const char* last_error() const { return last_error_; }

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
