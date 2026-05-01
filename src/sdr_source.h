#pragma once
#include <functional>
#include <cstdint>
#include <complex>

namespace dedective {

class SdrSource {

public:
    // Callback type: called with a buffer of complex<float> IQ samples
    using IQCallback = std::function<void(const std::complex<float>*, size_t)>;

    virtual ~SdrSource() = default;

    virtual bool open(unsigned device_index = 0) = 0;
    virtual void close() = 0;

    virtual bool set_freq(uint64_t hz) = 0;
    virtual bool set_sample_rate(uint32_t rate) = 0;
    virtual bool set_lna_gain(uint32_t gain_db) = 0;
    virtual bool set_vga_gain(uint32_t gain_db) = 0;
    virtual bool set_amp_enable(bool enable) = 0;

    virtual bool start(IQCallback cb) = 0;
    virtual bool stop() = 0;

    virtual bool is_streaming() const = 0;
    virtual const char* last_error() const = 0;
};

} // namespace dedective
