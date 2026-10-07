#pragma once
#include <cstdint>
#include <cstddef>
#include <functional>
#include <complex>

namespace dedective {

// Callback type: called with a buffer of complex<float> IQ samples
using IQCallback = std::function<void(const std::complex<float>*, size_t)>;

// Hardware-agnostic SDR source interface.
//
// Backends (HackRF, SDRplay) implement this so the rest of the application is
// independent of the radio hardware.  See source_factory.h for construction.
class IqSource {
public:
    virtual ~IqSource() = default;

    IqSource(const IqSource&)            = delete;
    IqSource& operator=(const IqSource&) = delete;

    // Device lifecycle
    virtual bool open(unsigned device_index = 0) = 0;
    virtual void close() = 0;

    // Configuration (applied immediately; while streaming where the hardware
    // supports live retuning)
    virtual bool set_freq(uint64_t freq_hz) = 0;
    virtual bool set_sample_rate(uint32_t sample_rate) = 0;
    virtual bool set_lna_gain(uint32_t gain_db) = 0;
    virtual bool set_vga_gain(uint32_t gain_db) = 0;
    virtual bool set_amp_enable(bool enable) = 0;

    // Streaming
    virtual bool start(IQCallback cb) = 0;
    virtual bool stop() = 0;
    virtual bool is_streaming() const = 0;

    virtual const char* last_error() const = 0;
    virtual const char* name() const = 0;

    // Largest sample rate the device supports (Hz).
    virtual uint32_t max_sample_rate() const = 0;

    // True if the device can capture the whole DECT band in a single pass
    // (>= WIDEBAND_SAMPLE_RATE).  When false the caller must fall back to
    // channel hopping.
    virtual bool supports_full_wideband() const = 0;

protected:
    IqSource() = default;
};

enum class SourceType { Auto, HackRF, SDRplay, Pluto };

const char* source_type_name(SourceType type);
bool parse_source_type(const char* text, SourceType& out);

} // namespace dedective
