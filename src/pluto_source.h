#pragma once
#include "iq_source.h"

#ifdef DEDECTIVE_HAVE_PLUTO

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// Forward declarations so libiio stays out of the header.
struct iio_context;
struct iio_device;
struct iio_channel;
struct iio_buffer;

namespace dedective {

// Wraps the ADALM-PLUTO (AD936x) via libiio.
//
// The Pluto can stream over USB or over its RNDIS network link; the connection
// URI can be forced with the PLUTO_URI environment variable, otherwise a list
// of common URIs is tried in turn.  Instantaneous bandwidth is limited by the
// USB 2.0 link, so callers fall back to channel hopping.
class PlutoSource : public IqSource {
public:
    PlutoSource();
    ~PlutoSource() override;

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
    const char* name() const override { return "PlutoSDR"; }
    uint32_t max_sample_rate() const override { return 30'720'000; }
    bool supports_full_wideband() const override { return false; }

private:
    bool connect(const char* uri);
    void apply_sample_rate();
    void apply_frequency();
    void apply_gain();
    void apply_tracking();
    void stream_loop();

    iio_context* ctx_   = nullptr;
    iio_device*  phy_   = nullptr;   // ad9361-phy
    iio_device*  rx_dev_ = nullptr;  // cf-ad9361-lpc (streaming device)
    iio_channel* rx_phy_ = nullptr;  // phy "voltage0" (gain/bw/sample-rate)
    iio_channel* rx_lo_  = nullptr;  // phy "altvoltage0" (RX LO)
    iio_channel* rx_i_   = nullptr;  // voltage0 scan element
    iio_channel* rx_q_   = nullptr;  // voltage1 scan element
    iio_buffer*  buf_    = nullptr;

    uint32_t sample_rate_ = 4'608'000;
    uint64_t freq_hz_     = 1'921'536'000ULL;
    uint32_t lna_gain_    = 32;
    uint32_t vga_gain_    = 20;
    bool     amp_         = false;

    IQCallback        callback_;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread       thread_;
    std::string       last_error_;
    std::vector<std::complex<float>> conv_buf_;
};

} // namespace dedective

#endif // DEDECTIVE_HAVE_PLUTO
