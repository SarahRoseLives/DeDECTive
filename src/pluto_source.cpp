#include "pluto_source.h"

#ifdef DEDECTIVE_HAVE_PLUTO

#include <iio.h>

#include <algorithm>
#include <cerrno>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace dedective {

namespace {

constexpr unsigned kBufferSamples = 32768;
constexpr const char* kPhyDevice = "ad9361-phy";
constexpr const char* kRxDevice  = "cf-ad9361-lpc";

// DECT channels are spaced 1.728 MHz apart, so a ~1.8 MHz RF filter rejects
// the adjacent channels far better than a filter as wide as the sample rate.
constexpr long long kNarrowbandRfBw = 1'800'000LL;

bool ch_write_longlong(iio_channel* ch, const char* attr, long long value) {
    return ch && iio_channel_attr_write_longlong(ch, attr, value) >= 0;
}

bool ch_write_str(iio_channel* ch, const char* attr, const char* value) {
    return ch && iio_channel_attr_write(ch, attr, value) >= 0;
}

bool ch_write_bool(iio_channel* ch, const char* attr, bool value) {
    return ch && iio_channel_attr_write_bool(ch, attr, value) >= 0;
}

long long ch_read_longlong(iio_channel* ch, const char* attr, long long fallback) {
    long long v = fallback;
    if (ch && iio_channel_attr_read_longlong(ch, attr, &v) < 0) v = fallback;
    return v;
}

std::string iio_error_string(int err) {
    char buf[256] = {};
    iio_strerror(err, buf, sizeof(buf));
    return std::string(buf);
}

} // namespace

PlutoSource::PlutoSource() = default;

PlutoSource::~PlutoSource() {
    stop();
    close();
}

bool PlutoSource::connect(const char* uri) {
    ctx_ = iio_create_context_from_uri(uri);
    if (!ctx_) return false;

    phy_    = iio_context_find_device(ctx_, kPhyDevice);
    rx_dev_ = iio_context_find_device(ctx_, kRxDevice);
    if (!phy_ || !rx_dev_) {
        last_error_ = "Pluto IIO devices not present";
        iio_context_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }

    rx_phy_ = iio_device_find_channel(phy_, "voltage0", false);
    rx_lo_  = iio_device_find_channel(phy_, "altvoltage0", true);
    rx_i_   = iio_device_find_channel(rx_dev_, "voltage0", false);
    rx_q_   = iio_device_find_channel(rx_dev_, "voltage1", false);
    if (!rx_phy_ || !rx_lo_ || !rx_i_ || !rx_q_) {
        last_error_ = "Pluto RX channels not found";
        iio_context_destroy(ctx_);
        ctx_ = nullptr;
        return false;
    }
    return true;
}

bool PlutoSource::open(unsigned /*device_index*/) {
    if (ctx_) return true;
    last_error_.clear();

    auto adopt = [this]() {
        apply_sample_rate();
        apply_frequency();
        apply_gain();
        apply_tracking();
        last_error_.clear();
    };

    // Build a candidate URI list.  USB is preferred over the network/RNDIS
    // link: it is far less prone to the dropped samples that wreck DECT timing.
    std::vector<std::string> usb_uris, net_uris;
    if (iio_scan_context* scan = iio_create_scan_context(nullptr, 0)) {
        iio_context_info** info = nullptr;
        const ssize_t count = iio_scan_context_get_info_list(scan, &info);
        if (count > 0) {
            for (ssize_t i = 0; i < count; ++i) {
                const char* uri = iio_context_info_get_uri(info[i]);
                if (!uri) continue;
                if (std::strncmp(uri, "usb:", 4) == 0)
                    usb_uris.emplace_back(uri);
                else
                    net_uris.emplace_back(uri);
            }
            iio_context_info_list_free(info);
        }
        iio_scan_context_destroy(scan);
    }

    std::vector<std::string> uris;
    if (const char* env = std::getenv("PLUTO_URI"); env && *env)
        uris.emplace_back(env);
    uris.insert(uris.end(), usb_uris.begin(), usb_uris.end());
    uris.insert(uris.end(), net_uris.begin(), net_uris.end());
    uris.emplace_back("ip:pluto.local");
    uris.emplace_back("ip:192.168.2.1");
    uris.emplace_back("ip:");

    for (const auto& uri : uris) {
        errno = 0;
        if (connect(uri.c_str())) {
            adopt();
            if (std::getenv("PLUTO_DEBUG"))
                std::fprintf(stderr, "[pluto] connected via %s\n", uri.c_str());
            return true;
        }
        if (ctx_) {
            iio_context_destroy(ctx_);
            ctx_ = nullptr;
        }
    }

    last_error_ =
        "Unable to open PlutoSDR. Install the Pluto USB/RNDIS driver, or set "
        "PLUTO_URI (e.g. PLUTO_URI=ip:192.168.2.1 or PLUTO_URI=usb:<bus>.<port>).";
    return false;
}

void PlutoSource::close() {
    stop();
    if (ctx_) {
        iio_context_destroy(ctx_);
        ctx_ = nullptr;
    }
    phy_ = nullptr;
    rx_dev_ = nullptr;
    rx_phy_ = nullptr;
    rx_lo_ = nullptr;
    rx_i_ = nullptr;
    rx_q_ = nullptr;
}

void PlutoSource::apply_sample_rate() {
    if (!rx_phy_) return;
    ch_write_longlong(rx_phy_, "sampling_frequency", static_cast<long long>(sample_rate_));
    long long bw = (sample_rate_ <= 6'000'000)
                 ? kNarrowbandRfBw
                 : static_cast<long long>(sample_rate_);
    if (const char* env = std::getenv("PLUTO_RF_BW"); env && *env)
        bw = std::atoll(env);
    ch_write_longlong(rx_phy_, "rf_bandwidth", bw);
}

void PlutoSource::apply_tracking() {
    if (!rx_phy_) return;
    if (const char* env = std::getenv("PLUTO_TRACKING"); env && *env && std::atoi(env) == 0)
        return;
    // Enable the AD9361's internal DC-offset and quadrature (IQ) tracking so the
    // receiver removes its own impairments; this improves the DECT A/B-field
    // CRC pass rate.
    ch_write_bool(rx_phy_, "quadrature_tracking_en", true);
    ch_write_bool(rx_phy_, "bb_dc_offset_tracking_en", true);
    ch_write_bool(rx_phy_, "rf_dc_offset_tracking_en", true);
}

void PlutoSource::apply_frequency() {
    ch_write_longlong(rx_lo_, "frequency", static_cast<long long>(freq_hz_));
}

void PlutoSource::apply_gain() {
    if (!rx_phy_) return;
    ch_write_str(rx_phy_, "gain_control_mode", "manual");
    const long long gain = std::clamp<long long>(
        static_cast<long long>(lna_gain_) + static_cast<long long>(vga_gain_),
        0, 71);
    ch_write_longlong(rx_phy_, "hardwaregain", gain);

    if (std::getenv("PLUTO_DEBUG")) {
        std::fprintf(stderr,
            "[pluto] fs=%lld Hz rf_bw=%lld Hz gain=%lld dB\n",
            ch_read_longlong(rx_phy_, "sampling_frequency", -1),
            ch_read_longlong(rx_phy_, "rf_bandwidth", -1),
            ch_read_longlong(rx_phy_, "hardwaregain", -1));
    }
}

bool PlutoSource::set_freq(uint64_t freq_hz) {
    if (!ctx_) { last_error_ = "Device not open"; return false; }
    freq_hz_ = freq_hz;
    apply_frequency();
    return true;
}

bool PlutoSource::set_sample_rate(uint32_t sample_rate) {
    if (!ctx_) { last_error_ = "Device not open"; return false; }
    sample_rate_ = std::min(sample_rate, max_sample_rate());
    apply_sample_rate();
    apply_tracking();
    return true;
}

bool PlutoSource::set_lna_gain(uint32_t gain_db) {
    if (!ctx_) { last_error_ = "Device not open"; return false; }
    lna_gain_ = std::min(gain_db, 76u);
    apply_gain();
    return true;
}

bool PlutoSource::set_vga_gain(uint32_t gain_db) {
    if (!ctx_) { last_error_ = "Device not open"; return false; }
    vga_gain_ = std::min(gain_db, 76u);
    apply_gain();
    return true;
}

bool PlutoSource::set_amp_enable(bool enable) {
    amp_ = enable;
    (void)amp_;
    return true;
}

bool PlutoSource::start(IQCallback cb) {
    if (!ctx_) { last_error_ = "Device not open"; return false; }
    if (streaming_.load(std::memory_order_relaxed)) return true;

    callback_ = std::move(cb);

    iio_channel_enable(rx_i_);
    iio_channel_enable(rx_q_);

    buf_ = iio_device_create_buffer(rx_dev_, kBufferSamples, false);
    if (!buf_) {
        iio_channel_disable(rx_i_);
        iio_channel_disable(rx_q_);
        last_error_ = "Failed to create Pluto IIO buffer";
        return false;
    }

    stop_requested_.store(false, std::memory_order_relaxed);
    streaming_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&PlutoSource::stream_loop, this);
    return true;
}

void PlutoSource::stream_loop() {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
        const ssize_t nbytes = iio_buffer_refill(buf_);
        if (nbytes < 0) {
            if (stop_requested_.load(std::memory_order_relaxed)) break;
            last_error_ = "Pluto buffer refill failed: " +
                          iio_error_string(static_cast<int>(nbytes));
            break;
        }

        void* first = iio_buffer_first(buf_, rx_i_);
        const ptrdiff_t step = iio_buffer_step(buf_);
        void* end = iio_buffer_end(buf_);
        if (!first || step <= 0 || end <= first) continue;

        const size_t count =
            static_cast<size_t>(static_cast<char*>(end) - static_cast<char*>(first)) /
            static_cast<size_t>(step);

        if (conv_buf_.size() < count) conv_buf_.resize(count);

        char* p = static_cast<char*>(first);
        constexpr float scale = 1.0f / 32768.0f;
        for (size_t k = 0; k < count; ++k, p += step) {
            const auto* iq = reinterpret_cast<const int16_t*>(p);
            conv_buf_[k] = std::complex<float>(
                static_cast<float>(iq[0]) * scale,
                static_cast<float>(iq[1]) * scale);
        }

        if (callback_ && count > 0)
            callback_(conv_buf_.data(), count);
    }
    streaming_.store(false, std::memory_order_relaxed);
}

bool PlutoSource::stop() {
    if (!streaming_.load(std::memory_order_relaxed) &&
        !thread_.joinable() && !buf_) {
        return true;
    }

    stop_requested_.store(true, std::memory_order_relaxed);
    if (buf_) iio_buffer_cancel(buf_);
    if (thread_.joinable()) thread_.join();

    if (buf_) {
        iio_buffer_destroy(buf_);
        buf_ = nullptr;
    }
    if (rx_i_) iio_channel_disable(rx_i_);
    if (rx_q_) iio_channel_disable(rx_q_);

    streaming_.store(false, std::memory_order_relaxed);
    return true;
}

} // namespace dedective

#endif // DEDECTIVE_HAVE_PLUTO
