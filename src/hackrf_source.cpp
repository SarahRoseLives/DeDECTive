#include "hackrf_source.h"

#ifdef DEDECTIVE_HAVE_HACKRF

#include <hackrf.h>
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace dedective {

// Trampoline: libhackrf calls this C-style; we recover the HackrfSource pointer
// from transfer->rx_ctx which we set in start().
struct HackrfTransfer {
    // Mirrors the layout of hackrf_transfer for the fields we need.
    // We just use the real hackrf_transfer* via void* cast.
};

// ── static helpers ──────────────────────────────────────────────────────────

// Convert HackRF raw signed 8-bit IQ pairs to complex<float> normalized to [-1,+1]
static void convert_s8_to_cf32(const int8_t* in, std::complex<float>* out, size_t n_samples) {
    constexpr float scale = 1.0f / 128.0f;
    for (size_t i = 0; i < n_samples; ++i) {
        out[i] = std::complex<float>(in[2*i] * scale, in[2*i+1] * scale);
    }
}

// ── HackrfSource ────────────────────────────────────────────────────────────

HackrfSource::HackrfSource()
    : device_(nullptr), streaming_(false), last_error_("") {}

HackrfSource::~HackrfSource() {
    stop();
    close();
}

bool HackrfSource::open(unsigned device_index) {
    int r = hackrf_init();
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }

    hackrf_device_list_t* list = hackrf_device_list();
    if (!list || list->devicecount == 0) {
        last_error_ = "No HackRF devices found";
        hackrf_exit();
        return false;
    }

    if (device_index >= (unsigned)list->devicecount) {
        last_error_ = "Device index out of range";
        hackrf_device_list_free(list);
        hackrf_exit();
        return false;
    }

    hackrf_device* dev = nullptr;
    r = hackrf_device_list_open(list, device_index, &dev);
    hackrf_device_list_free(list);

    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        hackrf_exit();
        return false;
    }

    device_ = dev;
    last_error_ = "";
    return true;
}

void HackrfSource::close() {
    if (device_) {
        hackrf_close(static_cast<hackrf_device*>(device_));
        device_ = nullptr;
        hackrf_exit();
    }
}

bool HackrfSource::set_freq(uint64_t freq_hz) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_freq(static_cast<hackrf_device*>(device_), freq_hz);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_sample_rate(uint32_t sample_rate) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_sample_rate(static_cast<hackrf_device*>(device_),
                                   static_cast<double>(sample_rate));
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    const uint32_t bandwidth = hackrf_compute_baseband_filter_bw(sample_rate);
    r = hackrf_set_baseband_filter_bandwidth(static_cast<hackrf_device*>(device_), bandwidth);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_lna_gain(uint32_t gain_db) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    // LNA gain: 0, 8, 16, 24, 32, 40 dB
    uint32_t clamped = std::min(gain_db, 40u);
    clamped = (clamped / 8) * 8;
    int r = hackrf_set_lna_gain(static_cast<hackrf_device*>(device_), clamped);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_vga_gain(uint32_t gain_db) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    // VGA gain: 0–62 dB in steps of 2
    uint32_t clamped = std::min(gain_db, 62u);
    clamped = (clamped / 2) * 2;
    int r = hackrf_set_vga_gain(static_cast<hackrf_device*>(device_), clamped);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_amp_enable(bool enable) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_amp_enable(static_cast<hackrf_device*>(device_), enable ? 1 : 0);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

int HackrfSource::rx_callback(hackrf_transfer* transfer) {
    HackrfSource* self = static_cast<HackrfSource*>(transfer->rx_ctx);

    if (!self->streaming_) return 0;

    size_t n_samples = transfer->valid_length / 2; // 2 bytes (I+Q) per sample

    if (self->conv_buf_.size() < n_samples)
        self->conv_buf_.resize(n_samples);

    convert_s8_to_cf32(reinterpret_cast<const int8_t*>(transfer->buffer),
                       self->conv_buf_.data(), n_samples);

    self->callback_(self->conv_buf_.data(), n_samples);
    return 0; // returning non-zero stops streaming
}

bool HackrfSource::start(IQCallback cb) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    if (streaming_) return true;

    callback_  = std::move(cb);
    streaming_ = true;

    int r = hackrf_start_rx(static_cast<hackrf_device*>(device_), rx_callback, this);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        streaming_  = false;
        return false;
    }
    return true;
}

bool HackrfSource::stop() {
    if (!streaming_) return true;
    streaming_ = false;
    if (device_) {
        hackrf_stop_rx(static_cast<hackrf_device*>(device_));
    }
    return true;
}

bool HackrfSource::is_streaming() const {
    return streaming_;
}

} // namespace dedective

#endif // DEDECTIVE_HAVE_HACKRF
