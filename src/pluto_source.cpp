#include "pluto_source.h"
#include <iostream>
#include <cstring>

namespace dedective {

#define DEFAULT_URI "ip:192.168.2.1"
#define BUFFER_SAMPLES 16384

// Convert raw signed 16-bit IQ pairs to complex<float> normalized to [-1,+1]
static void convert_int16_to_cf32(const int16_t* in, std::complex<float>* out, size_t n_complex_samples) {
    constexpr float scale = 1.0f / 32768.0f;
    for (size_t i = 0; i < n_complex_samples; ++i) {
        out[i] = std::complex<float>(static_cast<float>(in[2*i]) * scale, static_cast<float>(in[2*i+1]) * scale);
    }
}

PlutoSource::PlutoSource() : ctx_(nullptr), streaming_(false), last_error_("") {}

PlutoSource::~PlutoSource() {
    stop();
    close();
}

bool PlutoSource::open(unsigned device_index) {
    ctx_ = iio_create_context_from_uri(DEFAULT_URI);
    if (!ctx_) {
        std::cerr << "Failed to create IIO context\n";
        return false;
    }

    rx_dev_ = iio_context_find_device(ctx_, "cf-ad9361-lpc");
    if (!rx_dev_) {
        std::cerr << "RX device not found\n";
        return false;
    }

    rx_i_ = iio_device_find_channel(rx_dev_, "voltage0", false);
    rx_q_ = iio_device_find_channel(rx_dev_, "voltage1", false);

    if (!rx_i_ || !rx_q_) {
        std::cerr << "RX channels not found\n";
        return false;
    }

    iio_channel_enable(rx_i_);
    iio_channel_enable(rx_q_);

    rxbuf_ = iio_device_create_buffer(rx_dev_, BUFFER_SAMPLES, false);
    if (!rxbuf_) {
        std::cerr << "Failed to create RX buffer\n";
        return false;
    }

    return true;
}

void PlutoSource::close() {
    if (rxbuf_) {
        iio_buffer_destroy(rxbuf_);
        rxbuf_ = nullptr;
    }

    if (ctx_) {
        iio_context_destroy(ctx_);
        ctx_ = nullptr;
    }
}

bool PlutoSource::start(dedective::SdrSource::IQCallback cb) {
    if (!rxbuf_) return false;

    callback_ = cb;
    streaming_ = true;

    worker_ = std::thread(&PlutoSource::rx_loop, this);
    return true;
}

bool PlutoSource::stop() {
    streaming_ = false;

    if (worker_.joinable())
        worker_.join();

    return true;
}

void PlutoSource::rx_loop() {
    while (streaming_) {
        ssize_t nbytes = iio_buffer_refill(rxbuf_);

        if (nbytes < 0) {
            std::cerr << "Buffer refill error\n";
            continue;
        }

        if (callback_) {
            // we know we will get a power of two samples back
            size_t n_complex_samples = nbytes / (2 * sizeof(int16_t));

            // interleaved IQ int16_t
            int16_t* data = static_cast<int16_t*>(iio_buffer_start(rxbuf_));
            
            // should really have some sort of locking
            // we could overwrite current data
            if (buff_1_flag_) {
                if (conv_buf_1_.size() < n_complex_samples)
                    conv_buf_1_.resize(n_complex_samples);
            
                convert_int16_to_cf32(data, conv_buf_1_.data(), n_complex_samples);

                callback_(conv_buf_1_.data(), n_complex_samples);
                buff_1_flag_ = false;
            } else {
                if (conv_buf_2_.size() < n_complex_samples)
                    conv_buf_2_.resize(n_complex_samples);
            
                // problem as we are in separate thread and we could overwrite current data
                convert_int16_to_cf32(data, conv_buf_2_.data(), n_complex_samples);
                
                callback_(conv_buf_2_.data(), n_complex_samples);
                buff_1_flag_ = true;
            }
        }
    }
}

bool PlutoSource::set_freq(uint64_t hz) {
    if (!ctx_) return false;

    iio_device* phy = iio_context_find_device(ctx_, "ad9361-phy");
    if (!phy) return false;

    iio_channel* lo = iio_device_find_channel(phy, "altvoltage0", true);
    if (!lo) return false;

    //uint32_t hz_small = static_cast<uint32_t>(hz & 0xffffffff);
    return iio_channel_attr_write_longlong(lo, "frequency", hz) == 0;
}

bool PlutoSource::set_sample_rate(uint32_t rate) {
    if (!ctx_) return false;

    iio_device* phy = iio_context_find_device(ctx_, "ad9361-phy");
    if (!phy) return false;

    iio_channel* ch = iio_device_find_channel(phy, "voltage0", false);
    if (!ch) return false;

    return iio_channel_attr_write_longlong(ch, "sampling_frequency", rate) == 0;
}

bool PlutoSource::set_lna_gain(uint32_t gain_db) {
    if (!ctx_) return false;

    iio_device* phy = iio_context_find_device(ctx_, "ad9361-phy");
    if (!phy) return false;

    iio_channel* ch = iio_device_find_channel(phy, "voltage0", false);
    if (!ch) return false;

    // fast_attack gain mode
    iio_channel_attr_write(ch, "gain_control_mode", "fast_attack");

    return true;
    //return iio_channel_attr_write_longlong(ch, "hardwaregain", gain_db) == 0;
}

bool PlutoSource::set_vga_gain(uint32_t gain_db) { return true; }
bool PlutoSource::set_amp_enable(bool enable) { return true; }

} // namespace dedective
