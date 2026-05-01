#pragma once

#include "sdr_source.h"
#include <iio.h>
#include <thread>
#include <atomic>

namespace dedective {

class PlutoSource : public dedective::SdrSource {
public:
    PlutoSource();
    ~PlutoSource();

    bool open(unsigned device_index = 0);
    void close();

    bool start(dedective::SdrSource::IQCallback cb);
    bool stop();

    bool set_freq(uint64_t hz);
    bool set_sample_rate(uint32_t rate);
    bool set_gain(int gain);
    bool set_lna_gain(uint32_t gain_db);
    bool set_vga_gain(uint32_t gain_db);
    bool set_amp_enable(bool enable);

private:
    IQCallback callback_;

    std::thread worker_;
    std::atomic<bool> streaming_{false};
    const char* last_error_;

    // libiio objects
    iio_context* ctx_ = nullptr;
    iio_device* rx_dev_ = nullptr;
    iio_channel* rx_i_ = nullptr;
    iio_channel* rx_q_ = nullptr;
    iio_buffer* rxbuf_ = nullptr;

    void rx_loop();

    bool is_streaming() const { return streaming_; }
    const char* last_error() const { return last_error_; }

    // Conversion scratch buffer — reused across callbacks to avoid alloc
    bool buff_1_flag_ = true;
    std::vector<std::complex<float>> conv_buf_1_;
    std::vector<std::complex<float>> conv_buf_2_;
};

} // namespace dedective
