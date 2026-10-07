#include "sdrplay_source.h"

#ifdef DEDECTIVE_HAVE_SDRPLAY

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace dedective {

namespace {

std::string err_string(sdrplay_api_ErrT err) {
    const char* s = sdrplay_api_GetErrorString(err);
    return s ? std::string(s) : std::string("unknown SDRplay error");
}

// Pick the smallest tuner bandwidth that covers the requested sample rate.
sdrplay_api_Bw_MHzT bandwidth_for_rate(uint32_t sample_rate_hz) {
    const int rate_khz = static_cast<int>((sample_rate_hz + 500) / 1000);
    if (rate_khz <= 1536) return sdrplay_api_BW_1_536;
    if (rate_khz <= 5000) return sdrplay_api_BW_5_000;
    if (rate_khz <= 6000) return sdrplay_api_BW_6_000;
    if (rate_khz <= 7000) return sdrplay_api_BW_7_000;
    return sdrplay_api_BW_8_000;
}

} // namespace

SdrplaySource::SdrplaySource() = default;

SdrplaySource::~SdrplaySource() {
    stop();
    close();
}

sdrplay_api_RxChannelParamsT* SdrplaySource::rx_channel() const {
    if (!params_ || !params_->rxChannelA) return nullptr;
    return params_->rxChannelA;
}

unsigned char SdrplaySource::lna_state_max() const {
    switch (hw_ver_) {
    case SDRPLAY_RSP1_ID:  return 3;
    case SDRPLAY_RSP2_ID:  return 8;
    default:               return 9;   // RSP1A, RSPduo, RSPdx, RSP1B, ...
    }
}

void SdrplaySource::apply_gain_params() {
    sdrplay_api_RxChannelParamsT* ch = rx_channel();
    if (!ch) return;

    sdrplay_api_GainT& gain = ch->tunerParams.gain;

    // Map the generic VGA slider (0..62, more = more gain) onto the IF
    // gain-reduction control (20..59 dB, more = less gain).
    const int vga = static_cast<int>(std::min(vga_gain_, 62u));
    int gr_db = 59 - static_cast<int>(std::lround(vga * 39.0 / 62.0));
    gr_db = std::clamp(gr_db, 20, 59);
    gain.gRdB = gr_db;

    // Map the generic LNA slider (0..40, more = more gain) onto the device's
    // LNA state (0 = maximum gain, increasing state adds attenuation).
    const unsigned char max_state = lna_state_max();
    const int lna = static_cast<int>(std::min(lna_gain_, 40u));
    int state = static_cast<int>(std::lround((40 - lna) / 40.0 * max_state));
    state = std::clamp(state, 0, static_cast<int>(max_state));
    gain.LNAstate = static_cast<unsigned char>(state);

    gain.minGr = sdrplay_api_NORMAL_MIN_GR;
    gain.syncUpdate = 0;
}

void SdrplaySource::apply_bandwidth() {
    sdrplay_api_RxChannelParamsT* ch = rx_channel();
    if (!ch) return;
    ch->tunerParams.bwType = bandwidth_for_rate(sample_rate_);
    ch->tunerParams.ifType = sdrplay_api_IF_Zero;
}

bool SdrplaySource::open(unsigned device_index) {
    if (api_open_) return true;

    sdrplay_api_ErrT err = sdrplay_api_Open();
    if (err != sdrplay_api_Success) {
        last_error_ = err_string(err);
        return false;
    }
    api_open_ = true;

    float api_ver = 0.0f;
    if (sdrplay_api_ApiVersion(&api_ver) == sdrplay_api_Success) {
        if (api_ver < SDRPLAY_API_VERSION - 0.01f) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                          "SDRplay API %.2f is older than required %.2f",
                          static_cast<double>(api_ver),
                          static_cast<double>(SDRPLAY_API_VERSION));
            last_error_ = buf;
            close();
            return false;
        }
    }

    sdrplay_api_DeviceT devices[SDRPLAY_MAX_DEVICES];
    unsigned int num_devices = 0;
    err = sdrplay_api_GetDevices(devices, &num_devices, SDRPLAY_MAX_DEVICES);
    if (err != sdrplay_api_Success) {
        last_error_ = err_string(err);
        close();
        return false;
    }
    if (num_devices == 0) {
        last_error_ = "No SDRplay devices found";
        close();
        return false;
    }
    if (device_index >= num_devices) {
        last_error_ = "SDRplay device index out of range";
        close();
        return false;
    }

    device_ = devices[device_index];
    hw_ver_ = device_.hwVer;

    // RSPduo must select a tuner mode before SelectDevice().
    if (device_.hwVer == SDRPLAY_RSPduo_ID) {
        device_.tuner = sdrplay_api_Tuner_A;
        device_.rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
    }

    err = sdrplay_api_SelectDevice(&device_);
    if (err != sdrplay_api_Success) {
        last_error_ = err_string(err);
        close();
        return false;
    }
    selected_ = true;
    hw_ver_ = device_.hwVer;

    err = sdrplay_api_GetDeviceParams(device_.dev, &params_);
    if (err != sdrplay_api_Success || !params_) {
        last_error_ = err_string(err);
        close();
        return false;
    }

    if (params_->devParams) {
        params_->devParams->fsFreq.fsHz = static_cast<double>(sample_rate_);
    }
    sdrplay_api_RxChannelParamsT* ch = rx_channel();
    if (ch) {
        ch->tunerParams.rfFreq.rfHz = static_cast<double>(freq_hz_);
        ch->ctrlParams.dcOffset.DCenable = 1;
        ch->ctrlParams.dcOffset.IQenable = 1;
        ch->ctrlParams.decimation.enable = 0;
        ch->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
    }
    apply_bandwidth();
    apply_gain_params();

    last_error_.clear();
    return true;
}

void SdrplaySource::close() {
    stop();
    if (selected_) {
        sdrplay_api_ReleaseDevice(&device_);
        selected_ = false;
    }
    params_ = nullptr;
    if (api_open_) {
        sdrplay_api_Close();
        api_open_ = false;
    }
}

bool SdrplaySource::set_freq(uint64_t freq_hz) {
    if (!selected_) { last_error_ = "Device not open"; return false; }
    freq_hz_ = freq_hz;

    sdrplay_api_RxChannelParamsT* ch = rx_channel();
    if (!ch) { last_error_ = "Device parameters unavailable"; return false; }
    ch->tunerParams.rfFreq.rfHz = static_cast<double>(freq_hz);

    if (streaming_.load(std::memory_order_relaxed)) {
        sdrplay_api_ErrT err = sdrplay_api_Update(
            device_.dev, sdrplay_api_Tuner_A,
            sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            last_error_ = err_string(err);
            return false;
        }
    }
    return true;
}

bool SdrplaySource::set_sample_rate(uint32_t sample_rate) {
    if (!selected_) { last_error_ = "Device not open"; return false; }
    if (sample_rate > max_sample_rate()) sample_rate = max_sample_rate();
    sample_rate_ = sample_rate;

    if (params_ && params_->devParams)
        params_->devParams->fsFreq.fsHz = static_cast<double>(sample_rate);
    apply_bandwidth();

    // Sample-rate changes require a stop/start cycle; the app always stops
    // before reconfiguring.  If we are streaming, apply it live anyway.
    if (streaming_.load(std::memory_order_relaxed)) {
        sdrplay_api_ErrT err = sdrplay_api_Update(
            device_.dev, sdrplay_api_Tuner_A,
            sdrplay_api_Update_Tuner_BwType, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            last_error_ = err_string(err);
            return false;
        }
    }
    return true;
}

bool SdrplaySource::set_lna_gain(uint32_t gain_db) {
    if (!selected_) { last_error_ = "Device not open"; return false; }
    lna_gain_ = gain_db;
    apply_gain_params();
    if (streaming_.load(std::memory_order_relaxed)) {
        sdrplay_api_ErrT err = sdrplay_api_Update(
            device_.dev, sdrplay_api_Tuner_A,
            sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            last_error_ = err_string(err);
            return false;
        }
    }
    return true;
}

bool SdrplaySource::set_vga_gain(uint32_t gain_db) {
    if (!selected_) { last_error_ = "Device not open"; return false; }
    vga_gain_ = gain_db;
    apply_gain_params();
    if (streaming_.load(std::memory_order_relaxed)) {
        sdrplay_api_ErrT err = sdrplay_api_Update(
            device_.dev, sdrplay_api_Tuner_A,
            sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
        if (err != sdrplay_api_Success) {
            last_error_ = err_string(err);
            return false;
        }
    }
    return true;
}

bool SdrplaySource::set_amp_enable(bool enable) {
    // SDRplay receivers have no switchable front-end amplifier; accepted for
    // interface compatibility.
    amp_ = enable;
    (void)amp_;
    return true;
}

void SdrplaySource::stream_callback(short* xi, short* xq,
                                    sdrplay_api_StreamCbParamsT* /*params*/,
                                    unsigned int numSamples, unsigned int /*reset*/,
                                    void* cb_context) {
    SdrplaySource* self = static_cast<SdrplaySource*>(cb_context);
    if (!self || !self->streaming_.load(std::memory_order_relaxed)) return;
    if (!self->callback_ || numSamples == 0) return;

    if (self->conv_buf_.size() < numSamples)
        self->conv_buf_.resize(numSamples);

    constexpr float scale = 1.0f / 32768.0f;
    for (unsigned int i = 0; i < numSamples; ++i) {
        self->conv_buf_[i] = std::complex<float>(
            static_cast<float>(xi[i]) * scale,
            static_cast<float>(xq[i]) * scale);
    }
    self->callback_(self->conv_buf_.data(), numSamples);
}

void SdrplaySource::event_callback(sdrplay_api_EventT event_id,
                                   sdrplay_api_TunerSelectT /*tuner*/,
                                   sdrplay_api_EventParamsT* params,
                                   void* cb_context) {
    SdrplaySource* self = static_cast<SdrplaySource*>(cb_context);
    if (!self) return;
    switch (event_id) {
    case sdrplay_api_DeviceFailure:
        self->last_error_ = "SDRplay device failure";
        break;
    case sdrplay_api_DeviceRemoved:
        self->last_error_ = "SDRplay device removed";
        self->streaming_.store(false, std::memory_order_relaxed);
        break;
    case sdrplay_api_PowerOverloadChange:
        if (params &&
            params->powerOverloadParams.powerOverloadChangeType ==
                sdrplay_api_Overload_Corrected) {
            // Acknowledge so the API resumes streaming.
            sdrplay_api_Update(
                self->device_.dev, sdrplay_api_Tuner_A,
                sdrplay_api_Update_Ctrl_OverloadMsgAck,
                sdrplay_api_Update_Ext1_None);
        }
        break;
    default:
        break;
    }
}

bool SdrplaySource::start(IQCallback cb) {
    if (!selected_) { last_error_ = "Device not open"; return false; }
    if (streaming_.load(std::memory_order_relaxed)) return true;

    callback_ = std::move(cb);
    streaming_.store(true, std::memory_order_relaxed);

    sdrplay_api_CallbackFnsT fns{};
    fns.StreamACbFn = &SdrplaySource::stream_callback;
    fns.StreamBCbFn = nullptr;
    fns.EventCbFn   = &SdrplaySource::event_callback;

    sdrplay_api_ErrT err = sdrplay_api_Init(device_.dev, &fns, this);
    if (err != sdrplay_api_Success) {
        last_error_ = err_string(err);
        streaming_.store(false, std::memory_order_relaxed);
        return false;
    }
    initialized_ = true;
    return true;
}

bool SdrplaySource::stop() {
    if (!streaming_.load(std::memory_order_relaxed) && !initialized_)
        return true;
    streaming_.store(false, std::memory_order_relaxed);
    if (initialized_) {
        sdrplay_api_Uninit(device_.dev);
        initialized_ = false;
    }
    return true;
}

} // namespace dedective

#endif // DEDECTIVE_HAVE_SDRPLAY
