#include "hopping_scanner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace dedective {

namespace {

constexpr float POWER_EPSILON = 1e-12f;

uint8_t slot_state_for_part(const PartInfo& part) {
    if (part.voice_present)
        return part.type == PartType::RFP ? 4 : 5;
    if (part.qt_synced)
        return part.type == PartType::RFP ? 2 : 3;
    return 1;
}

} // namespace

HoppingScanner::HoppingScanner(IqSource& source)
    : source_(source) {
    channels_       = &dect_channels(DectBand::US);
    center_freq_hz_ = dect_center_freq(DectBand::US);
    fft_db_.fill(-110.0f);
    smoothed_db_.fill(-110.0f);
    waterfall_.assign(WATERFALL_ROWS * PLOT_BINS, -110.0f);
}

HoppingScanner::~HoppingScanner() {
    stop();
}

void HoppingScanner::set_band(DectBand band) {
    std::lock_guard<std::mutex> lock(mutex_);
    band_           = band;
    channels_       = &dect_channels(band);
    center_freq_hz_ = dect_center_freq(band);

    for (auto& s : stats_) s = ChannelStat{};
    current_ = ChannelStat{};
    phase_diff_.reset();
    dc_blocker_.reset();
    smoothed_db_.fill(-110.0f);
    fft_db_.fill(-110.0f);
    std::fill(waterfall_.begin(), waterfall_.end(), -110.0f);
    waterfall_head_ = 0;
    waterfall_filled_ = 0;
    current_ch_ = 0;
    power_accum_ = 0.0;
    power_count_ = 0;
    noise_floor_db_ = -110.0f;

    if (running_) {
        receiver_.reset();
        decoder_.reset();
        build_pipeline_locked();
        source_.set_freq((*channels_)[current_ch_].freq_hz);
        hop_start_ = std::chrono::steady_clock::now();
    }
}

void HoppingScanner::build_pipeline_locked() {
    receiver_ = std::make_unique<PacketReceiver>(
        [this](const ReceivedPacket& pkt) {
            ++current_.packets;
            if (decoder_) decoder_->process_packet(pkt);
        },
        [this](int rx_id) {
            if (decoder_) decoder_->notify_lost(rx_id);
        });

    decoder_ = std::make_unique<PacketDecoder>(
        [this](const PartInfo parts[], int count) {
            bool voice = false;
            bool qt    = false;
            uint64_t ok = 0, fail = 0, skip = 0;
            std::array<uint8_t, DECT_SLOT_COUNT> slots{};
            for (int i = 0; i < count; ++i) {
                if (parts[i].voice_present) voice = true;
                if (parts[i].qt_synced)     qt    = true;
                ok   += parts[i].voice_frames_ok;
                fail += parts[i].voice_xcrc_fail;
                skip += parts[i].voice_skipped;
                if (parts[i].slot < DECT_SLOT_COUNT) {
                    slots[parts[i].slot] =
                        std::max(slots[parts[i].slot], slot_state_for_part(parts[i]));
                }
            }
            current_.voice = voice;
            current_.qt = qt;
            current_.active_parts = count;
            current_.voice_ok = ok;
            current_.xcrc_fail = fail;
            current_.skipped = skip;
            current_.slots = slots;
        },
        [this](int /*rx_id*/, const int16_t* pcm, size_t count) {
            if (!audio_out_) return;
            int selected = audio_channel_;
            if (selected < 0) {
                audio_channel_ = current_ch_;
                selected = current_ch_;
            }
            if (selected == current_ch_)
                audio_out_->write_samples(pcm, count);
        });
}

void HoppingScanner::handle_samples(const std::complex<float>* samples, size_t n) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !receiver_) return;

    for (size_t i = 0; i < n; ++i) {
        std::complex<float> s = dc_block_enabled_
            ? dc_blocker_.process(samples[i])
            : samples[i];
        power_accum_ += static_cast<double>(std::norm(s));
        ++power_count_;
        const float phase = phase_diff_.process(s);
        receiver_->process_sample(phase);
    }
}

void HoppingScanner::finalize_current_locked() {
    if (power_count_ > 0) {
        const double mean = power_accum_ / static_cast<double>(power_count_);
        current_.power_db = 10.0f * std::log10(static_cast<float>(mean) + POWER_EPSILON);
    }
    stats_[current_ch_] = current_;
}

void HoppingScanner::push_waterfall_row_locked() {
    std::array<float, PLOT_BINS> row;
    row.fill(-110.0f);

    std::array<float, NUM_DECT_CHANNELS> power{};
    for (size_t i = 0; i < NUM_DECT_CHANNELS; ++i)
        power[i] = stats_[i].power_db;

    std::array<float, NUM_DECT_CHANNELS> sorted = power;
    std::nth_element(sorted.begin(), sorted.begin() + NUM_DECT_CHANNELS / 2, sorted.end());
    noise_floor_db_ = sorted[NUM_DECT_CHANNELS / 2];

    for (size_t i = 0; i < NUM_DECT_CHANNELS; ++i) {
        const double offset = static_cast<double>((*channels_)[i].freq_hz) -
                              static_cast<double>(center_freq_hz_);
        const float norm = 0.5f +
            static_cast<float>(offset) / static_cast<float>(WIDEBAND_SAMPLE_RATE);
        int bin = static_cast<int>(std::lround(norm * static_cast<float>(PLOT_BINS)));
        bin = std::clamp(bin, 0, static_cast<int>(PLOT_BINS) - 1);
        const float v = stats_[i].power_db;
        row[bin] = std::max(row[bin], v);
        if (bin > 0) row[bin - 1] = std::max(row[bin - 1], v - 6.0f);
        if (bin < static_cast<int>(PLOT_BINS) - 1)
            row[bin + 1] = std::max(row[bin + 1], v - 6.0f);
    }

    for (size_t c = 0; c < PLOT_BINS; ++c) {
        if (row[c] > smoothed_db_[c])
            smoothed_db_[c] = 0.6f * smoothed_db_[c] + 0.4f * row[c];
        else
            smoothed_db_[c] = 0.88f * smoothed_db_[c] + 0.12f * row[c];
    }
    fft_db_ = smoothed_db_;

    const size_t off = waterfall_head_ * PLOT_BINS;
    std::copy(row.begin(), row.end(),
              waterfall_.begin() + static_cast<std::ptrdiff_t>(off));
    waterfall_head_ = (waterfall_head_ + 1) % WATERFALL_ROWS;
    waterfall_filled_ = std::min(waterfall_filled_ + 1, WATERFALL_ROWS);
}

bool HoppingScanner::start() {
    if (running_) return true;

    int next_channel = 0;
    uint64_t next_freq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!source_.set_sample_rate(SAMPLE_RATE)) return false;

        build_pipeline_locked();
        current_ = ChannelStat{};
        phase_diff_.reset();
        dc_blocker_.reset();
        current_ch_ = 0;
        power_accum_ = 0.0;
        power_count_ = 0;
        hop_start_ = std::chrono::steady_clock::now();

        next_channel = current_ch_;
        next_freq = (*channels_)[next_channel].freq_hz;
        if (!source_.set_freq(next_freq)) return false;
        running_ = true;
    }

    if (!source_.start([this](const std::complex<float>* s, size_t n) {
            handle_samples(s, n);
        })) {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        return false;
    }
    return true;
}

void HoppingScanner::stop() {
    source_.stop();
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    receiver_.reset();
    decoder_.reset();
}

void HoppingScanner::set_dwell_ms(int ms) {
    dwell_ms_.store(std::clamp(ms, 50, 5000), std::memory_order_relaxed);
}

bool HoppingScanner::update_visuals() {
    uint64_t next_freq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return false;

        const auto now = std::chrono::steady_clock::now();
        if (now - hop_start_ <
            std::chrono::milliseconds(dwell_ms_.load(std::memory_order_relaxed))) {
            return true;
        }

        finalize_current_locked();
        push_waterfall_row_locked();

        current_ch_ = (current_ch_ + 1) % static_cast<int>(NUM_DECT_CHANNELS);
        current_ = ChannelStat{};
        phase_diff_.reset();
        dc_blocker_.reset();
        build_pipeline_locked();
        power_accum_ = 0.0;
        power_count_ = 0;
        hop_start_ = now;

        next_freq = (*channels_)[current_ch_].freq_hz;
    }

    // Retune without holding the pipeline lock (the stream callback may be
    // running and needs it).
    source_.set_freq(next_freq);
    return true;
}

WidebandSnapshot HoppingScanner::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);

    WidebandSnapshot out;
    out.ready = running_;
    out.noise_floor_db = noise_floor_db_;
    out.buffered_samples = 0;

    for (size_t i = 0; i < NUM_DECT_CHANNELS; ++i) {
        const ChannelStat& c = stats_[i];
        WidebandChannelView& view = out.channels[i];
        view.channel_number    = (*channels_)[i].number;
        view.freq_hz           = (*channels_)[i].freq_hz;
        view.smoothed_power_db = c.power_db;
        view.relative_power_db = c.power_db - noise_floor_db_;
        view.active            = c.voice || c.active_parts > 0 ||
                                 c.packets > 0 || view.relative_power_db >= 3.0f;
        view.voice_detected    = c.voice;
        view.qt_synced         = c.qt;
        view.active_parts      = c.active_parts;
        view.packets_seen      = c.packets;
        view.voice_frames_ok   = c.voice_ok;
        view.voice_xcrc_fail   = c.xcrc_fail;
        view.voice_skipped     = c.skipped;
        out.slot_state[i]      = c.slots;
    }

    out.fft_db.assign(fft_db_.begin(), fft_db_.end());
    out.waterfall_cols = PLOT_BINS;
    out.waterfall_rows = WATERFALL_ROWS;
    out.waterfall_db.assign(WATERFALL_ROWS * PLOT_BINS, -110.0f);

    if (waterfall_filled_ > 0) {
        const bool full = waterfall_filled_ == WATERFALL_ROWS;
        const size_t start = full ? waterfall_head_ : 0;
        const size_t padding = WATERFALL_ROWS - waterfall_filled_;
        for (size_t row = 0; row < waterfall_filled_; ++row) {
            const size_t src_row = full ? ((start + row) % WATERFALL_ROWS) : row;
            const size_t dst_row = padding + row;
            std::copy_n(
                waterfall_.begin() + static_cast<std::ptrdiff_t>(src_row * PLOT_BINS),
                PLOT_BINS,
                out.waterfall_db.begin() + static_cast<std::ptrdiff_t>(dst_row * PLOT_BINS));
        }
    }

    return out;
}

void HoppingScanner::render_frame() {
    update_visuals();
    const WidebandSnapshot current = snapshot();

    std::printf("\x1b[H\x1b[J");
    std::printf("DeDECTive hopping scanner (channel-by-channel)\n");
    std::printf("  Center: %.3f MHz  Sample rate: %.3f Msps  Dwell: %d ms\n",
                center_freq_hz_ / 1e6, SAMPLE_RATE / 1e6, dwell_ms_.load());
    std::printf("  Bars show channel power relative to the current band median.\n\n");

    for (size_t i = 0; i < NUM_DECT_CHANNELS; ++i) {
        const auto& channel = current.channels[i];
        const int bar_len = std::clamp(
            static_cast<int>(std::lround(channel.relative_power_db * 2.0f)), 0, 28);

        char bar[29];
        std::fill(std::begin(bar), std::end(bar) - 1, '.');
        bar[28] = '\0';
        for (int j = 0; j < bar_len; ++j) bar[j] = '#';

        std::printf("  Ch %-2d %.3f MHz  %+5.1f dB  [%s]  parts:%d  voice:%s  pkts:%llu\n",
                    channel.channel_number,
                    channel.freq_hz / 1e6,
                    channel.relative_power_db,
                    bar,
                    channel.active_parts,
                    channel.voice_detected ? "YES" : " no",
                    static_cast<unsigned long long>(channel.packets_seen));
    }

    std::printf("\nPress Ctrl-C to stop.\n");
    std::fflush(stdout);
}

void HoppingScanner::set_audio_output(AudioOutput* audio) {
    std::lock_guard<std::mutex> lock(mutex_);
    audio_out_ = audio;
}

void HoppingScanner::set_audio_channel(int channel_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    audio_channel_ = channel_index;
}

int HoppingScanner::audio_channel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return audio_channel_;
}

} // namespace dedective
