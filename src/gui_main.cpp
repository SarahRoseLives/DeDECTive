#include "sdr_source.h"
#include "hackrf_source.h"
#include "pluto_source.h"
#include "wideband_monitor.h"
#include "packet_receiver.h"
#include "packet_decoder.h"
#include "phase_diff.h"
#include "audio_output.h"
#include "dc_blocker.h"
#include "dect_channels.h"

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#if __has_include("imgui_impl_sdl2.h")
#include "imgui_impl_sdl2.h"
#else
#include "imgui_impl_sdl.h"
#endif

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

using namespace dedective;

namespace {

enum class CaptureMode { IDLE, WIDEBAND, NARROWBAND };

struct GuiState {
    float fft_min_db = -13.0f;
    float fft_max_db = 40.0f;
    float waterfall_min_db = 4.0f;
    float waterfall_max_db = 35.0f;
    bool  audio_enabled = true;
    float audio_volume = 0.8f;
    bool  audio_muted = false;
    bool  dc_block = true;     // DC offset correction enabled
    bool  follow_call = false; // auto-retune when call moves channels
    int   band_index = 0;      // 0 = US, 1 = EU
};

// Narrowband state: single-channel decode pipeline
struct NarrowbandContext {
    PhaseDiff                        phase_diff;
    DCBlocker                        dc_blocker;
    std::unique_ptr<PacketReceiver>  receiver;
    std::unique_ptr<PacketDecoder>   decoder;
    int                              channel_index = -1;
    uint64_t                         packets_seen  = 0;

    // Snapshot of part info for GUI display (protected by mutex)
    std::mutex                       mutex;
    PartInfo                         parts[MAX_PARTS];
    int                              part_count = 0;

    // Voice mixing: combine RFP + PP into one frame so the audio ring
    // buffer isn't filled at 2× the drain rate.  Accessed only from
    // the HackRF callback thread — no lock needed.
    int16_t  mix_frame[80] = {};
    int      mix_first_rx_id = -1;   // -1 = empty
};

struct CaptureController {
    std::unique_ptr<SdrSource> sdr;
    WidebandMonitor  monitor;
    AudioOutput      audio;
    bool             device_open = false;
    CaptureMode      mode = CaptureMode::IDLE;
    uint32_t         lna_gain = 32;
    uint32_t         vga_gain = 20;
    bool             amp_enable = false;
    bool             dc_block_enabled = true;
    DectBand         active_band = DectBand::US;
    std::string      last_error;

    // Narrowband state
    NarrowbandContext nb;

    // ── Wideband capture ─────────────────────────────────────────────
    bool start_wideband() {
        if (mode != CaptureMode::IDLE) stop();

        last_error.clear();
        if (!open_device()) return false;

        if (!sdr->set_sample_rate(WIDEBAND_SAMPLE_RATE) ||
            !apply_gains() ||
            !sdr->set_freq(monitor.center_freq())) {
            last_error = sdr->last_error();
            return false;
        }

        if (!sdr->start([this](const std::complex<float>* samples, size_t n) {
                monitor.ingest(samples, n);
            })) {
            last_error = sdr->last_error();
            return false;
        }

        mode = CaptureMode::WIDEBAND;
        return true;
    }

    // ── Narrowband (voice decode) capture ────────────────────────────
    bool start_narrowband(int channel_index) {
        const auto& channels = dect_channels(active_band);
        if (channel_index < 0 || channel_index >= static_cast<int>(channels.size()))
            return false;
        if (mode != CaptureMode::IDLE) stop();

        last_error.clear();
        if (!open_device()) return false;

        // Set up single-channel pipeline
        nb.channel_index = channel_index;
        nb.packets_seen  = 0;
        nb.part_count    = 0;
        nb.phase_diff    = PhaseDiff();
        nb.dc_blocker.reset();
        nb.mix_first_rx_id = -1;

        nb.decoder = std::make_unique<PacketDecoder>(
            // on_update: snapshot parts for GUI
            [this](const PartInfo parts[], int count) {
                std::lock_guard<std::mutex> lock(nb.mutex);
                nb.part_count = count;
                for (int i = 0; i < count; ++i) {
                    nb.parts[i] = parts[i];
                }
            },
            // on_voice: mix RFP + PP into one frame before writing to audio.
            // Without mixing, both sides each produce 80 samples per 10 ms
            // frame, filling the 8 kHz ring buffer at 2× the drain rate.
            [this](int rx_id, const int16_t* pcm, size_t count) {
                if (count != 80) { audio.write_samples(pcm, count); return; }

                if (nb.mix_first_rx_id < 0) {
                    // First contributor — store
                    std::memcpy(nb.mix_frame, pcm, 80 * sizeof(int16_t));
                    nb.mix_first_rx_id = rx_id;
                } else if (rx_id != nb.mix_first_rx_id) {
                    // Second contributor (other part) — mix and flush
                    for (int i = 0; i < 80; ++i) {
                        int32_t m = static_cast<int32_t>(nb.mix_frame[i])
                                  + static_cast<int32_t>(pcm[i]);
                        nb.mix_frame[i] = static_cast<int16_t>(
                            std::clamp(m, (int32_t)-32768, (int32_t)32767));
                    }
                    audio.write_samples(nb.mix_frame, 80);
                    nb.mix_first_rx_id = -1;
                } else {
                    // Same part again (other side has no voice) — flush old, store new
                    audio.write_samples(nb.mix_frame, 80);
                    std::memcpy(nb.mix_frame, pcm, 80 * sizeof(int16_t));
                }
            }
        );

        nb.receiver = std::make_unique<PacketReceiver>(
            [this](const ReceivedPacket& pkt) {
                ++nb.packets_seen;
                nb.decoder->process_packet(pkt);
            },
            [this](int rx_id) {
                nb.decoder->notify_lost(rx_id);
            }
        );

        uint64_t freq = channels[channel_index].freq_hz;
        if (!sdr->set_sample_rate(SAMPLE_RATE) ||
            !apply_gains() ||
            !sdr->set_freq(freq)) {
            last_error = sdr->last_error();
            return false;
        }

        if (!sdr->start([this](const std::complex<float>* samples, size_t n) {
                for (size_t i = 0; i < n; ++i) {
                    auto s = dc_block_enabled
                        ? nb.dc_blocker.process(samples[i]) : samples[i];
                    float phase = nb.phase_diff.process(s);
                    nb.receiver->process_sample(phase);
                }
            })) {
            last_error = sdr->last_error();
            return false;
        }

        mode = CaptureMode::NARROWBAND;
        return true;
    }

    void start_audio() {
        if (!audio.is_running())
            audio.start();
    }

    void stop() {
        if (mode != CaptureMode::IDLE) {
            sdr->stop();
        }
        if (device_open) {
            sdr->close();
            device_open = false;
        }
        audio.stop();
        nb.receiver.reset();
        nb.decoder.reset();
        nb.channel_index = -1;
        mode = CaptureMode::IDLE;
    }

    ~CaptureController() {
        stop();
    }

private:
    bool open_device() {
        if (!device_open) {
            // bodge in pluto
            bool use_pluto = false;
             if (!use_pluto) {
                sdr = std::make_unique<HackrfSource>();
                std::printf("DeDECTive — Opening HackRF... ");
            }
            else {
                sdr = std::make_unique<PlutoSource>();
                std::printf("DeDECTive — Opening Pluto... ");
            }
            if (!sdr->open()) {
                last_error = sdr->last_error();
                return false;
            }
            device_open = true;
        }
        return true;
    }

    bool apply_gains() {
        return sdr->set_lna_gain(lna_gain) &&
               sdr->set_vga_gain(vga_gain) &&
               sdr->set_amp_enable(amp_enable);
    }
};

ImU32 heatmap_color(float t) {
    t = std::clamp(t, 0.0f, 1.0f);

    ImVec4 c0(0.02f, 0.02f, 0.08f, 1.0f);
    ImVec4 c1(0.05f, 0.30f, 0.70f, 1.0f);
    ImVec4 c2(0.05f, 0.75f, 0.75f, 1.0f);
    ImVec4 c3(0.95f, 0.85f, 0.20f, 1.0f);
    ImVec4 c4(0.95f, 0.25f, 0.10f, 1.0f);

    auto lerp = [](const ImVec4& a, const ImVec4& b, float x) {
        return ImVec4(
            a.x + (b.x - a.x) * x,
            a.y + (b.y - a.y) * x,
            a.z + (b.z - a.z) * x,
            1.0f);
    };

    ImVec4 c;
    if (t < 0.25f) c = lerp(c0, c1, t / 0.25f);
    else if (t < 0.5f) c = lerp(c1, c2, (t - 0.25f) / 0.25f);
    else if (t < 0.75f) c = lerp(c2, c3, (t - 0.5f) / 0.25f);
    else c = lerp(c3, c4, (t - 0.75f) / 0.25f);

    return ImGui::ColorConvertFloat4ToU32(c);
}

ImU32 slot_state_color(uint8_t state) {
    switch (state) {
    case 1: return IM_COL32(90, 110, 135, 255);
    case 2: return IM_COL32(90, 170, 255, 255);
    case 3: return IM_COL32(255, 185, 90, 255);
    case 4: return IM_COL32(80, 255, 180, 255);
    case 5: return IM_COL32(255, 110, 110, 255);
    default: return IM_COL32(18, 22, 30, 255);
    }
}

void draw_wideband_slot_grid(const WidebandSnapshot& snapshot, const ImVec2& size) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 top_left = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("wideband_slot_grid", size);
    const ImVec2 bottom_right(top_left.x + size.x, top_left.y + size.y);
    draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(10, 12, 18, 255));
    draw_list->AddRect(top_left, bottom_right, IM_COL32(80, 90, 110, 255));

    if (!snapshot.ready) {
        draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f),
                           IM_COL32(220, 220, 220, 255),
                           "Waiting for slot activity...");
        return;
    }

    const float left_pad = 34.0f;
    const float top_pad = 18.0f;
    const float grid_w = std::max(1.0f, size.x - left_pad);
    const float grid_h = std::max(1.0f, size.y - top_pad);
    const float cell_w = grid_w / static_cast<float>(DECT_SLOT_COUNT);
    const float cell_h = grid_h / static_cast<float>(snapshot.channels.size());

    for (size_t slot = 0; slot < DECT_SLOT_COUNT; ++slot) {
        if (slot % 4 != 0 && slot != DECT_SLOT_COUNT - 1) continue;
        char label[8];
        std::snprintf(label, sizeof(label), "%zu", slot);
        draw_list->AddText(ImVec2(top_left.x + left_pad + cell_w * static_cast<float>(slot) + 2.0f,
                                  top_left.y + 2.0f),
                           IM_COL32(210, 210, 210, 255), label);
    }

    for (size_t row = 0; row < snapshot.channels.size(); ++row) {
        const auto& channel = snapshot.channels[row];
        const float y = top_left.y + top_pad + cell_h * static_cast<float>(row);
        const ImU32 label_color = channel.voice_detected
            ? IM_COL32(255, 110, 110, 255)
            : (channel.active ? IM_COL32(255, 210, 90, 255) : IM_COL32(180, 180, 180, 255));
        char label[8];
        std::snprintf(label, sizeof(label), "%d", channel.channel_number);
        draw_list->AddText(ImVec2(top_left.x + 6.0f, y + 1.0f), label_color, label);

        for (size_t slot = 0; slot < DECT_SLOT_COUNT; ++slot) {
            const uint8_t state = snapshot.slot_state[row][slot];
            const ImVec2 p0(top_left.x + left_pad + cell_w * static_cast<float>(slot), y);
            const ImVec2 p1(p0.x + cell_w - 1.0f, p0.y + cell_h - 1.0f);
            draw_list->AddRectFilled(p0, p1, slot_state_color(state));
        }
    }
}

void draw_part_summary_card(const char* title,
                            const PartInfo* part,
                            const ImVec4& accent,
                            const ImVec2& size) {
    ImGui::BeginChild(title, size, true);
    ImGui::TextColored(accent, "%s", title);
    ImGui::Separator();

    if (!part) {
        ImGui::TextDisabled("Not detected");
        ImGui::EndChild();
        return;
    }

    ImGui::Text("Part ID: %s", part->part_id_valid ? part->part_id_hex().c_str() : "---");
    ImGui::Text("Slot: %u", static_cast<unsigned>(part->slot));
    ImGui::Text("Qt sync: %s", part->qt_synced ? "YES" : "no");
    ImGui::Text("Voice: %s", part->voice_present ? "YES" : "no");
    ImGui::Spacing();
    ImGui::Text("Decoded: %llu", static_cast<unsigned long long>(part->voice_frames_ok));
    ImGui::Text("Pkts OK: %llu", static_cast<unsigned long long>(part->packets_ok));
    ImGui::Text("Bad CRC: %llu", static_cast<unsigned long long>(part->packets_bad_crc));
    ImGui::Text("XCRC fail: %llu", static_cast<unsigned long long>(part->voice_xcrc_fail));
    ImGui::Text("Skipped: %llu", static_cast<unsigned long long>(part->voice_skipped));
    ImGui::EndChild();
}

void draw_waterfall(const WidebandSnapshot& snapshot,
                    const ImVec2& size,
                    float min_db,
                    float max_db,
                    uint64_t center_freq_hz) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 top_left = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("waterfall", size);
    const ImVec2 bottom_right(top_left.x + size.x, top_left.y + size.y);
    draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(10, 12, 18, 255));

    if (!snapshot.ready || snapshot.waterfall_rows == 0 || snapshot.waterfall_cols == 0) {
        draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f),
                           IM_COL32(220, 220, 220, 255),
                           "Waiting for wideband samples...");
        return;
    }

    const float cell_w = size.x / static_cast<float>(snapshot.waterfall_cols);
    const float cell_h = size.y / static_cast<float>(snapshot.waterfall_rows);

    for (size_t row = 0; row < snapshot.waterfall_rows; ++row) {
        for (size_t col = 0; col < snapshot.waterfall_cols; ++col) {
            const float db = snapshot.waterfall_db[row * snapshot.waterfall_cols + col];
            const float norm = (db - min_db) / std::max(1.0f, max_db - min_db);
            const ImU32 color = heatmap_color(norm);

            const ImVec2 p0(top_left.x + cell_w * static_cast<float>(col),
                            top_left.y + cell_h * static_cast<float>(row));
            const ImVec2 p1(p0.x + cell_w + 1.0f, p0.y + cell_h + 1.0f);
            draw_list->AddRectFilled(p0, p1, color);
        }
    }

    for (const auto& channel : snapshot.channels) {
        const float norm =
            0.5f + (static_cast<float>(static_cast<int64_t>(channel.freq_hz) -
                                       static_cast<int64_t>(center_freq_hz)) /
                    static_cast<float>(WIDEBAND_SAMPLE_RATE));
        const float x = top_left.x + norm * size.x;
        const ImU32 line_color = channel.voice_detected
            ? IM_COL32(255, 96, 96, 255)
            : (channel.active ? IM_COL32(255, 214, 80, 220) : IM_COL32(180, 180, 180, 120));

        draw_list->AddLine(ImVec2(x, top_left.y), ImVec2(x, bottom_right.y), line_color, 1.0f);
        char label[16];
        std::snprintf(label, sizeof(label), "Ch%d", channel.channel_number);
        draw_list->AddText(ImVec2(x + 2.0f, top_left.y + 2.0f), line_color, label);
    }
}

void draw_fft_plot(const WidebandSnapshot& snapshot,
                   const ImVec2& size,
                   float min_db,
                   float max_db,
                   uint64_t center_freq_hz) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 top_left = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("fft_plot", size);
    const ImVec2 bottom_right(top_left.x + size.x, top_left.y + size.y);
    draw_list->AddRectFilled(top_left, bottom_right, IM_COL32(10, 12, 18, 255));
    draw_list->AddRect(top_left, bottom_right, IM_COL32(80, 90, 110, 255));

    if (!snapshot.ready || snapshot.fft_db.empty()) {
        draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f),
                           IM_COL32(220, 220, 220, 255),
                           "Waiting for FFT samples...");
        return;
    }

    const float db_span = std::max(5.0f, max_db - min_db);
    const size_t sample_count = snapshot.fft_db.size();
    std::vector<ImVec2> points;
    points.reserve(sample_count);

    for (size_t i = 0; i < sample_count; ++i) {
        const float x = top_left.x + (static_cast<float>(i) / static_cast<float>(sample_count - 1)) * size.x;
        const float norm = std::clamp((snapshot.fft_db[i] - min_db) / db_span, 0.0f, 1.0f);
        const float y = bottom_right.y - norm * size.y;
        points.emplace_back(x, y);
    }

    for (int tick = 1; tick <= 4; ++tick) {
        const float y = top_left.y + (size.y * tick / 5.0f);
        draw_list->AddLine(ImVec2(top_left.x, y), ImVec2(bottom_right.x, y),
                           IM_COL32(35, 45, 60, 255), 1.0f);
    }

    draw_list->AddPolyline(points.data(), static_cast<int>(points.size()),
                           IM_COL32(110, 220, 255, 255), false, 2.0f);

    for (const auto& channel : snapshot.channels) {
        const float norm =
            0.5f + (static_cast<float>(static_cast<int64_t>(channel.freq_hz) -
                                       static_cast<int64_t>(center_freq_hz)) /
                    static_cast<float>(WIDEBAND_SAMPLE_RATE));
        const float x = top_left.x + norm * size.x;
        const ImU32 color = channel.voice_detected
            ? IM_COL32(255, 96, 96, 255)
            : (channel.active ? IM_COL32(255, 214, 80, 200) : IM_COL32(160, 160, 160, 100));
        draw_list->AddLine(ImVec2(x, top_left.y), ImVec2(x, bottom_right.y), color, 1.0f);
    }

    char max_label[16];
    char min_label[16];
    std::snprintf(max_label, sizeof(max_label), "%.0f dB", max_db);
    std::snprintf(min_label, sizeof(min_label), "%.0f dB", min_db);
    draw_list->AddText(ImVec2(top_left.x + 6.0f, top_left.y + 4.0f),
                       IM_COL32(220, 220, 220, 255), max_label);
    draw_list->AddText(ImVec2(top_left.x + 6.0f, bottom_right.y - 18.0f),
                       IM_COL32(220, 220, 220, 255), min_label);
}

void draw_channel_details_panel(const WidebandSnapshot& snapshot,
                                const ImVec2& size,
                                CaptureController& controller,
                                bool show_title = true) {
    ImGui::BeginChild("channel_details_panel", size, true);
    if (show_title) {
        ImGui::Text("Channel Details");
        ImGui::Separator();
    }

    if (ImGui::BeginTable("channel_details_table", 12,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                          ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
                          ImVec2(-1.0f, -1.0f))) {
        ImGui::TableSetupColumn("Ch");
        ImGui::TableSetupColumn("MHz");
        ImGui::TableSetupColumn("Rel dB");
        ImGui::TableSetupColumn("State");
        ImGui::TableSetupColumn("Parts");
        ImGui::TableSetupColumn("Voice");
        ImGui::TableSetupColumn("Qt");
        ImGui::TableSetupColumn("Decoded");
        ImGui::TableSetupColumn("XCRC Fail");
        ImGui::TableSetupColumn("Skipped");
        ImGui::TableSetupColumn("Packets");
        ImGui::TableSetupColumn("##tune");
        ImGui::TableHeadersRow();

        for (size_t ci = 0; ci < snapshot.channels.size(); ++ci) {
            const auto& channel = snapshot.channels[ci];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            if (channel.voice_detected)
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "Ch %d", channel.channel_number);
            else if (channel.active)
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.35f, 1.0f), "Ch %d", channel.channel_number);
            else
                ImGui::Text("Ch %d", channel.channel_number);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", channel.freq_hz / 1e6);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%+.1f", channel.relative_power_db);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%s", channel.active ? "ACTIVE" : "quiet");
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%d", channel.active_parts);
            ImGui::TableSetColumnIndex(5);
            if (channel.voice_detected)
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "YES");
            else
                ImGui::Text("no");
            ImGui::TableSetColumnIndex(6);
            if (channel.qt_synced)
                ImGui::Text("YES");
            else if (channel.active_parts > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "no");
            else
                ImGui::Text("-");
            ImGui::TableSetColumnIndex(7);
            if (channel.voice_frames_ok > 0)
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(channel.voice_frames_ok));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(8);
            if (channel.voice_xcrc_fail > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(channel.voice_xcrc_fail));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(9);
            if (channel.voice_skipped > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(channel.voice_skipped));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(10);
            ImGui::Text("%llu", static_cast<unsigned long long>(channel.packets_seen));
            ImGui::TableSetColumnIndex(11);
            ImGui::PushID(static_cast<int>(ci));
            if (channel.voice_detected || channel.active) {
                if (ImGui::SmallButton("Tune")) {
                    controller.stop();
                    if (controller.start_narrowband(static_cast<int>(ci)))
                        controller.start_audio();
                }
            }
            ImGui::PopID();
        }

        ImGui::EndTable();
    }

    ImGui::EndChild();
}

void draw_narrowband_panel(CaptureController& controller, const ImVec2& size) {
    ImGui::BeginChild("narrowband_panel", size, true);

    int ch_idx = controller.nb.channel_index;
    const auto& ch = dect_channels(controller.active_band)[ch_idx];
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.7f, 1.0f),
                       "Tuned: Channel %d — %.3f MHz  (narrowband voice decode)",
                       ch.number, ch.freq_hz / 1e6);
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 110.0f);
    if (ImGui::Button("Back to Scan", ImVec2(110.0f, 0.0f))) {
        controller.stop();
        if (controller.start_wideband())
            controller.start_audio();
    }
    ImGui::Separator();

    // Snapshot part info under lock
    PartInfo parts[MAX_PARTS];
    int part_count = 0;
    uint64_t pkts = 0;
    {
        std::lock_guard<std::mutex> lock(controller.nb.mutex);
        part_count = controller.nb.part_count;
        for (int i = 0; i < part_count; ++i)
            parts[i] = controller.nb.parts[i];
        pkts = controller.nb.packets_seen;
    }

    ImGui::Text("Packets received: %llu", static_cast<unsigned long long>(pkts));
    ImGui::Text("Parts detected: %d", part_count);
    ImGui::Spacing();
    const PartInfo* rfp_part = nullptr;
    const PartInfo* pp_part = nullptr;
    for (int i = 0; i < part_count; ++i) {
        if (!rfp_part && parts[i].type == PartType::RFP) rfp_part = &parts[i];
        if (!pp_part && parts[i].type == PartType::PP) pp_part = &parts[i];
    }

    const float cards_h = 165.0f;
    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const float card_w = (ImGui::GetContentRegionAvail().x - gap) * 0.5f;
    draw_part_summary_card("RFP Summary", rfp_part, ImVec4(0.5f, 0.8f, 1.0f, 1.0f),
                           ImVec2(card_w, cards_h));
    ImGui::SameLine();
    draw_part_summary_card("PP Summary", pp_part, ImVec4(1.0f, 0.8f, 0.5f, 1.0f),
                           ImVec2(card_w, cards_h));

    ImGui::Spacing();

    if (part_count > 0 && ImGui::BeginTable("nb_parts", 10,
            ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
            ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY,
            ImVec2(-1.0f, 0.0f))) {
        ImGui::TableSetupColumn("Type");
        ImGui::TableSetupColumn("Part ID");
        ImGui::TableSetupColumn("Slot");
        ImGui::TableSetupColumn("Voice");
        ImGui::TableSetupColumn("Qt Sync");
        ImGui::TableSetupColumn("Decoded");
        ImGui::TableSetupColumn("XCRC Fail");
        ImGui::TableSetupColumn("Skipped");
        ImGui::TableSetupColumn("Pkts OK");
        ImGui::TableSetupColumn("Bad CRC");
        ImGui::TableHeadersRow();

        for (int i = 0; i < part_count; ++i) {
            const PartInfo& p = parts[i];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextColored(
                p.type == PartType::RFP ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(1.0f, 0.8f, 0.5f, 1.0f),
                "%s", p.type_str().c_str());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%s", p.part_id_valid ? p.part_id_hex().c_str() : "---");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d", p.slot);
            ImGui::TableSetColumnIndex(3);
            if (p.voice_present)
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "YES");
            else
                ImGui::Text("no");
            ImGui::TableSetColumnIndex(4);
            if (p.qt_synced)
                ImGui::Text("YES");
            else
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "no");
            ImGui::TableSetColumnIndex(5);
            if (p.voice_frames_ok > 0)
                ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(p.voice_frames_ok));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(6);
            if (p.voice_xcrc_fail > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(p.voice_xcrc_fail));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(7);
            if (p.voice_skipped > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(p.voice_skipped));
            else
                ImGui::Text("0");
            ImGui::TableSetColumnIndex(8);
            ImGui::Text("%llu", static_cast<unsigned long long>(p.packets_ok));
            ImGui::TableSetColumnIndex(9);
            if (p.packets_bad_crc > 0)
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%llu",
                                   static_cast<unsigned long long>(p.packets_bad_crc));
            else
                ImGui::Text("0");
        }
        ImGui::EndTable();
    } else if (part_count == 0) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           "Listening for DECT packets on channel %d...", ch.number);
    }

    ImGui::EndChild();
}

void draw_vertical_range_controls(const char* min_label,
                                  const char* max_label,
                                  float* min_value,
                                  float* max_value,
                                  float min_limit,
                                  float max_limit,
                                  float min_gap) {
    ImGui::PushID(min_label);
    ImGui::BeginGroup();
    ImGui::TextUnformatted("dB");
    ImGui::VSliderFloat("##max", ImVec2(28.0f, 120.0f), max_value, min_limit, max_limit, "");
    ImGui::Text("%.0f", *max_value);
    ImGui::Spacing();
    ImGui::VSliderFloat("##min", ImVec2(28.0f, 120.0f), min_value, min_limit, max_limit, "");
    ImGui::Text("%.0f", *min_value);
    if (*max_value < *min_value + min_gap) {
        *max_value = *min_value + min_gap;
    }
    if (*max_value > max_limit) {
        *max_value = max_limit;
        *min_value = std::min(*min_value, *max_value - min_gap);
    }
    if (*min_value < min_limit) {
        *min_value = min_limit;
    }
    ImGui::TextUnformatted(max_label);
    ImGui::TextUnformatted(min_label);
    ImGui::EndGroup();
    ImGui::PopID();
}

bool save_screenshot_ppm(const std::string& path, int width, int height) {
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    out << "P6\n" << width << ' ' << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y) {
        out.write(reinterpret_cast<const char*>(pixels.data() + static_cast<size_t>(y) * width * 3),
                  static_cast<std::streamsize>(width) * 3);
    }
    return out.good();
}

} // namespace

int main(int argc, char* argv[]) {
    int demo_seconds = 0;
    std::string screenshot_path;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--demo-seconds" && i + 1 < argc) {
            demo_seconds = std::max(0, std::atoi(argv[++i]));
        } else if (std::string(argv[i]) == "--screenshot" && i + 1 < argc) {
            screenshot_path = argv[++i];
        }
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    SDL_Window* window = SDL_CreateWindow(
        "DeDECTive GUI",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1440, 900,
        static_cast<SDL_WindowFlags>(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI));
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 8.0f;
    style.FrameRounding = 4.0f;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    CaptureController controller;
    GuiState gui_state;
    const bool auto_start = demo_seconds > 0 || !screenshot_path.empty();
    bool screenshot_taken = false;
    auto auto_deadline = std::chrono::steady_clock::time_point{};
    bool done = false;
    bool disclaimer_accepted = false;

    // Follow-call state
    using clock = std::chrono::steady_clock;
    clock::time_point follow_voice_lost_at{};
    bool follow_had_voice = false;
    static constexpr auto FOLLOW_TIMEOUT = std::chrono::seconds(2);

    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                done = true;
            }
        }

        if (disclaimer_accepted && auto_start && controller.mode == CaptureMode::IDLE) {
            if (controller.start_wideband() && demo_seconds > 0) {
                auto_deadline = std::chrono::steady_clock::now() +
                                std::chrono::seconds(demo_seconds);
            }
        }

        const bool is_wideband  = controller.mode == CaptureMode::WIDEBAND;
        const bool is_narrowband = controller.mode == CaptureMode::NARROWBAND;
        const bool is_active    = is_wideband || is_narrowband;

        if (is_wideband) {
            controller.monitor.update_visuals();
        }
        const WidebandSnapshot snapshot = controller.monitor.snapshot();

        // ── Follow-call logic ────────────────────────────────────────
        if (gui_state.follow_call) {
            auto now = clock::now();

            if (is_narrowband) {
                // Check if any part still has voice
                bool any_voice = false;
                {
                    std::lock_guard<std::mutex> lock(controller.nb.mutex);
                    for (int i = 0; i < controller.nb.part_count; ++i)
                        if (controller.nb.parts[i].voice_present) { any_voice = true; break; }
                }
                if (any_voice) {
                    follow_had_voice = true;
                    follow_voice_lost_at = {};
                } else if (follow_had_voice) {
                    // Voice was present but now gone — start timeout
                    if (follow_voice_lost_at == clock::time_point{})
                        follow_voice_lost_at = now;
                    else if (now - follow_voice_lost_at > FOLLOW_TIMEOUT) {
                        // Timed out — return to wideband to scan for the call
                        controller.stop();
                        controller.start_wideband();
                        follow_had_voice = false;
                        follow_voice_lost_at = {};
                    }
                }
            } else if (is_wideband) {
                // Auto-tune to the first channel showing voice
                for (size_t i = 0; i < snapshot.channels.size(); ++i) {
                    if (snapshot.channels[i].voice_detected) {
                        controller.stop();
                        if (controller.start_narrowband(static_cast<int>(i))) {
                            if (gui_state.audio_enabled)
                                controller.start_audio();
                            follow_had_voice = false;
                            follow_voice_lost_at = {};
                        }
                        break;
                    }
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // ── Legal disclaimer modal ──────────────────────────────────
        if (!disclaimer_accepted) {
            ImGui::OpenPopup("Legal Disclaimer");
            ImVec2 center(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
            ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowSize(ImVec2(520, 0));
            if (ImGui::BeginPopupModal("Legal Disclaimer", nullptr,
                    ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
                ImGui::TextWrapped(
                    "WARNING: Intercepting or decoding wireless communications "
                    "you are not authorized to access may violate federal, state, "
                    "and local laws. This software is intended for use in "
                    "controlled lab/test environments only.");
                ImGui::Spacing();
                ImGui::TextWrapped(
                    "The author takes no responsibility for how this software "
                    "is used. By clicking Accept, you acknowledge that you accept "
                    "full legal responsibility for your actions.");
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                float button_w = 120.0f;
                float avail = ImGui::GetContentRegionAvail().x;
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - button_w * 2 - 20) * 0.5f);
                if (ImGui::Button("Accept", ImVec2(button_w, 0))) {
                    disclaimer_accepted = true;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine(0, 20);
                if (ImGui::Button("Decline", ImVec2(button_w, 0))) {
                    done = true;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }

        if (disclaimer_accepted) {

        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);
        ImGui::Begin("DeDECTive", nullptr,
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoTitleBar);

        if (is_narrowband) {
            const auto& ch = dect_channels(controller.active_band)[controller.nb.channel_index];
            ImGui::Text("DeDECTive — narrowband voice decode");
            ImGui::Text("Channel %d  %.3f MHz  Sample rate %.3f Msps",
                        ch.number, ch.freq_hz / 1e6, SAMPLE_RATE / 1e6);
        } else {
            ImGui::Text("DeDECTive — wideband scanner");
            ImGui::Text("Center %.3f MHz  Sample rate %.3f Msps",
                        controller.monitor.center_freq() / 1e6,
                        WIDEBAND_SAMPLE_RATE / 1e6);
        }
        ImGui::Separator();

        const float left_w = 270.0f;
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const float center_w = std::max(500.0f, ImGui::GetContentRegionAvail().x - left_w - spacing);

        // ── Left sidebar ────────────────────────────────────────────
        ImGui::BeginChild("settings_sidebar", ImVec2(left_w, 0), true);
        ImGui::Text("Capture Settings");
        ImGui::Separator();

        int lna_gain = static_cast<int>(controller.lna_gain);
        int vga_gain = static_cast<int>(controller.vga_gain);
        ImGui::BeginDisabled(is_active);
        if (ImGui::SliderInt("LNA gain", &lna_gain, 0, 40)) {
            controller.lna_gain = static_cast<uint32_t>(lna_gain);
        }
        if (ImGui::SliderInt("VGA gain", &vga_gain, 0, 62)) {
            controller.vga_gain = static_cast<uint32_t>(vga_gain);
        }
        ImGui::Checkbox("HackRF amp", &controller.amp_enable);
        ImGui::EndDisabled();

        if (ImGui::Checkbox("DC correction", &gui_state.dc_block)) {
            controller.dc_block_enabled = gui_state.dc_block;
            controller.monitor.set_dc_block(gui_state.dc_block);
        }

        ImGui::Spacing();
        ImGui::BeginDisabled(is_active);
        {
            const char* bands[] = { "US Band (1920 MHz)", "EU Band (1880 MHz)" };
            if (ImGui::Combo("Band", &gui_state.band_index, bands, 2)) {
                DectBand new_band = gui_state.band_index == 0 ? DectBand::US : DectBand::EU;
                controller.active_band = new_band;
                controller.monitor.set_band(new_band);
            }
        }
        ImGui::EndDisabled();

        if (!is_active) {
            if (ImGui::Button("Start wideband scan", ImVec2(-1.0f, 0.0f))) {
                controller.start_wideband();
            }
        } else {
            if (ImGui::Button("Stop capture", ImVec2(-1.0f, 0.0f))) {
                controller.stop();
            }
        }

        ImGui::Spacing();
        ImGui::Text("Status");
        if (is_narrowband) {
            ImGui::BulletText("Mode: NARROWBAND");
            ImGui::BulletText("Channel: %d", dect_channels(controller.active_band)[controller.nb.channel_index].number);
        } else if (is_wideband) {
            ImGui::BulletText("Mode: WIDEBAND");
            ImGui::BulletText("FFT buffer: %zu / %zu",
                              snapshot.buffered_samples, WidebandMonitor::FFT_SIZE);
            ImGui::BulletText("Noise floor: %.1f dB", snapshot.noise_floor_db);
        } else {
            ImGui::BulletText("Mode: idle");
        }

        if (!controller.last_error.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Last error");
            ImGui::TextWrapped("%s", controller.last_error.c_str());
        }

        // Audio controls — only relevant in narrowband mode
        if (is_narrowband) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Audio Output");
            ImGui::Separator();

            if (ImGui::Checkbox("Enable audio", &gui_state.audio_enabled)) {
                if (gui_state.audio_enabled) {
                    controller.start_audio();
                } else {
                    controller.audio.stop();
                }
            }

            if (ImGui::Checkbox("Mute", &gui_state.audio_muted)) {
                controller.audio.set_muted(gui_state.audio_muted);
            }

            if (ImGui::SliderFloat("Volume", &gui_state.audio_volume, 0.0f, 1.0f, "%.2f")) {
                controller.audio.set_volume(gui_state.audio_volume);
            }

            ImGui::BulletText("Audio: %s",
                              controller.audio.is_running() ? "playing" : "stopped");
        }

        // Follow call — narrowband only
        if (is_narrowband) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Text("Call Following");
            ImGui::Separator();
            ImGui::Checkbox("Follow call", &gui_state.follow_call);
            if (gui_state.follow_call) {
                ImGui::TextWrapped("Auto-retunes when the call "
                                   "moves to a different channel.");
            }
        }

        ImGui::EndChild();

        ImGui::SameLine();

        // ── Center panel ────────────────────────────────────────────
        ImGui::BeginChild("main_visuals", ImVec2(center_w, 0), false);

        if (is_narrowband) {
            // Narrowband mode: show detailed part info
            draw_narrowband_panel(controller, ImVec2(-1.0f, -1.0f));
        } else {
            // Wideband mode: FFT + waterfall + slots/details
            const float avail_h = ImGui::GetContentRegionAvail().y;
            const float panel_space_h = std::max(360.0f, avail_h - 90.0f);
            const float fft_h = std::clamp(panel_space_h * 0.18f, 105.0f, 135.0f);
            const float waterfall_h = std::clamp(panel_space_h * 0.30f, 155.0f, 210.0f);
            const float details_h = std::max(180.0f, panel_space_h - fft_h - waterfall_h);
            const float control_w = 56.0f;

            ImGui::Text("Wideband FFT");
            ImGui::BeginChild("fft_panel", ImVec2(-1.0f, fft_h + 8.0f), true);
            draw_fft_plot(snapshot,
                          ImVec2(ImGui::GetContentRegionAvail().x - control_w - ImGui::GetStyle().ItemSpacing.x,
                                 fft_h),
                          gui_state.fft_min_db,
                          gui_state.fft_max_db,
                          controller.monitor.center_freq());
            ImGui::SameLine();
            draw_vertical_range_controls("min", "max",
                                         &gui_state.fft_min_db, &gui_state.fft_max_db,
                                         -140.0f, 40.0f, 5.0f);
            ImGui::EndChild();

            ImGui::Spacing();
            ImGui::Text("Waterfall");
            ImGui::BeginChild("waterfall_panel", ImVec2(-1.0f, waterfall_h + 8.0f), true);
            draw_waterfall(snapshot,
                           ImVec2(ImGui::GetContentRegionAvail().x - control_w - ImGui::GetStyle().ItemSpacing.x,
                                  waterfall_h),
                           gui_state.waterfall_min_db,
                           gui_state.waterfall_max_db,
                           controller.monitor.center_freq());
            ImGui::SameLine();
            draw_vertical_range_controls("min", "max",
                                         &gui_state.waterfall_min_db, &gui_state.waterfall_max_db,
                                         -140.0f, 40.0f, 5.0f);
            ImGui::EndChild();

            ImGui::Spacing();
            ImGui::BeginChild("wideband_lower_panel", ImVec2(-1.0f, details_h), true);
            const float lower_avail_h = ImGui::GetContentRegionAvail().y;
            const float slot_grid_h = std::clamp(lower_avail_h * 0.34f, 110.0f, 145.0f);
            ImGui::TextUnformatted("Slots per Channel");
            draw_wideband_slot_grid(snapshot, ImVec2(ImGui::GetContentRegionAvail().x, slot_grid_h));
            ImGui::TextUnformatted("Blue/cyan = RFP slots, amber/red = PP slots, bright = voice.");
            ImGui::Spacing();
            draw_channel_details_panel(snapshot, ImVec2(-1.0f, -1.0f), controller, false);
            ImGui::EndChild();
        }

        ImGui::EndChild();

        ImGui::End();

        } // end if (disclaimer_accepted)

        ImGui::Render();
        glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
        glClearColor(0.04f, 0.05f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (!screenshot_path.empty() && !screenshot_taken && snapshot.ready) {
            int width = 0;
            int height = 0;
            SDL_GL_GetDrawableSize(window, &width, &height);
            screenshot_taken = save_screenshot_ppm(screenshot_path, width, height);
        }

        SDL_GL_SwapWindow(window);

        if (demo_seconds > 0 && is_active &&
            std::chrono::steady_clock::now() >= auto_deadline) {
            done = true;
        }
    }

    controller.stop();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
