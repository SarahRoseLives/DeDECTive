/*
 * DeDECTive — DECT Scanner & Voice Decoder (CLI)
 *
 * Default mode: voice-follow — scans the full band for DECT calls, automatically
 * tunes to active voice channels, and plays decoded audio.
 * Press 'n'/'p' to cycle through active voice channels, 'q' to quit.
 *
 * Build:  see CMakeLists.txt
 * Usage:  dedective [options]
 */

#include "iq_source.h"
#include "source_factory.h"
#include "phase_diff.h"
#include "packet_receiver.h"
#include "packet_decoder.h"
#include "dect_channels.h"
#include "wideband_monitor.h"
#include "hopping_scanner.h"
#include "audio_output.h"
#include "dc_blocker.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

using namespace dedective;
using Clock = std::chrono::steady_clock;

// ── Console initialisation ────────────────────────────────────────────────────
// On Windows the default console code page is not UTF-8 and ANSI/VT escape
// sequences are disabled, which makes the box-drawing UI render as garbage.
// Enable both so the terminal UI looks the same as on Linux.
static void setup_console() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (h && h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &mode)) {
        SetConsoleMode(h, mode | ENABLE_PROCESSED_OUTPUT |
                            ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }
#endif
}

// ── Global signal handler for clean Ctrl-C exit ───────────────────────────────
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// ── Blocking read of a single character from stdin ────────────────────────────
static int read_char_blocking() {
#ifdef _WIN32
    int c = _getch();
    return c;                       // returns the key directly (no echo)
#else
    unsigned char c = 0;
    return (::read(STDIN_FILENO, &c, 1) == 1) ? (int)c : -1;
#endif
}

// ── RAII raw terminal for non-blocking key reads ──────────────────────────────
struct RawTerminal {
#ifdef _WIN32
    bool active = false;
    void enable() { active = true; }
    ~RawTerminal() = default;
    int read_key() {
        if (!active) return -1;
        if (_kbhit()) return _getch();
        return -1;
    }
#else
    termios orig{};
    bool    active = false;

    void enable() {
        if (active) return;
        tcgetattr(STDIN_FILENO, &orig);
        termios raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active = true;
    }
    ~RawTerminal() { if (active) tcsetattr(STDIN_FILENO, TCSANOW, &orig); }

    int read_key() {
        if (!active) return -1;
        char c;
        return (::read(STDIN_FILENO, &c, 1) == 1) ? (unsigned char)c : -1;
    }
#endif
};

// ── Narrowband voice-decode context ───────────────────────────────────────────
struct NarrowbandState {
    PhaseDiff                        phase_diff;
    DCBlocker                        dc_blocker;
    std::unique_ptr<PacketReceiver>  receiver;
    std::unique_ptr<PacketDecoder>   decoder;

    // Snapshot of part info
    std::mutex  mutex;
    PartInfo    parts[MAX_PARTS];
    int         part_count = 0;
    uint64_t    packets_seen = 0;

    // Audio mixing (source callback thread only)
    int16_t     mix_frame[80] = {};
    int         mix_first_rx_id = -1;
};

// ── Voice-follow mode ─────────────────────────────────────────────────────────
static int voice_follow_mode(IqSource& source,
                             uint32_t lna_gain, uint32_t vga_gain,
                             bool amp_enable, DectBand band)
{
    const auto& channels = dect_channels(band);
    const uint64_t center_freq = dect_center_freq(band);
    const bool full_band = source.supports_full_wideband();

    // Scanner — full-band FFT for wideband-capable devices, otherwise hopping.
    WidebandMonitor monitor;
    monitor.set_band(band);
    std::unique_ptr<HoppingScanner> hopper;
    if (!full_band) {
        hopper = std::make_unique<HoppingScanner>(source);
        hopper->set_band(band);
    }

    auto scanner_snapshot = [&]() -> WidebandSnapshot {
        return full_band ? monitor.snapshot() : hopper->snapshot();
    };
    auto scanner_update = [&]() {
        if (full_band) monitor.update_visuals();
        else           hopper->update_visuals();
    };
    auto scanner_render = [&]() {
        if (full_band) monitor.render_frame();
        else           hopper->render_frame();
    };

    // Narrowband pipeline (created on demand)
    NarrowbandState nb;
    AudioOutput audio;

    // State machine
    enum class Mode { SCANNING, NARROWBAND };
    Mode  mode = Mode::SCANNING;
    int   tuned_channel = -1;
    bool  voice_was_present = false;
    Clock::time_point voice_lost_at;
    constexpr auto FOLLOW_TIMEOUT = std::chrono::seconds(2);

    std::vector<int> voice_channels;

    RawTerminal term;
    term.enable();

    auto apply_gains = [&]() -> bool {
        return source.set_lna_gain(lna_gain) &&
               source.set_vga_gain(vga_gain) &&
               source.set_amp_enable(amp_enable);
    };

    // ── Start scanning ──
    auto start_scanning = [&]() -> bool {
        source.stop();
        if (full_band) {
            if (!source.set_sample_rate(WIDEBAND_SAMPLE_RATE) ||
                !apply_gains() ||
                !source.set_freq(center_freq)) {
                std::fprintf(stderr, "SDR config failed: %s\n", source.last_error());
                return false;
            }
            if (!source.start([&](const std::complex<float>* s, size_t n) {
                    monitor.ingest(s, n);
                })) {
                std::fprintf(stderr, "SDR start failed: %s\n", source.last_error());
                return false;
            }
        } else {
            if (!apply_gains()) {
                std::fprintf(stderr, "SDR gain config failed: %s\n", source.last_error());
                return false;
            }
            if (!hopper->start()) {
                std::fprintf(stderr, "SDR start failed: %s\n", source.last_error());
                return false;
            }
        }
        mode = Mode::SCANNING;
        tuned_channel = -1;
        voice_was_present = false;
        return true;
    };

    // ── Start narrowband on a specific channel ──
    auto start_narrowband = [&](int ch_idx) -> bool {
        source.stop();

        nb.phase_diff = PhaseDiff();
        nb.dc_blocker.reset();
        nb.packets_seen = 0;
        nb.part_count = 0;
        nb.mix_first_rx_id = -1;

        nb.decoder = std::make_unique<PacketDecoder>(
            [&](const PartInfo parts[], int count) {
                std::lock_guard<std::mutex> lock(nb.mutex);
                nb.part_count = count;
                for (int i = 0; i < count; ++i)
                    nb.parts[i] = parts[i];
            },
            [&](int rx_id, const int16_t* pcm, size_t count) {
                if (count != 80) { audio.write_samples(pcm, count); return; }
                if (nb.mix_first_rx_id < 0) {
                    std::memcpy(nb.mix_frame, pcm, 80 * sizeof(int16_t));
                    nb.mix_first_rx_id = rx_id;
                } else if (rx_id != nb.mix_first_rx_id) {
                    for (int i = 0; i < 80; ++i) {
                        int32_t m = (int32_t)nb.mix_frame[i] + (int32_t)pcm[i];
                        nb.mix_frame[i] = (int16_t)std::clamp(m, -32768, 32767);
                    }
                    audio.write_samples(nb.mix_frame, 80);
                    nb.mix_first_rx_id = -1;
                } else {
                    audio.write_samples(nb.mix_frame, 80);
                    std::memcpy(nb.mix_frame, pcm, 80 * sizeof(int16_t));
                }
            }
        );

        nb.receiver = std::make_unique<PacketReceiver>(
            [&](const ReceivedPacket& pkt) {
                ++nb.packets_seen;
                nb.decoder->process_packet(pkt);
            },
            [&](int rx_id) { nb.decoder->notify_lost(rx_id); }
        );

        uint64_t freq = channels[ch_idx].freq_hz;
        if (!source.set_sample_rate(SAMPLE_RATE) ||
            !apply_gains() ||
            !source.set_freq(freq)) {
            std::fprintf(stderr, "SDR config failed: %s\n", source.last_error());
            return false;
        }

        if (!source.start([&](const std::complex<float>* samples, size_t n) {
                for (size_t i = 0; i < n; ++i) {
                    auto s = nb.dc_blocker.process(samples[i]);
                    float phase = nb.phase_diff.process(s);
                    nb.receiver->process_sample(phase);
                }
            })) {
            std::fprintf(stderr, "SDR start failed: %s\n", source.last_error());
            return false;
        }

        if (!audio.is_running()) audio.start();
        mode = Mode::NARROWBAND;
        tuned_channel = ch_idx;
        voice_was_present = false;
        return true;
    };

    // ── Render scanning status ──
    auto render_scanning = [&]() {
        std::printf("\x1b[H\x1b[J");  // clear screen
        std::printf("DeDECTive — Voice Follow Mode\n");
        std::printf("  Band: %s  LNA: %u dB  VGA: %u dB  Source: %s\n",
                    band == DectBand::US ? "US (1920 MHz)" : "EU (1880 MHz)",
                    lna_gain, vga_gain, source.name());
        std::printf("  Mode: SCANNING for DECT calls...\n\n");

        auto snap = scanner_snapshot();
        if (!snap.ready) {
            std::printf("  Filling scan buffer...\n");
        } else {
            std::printf("  %-4s  %-14s  %-8s  %-6s  %-8s\n",
                        "Ch", "Freq (MHz)", "Active", "Voice", "Pkts");
            std::printf("  %-4s  %-14s  %-8s  %-6s  %-8s\n",
                        "──", "──────────────", "────────", "──────", "────────");
            voice_channels.clear();
            for (size_t i = 0; i < channels.size(); ++i) {
                const auto& c = snap.channels[i];
                if (c.voice_detected) voice_channels.push_back(static_cast<int>(i));
                std::printf("  %-4d  %-14.3f  %-8s  %-6s  %-8llu",
                            c.channel_number, c.freq_hz / 1e6,
                            c.active ? "YES" : "·",
                            c.voice_detected ? "\x1b[32mYES\x1b[0m" : "·",
                            (unsigned long long)c.packets_seen);
                if (c.voice_detected) std::printf("  ← voice");
                std::printf("\n");
            }
        }

        std::printf("\n  [n] Next  [p] Prev  [q] Quit\n");
        std::fflush(stdout);
    };

    // ── Render narrowband status ──
    auto render_narrowband = [&]() {
        std::printf("\x1b[H\x1b[J");
        std::printf("DeDECTive — Voice Follow Mode\n");
        std::printf("  Band: %s  LNA: %u dB  VGA: %u dB  Source: %s\n",
                    band == DectBand::US ? "US (1920 MHz)" : "EU (1880 MHz)",
                    lna_gain, vga_gain, source.name());
        std::printf("  Mode: NARROWBAND — Channel %d  %.3f MHz\n",
                    channels[tuned_channel].number,
                    channels[tuned_channel].freq_hz / 1e6);
        std::printf("  Audio: %s  Packets: %llu\n\n",
                    audio.is_running() ? "playing" : "stopped",
                    (unsigned long long)nb.packets_seen);

        {
            std::lock_guard<std::mutex> lock(nb.mutex);
            if (nb.part_count == 0) {
                std::printf("  Waiting for DECT sync...\n");
            } else {
                std::printf("  %-12s  %-4s  %-6s  %-4s  %-5s  %-8s  %-6s  %-10s  %-10s  %-8s\n",
                            "ID", "Type", "Voice", "Qt", "Slot", "Pkts", "CRC%", "Decoded", "XCRC Fail", "Skipped");
                std::printf("  %-12s  %-4s  %-6s  %-4s  %-5s  %-8s  %-6s  %-10s  %-10s  %-8s\n",
                            "────────────", "────", "──────", "────", "─────", "────────", "──────",
                            "──────────", "──────────", "────────");
                for (int i = 0; i < nb.part_count; ++i) {
                    const auto& p = nb.parts[i];
                    int crc_pct = (p.packets_ok + p.packets_bad_crc > 0)
                        ? (int)(100 * p.packets_ok / (p.packets_ok + p.packets_bad_crc)) : 0;
                    std::printf("  %-12s  %-4s  ",
                                p.part_id_valid ? p.part_id_hex().c_str() : "??????????",
                                p.type_str().c_str());
                    if (p.voice_present)
                        std::printf("\x1b[32m%-6s\x1b[0m  ", "YES");
                    else
                        std::printf("%-6s  ", "no");
                    std::printf("%-4s  %-5d  %-8llu  %-6d  %-10llu  %-10llu  %-8llu\n",
                                p.qt_synced ? "YES" : "no",
                                p.slot,
                                (unsigned long long)p.packets_ok,
                                crc_pct,
                                (unsigned long long)p.voice_frames_ok,
                                (unsigned long long)p.voice_xcrc_fail,
                                (unsigned long long)p.voice_skipped);
                }
            }
        }

        if (!voice_channels.empty()) {
            std::printf("\n  Voice channels:");
            for (int vi : voice_channels) {
                if (vi == tuned_channel)
                    std::printf(" \x1b[33m[%d]\x1b[0m", channels[vi].number);
                else
                    std::printf(" %d", channels[vi].number);
            }
            std::printf("\n");
        }

        std::printf("\n  [n] Next  [p] Prev  [q] Quit\n");
        std::fflush(stdout);
    };

    // ── Cycle to next/prev voice channel ──
    auto cycle_channel = [&](int dir) {
        if (voice_channels.empty()) return;
        if (tuned_channel < 0) {
            start_narrowband(voice_channels[0]);
            return;
        }
        auto it = std::find(voice_channels.begin(), voice_channels.end(), tuned_channel);
        int pos = (it != voice_channels.end())
                  ? static_cast<int>(it - voice_channels.begin()) : 0;
        pos = (pos + dir + static_cast<int>(voice_channels.size()))
              % static_cast<int>(voice_channels.size());
        int next = voice_channels[pos];
        if (next != tuned_channel)
            start_narrowband(next);
    };

    // ── Start scanning ──
    if (!start_scanning()) return 1;

    // ── Main loop ──
    while (g_running) {
        int key = term.read_key();
        if (key == 'q' || key == 'Q') break;
        if (key == 'n' || key == 'N') cycle_channel(+1);
        if (key == 'p' || key == 'P') cycle_channel(-1);

        if (mode == Mode::SCANNING) {
            scanner_update();
            auto snap = scanner_snapshot();

            voice_channels.clear();
            if (snap.ready) {
                for (size_t i = 0; i < channels.size(); ++i)
                    if (snap.channels[i].voice_detected)
                        voice_channels.push_back(static_cast<int>(i));
            }

            if (!voice_channels.empty()) {
                start_narrowband(voice_channels[0]);
            } else {
                render_scanning();
            }
        } else {
            bool voice_now = false;
            {
                std::lock_guard<std::mutex> lock(nb.mutex);
                for (int i = 0; i < nb.part_count; ++i)
                    if (nb.parts[i].voice_present) { voice_now = true; break; }
            }

            if (voice_now) {
                voice_was_present = true;
            } else if (voice_was_present) {
                voice_was_present = false;
                voice_lost_at = Clock::now();
            }

            if (!voice_now && !voice_was_present &&
                nb.packets_seen > 0 &&
                Clock::now() - voice_lost_at > FOLLOW_TIMEOUT) {
                start_scanning();
            } else {
                render_narrowband();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    source.stop();
    audio.stop();
    std::printf("\x1b[H\x1b[J");
    std::printf("DeDECTive — stopped.\n");
    return 0;
}

// ── Per-channel scanning context (legacy -c mode) ─────────────────────────────
struct ScanContext {
    PhaseDiff      phase_diff;
    PacketReceiver receiver;
    PacketDecoder  decoder;
    AudioOutput*   audio_out = nullptr;
    std::mutex     print_mutex;
    uint64_t       cur_freq_hz  = 0;
    int            cur_channel  = -1;
    uint64_t       packets_seen = 0;

    ScanContext(int channel, uint64_t freq_hz, AudioOutput* audio = nullptr)
        : receiver(
            [this](const ReceivedPacket& pkt) {
                ++packets_seen;
                decoder.process_packet(pkt);
            },
            [this](int rx_id) { decoder.notify_lost(rx_id); }
          )
        , decoder(
            [this](const PartInfo parts[], int count) { print_parts(parts, count); },
            [this](int rx_id, const int16_t* pcm, size_t count) {
                if (audio_out) audio_out->write_samples(pcm, count);
            }
          )
        , audio_out(audio), cur_freq_hz(freq_hz), cur_channel(channel)
    {}

    void print_parts(const PartInfo* parts, int count) {
        std::lock_guard<std::mutex> lock(print_mutex);
        std::printf("\r  [Ch %d  %.3f MHz]  %d device(s):\n",
                    cur_channel, cur_freq_hz / 1e6, count);
        for (int i = 0; i < count; ++i) {
            const PartInfo& p = parts[i];
            int crc_pct = (p.packets_ok + p.packets_bad_crc > 0)
                ? (int)(100 * p.packets_ok / (p.packets_ok + p.packets_bad_crc)) : 0;
            std::printf("    rx%d  ID: %-10s  Type: %-3s  Voice: %-3s"
                        "  Pkts: %4llu  CRC ok: %3d%%\n",
                        p.rx_id,
                        p.part_id_valid ? p.part_id_hex().c_str() : "??????????",
                        p.type_str().c_str(), p.voice_present ? "YES" : "no",
                        (unsigned long long)p.packets_ok, crc_pct);
        }
        std::fflush(stdout);
    }
};

static void iq_callback(ScanContext* ctx, const std::complex<float>* samples, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        float phase = ctx->phase_diff.process(samples[i]);
        ctx->receiver.process_sample(phase);
    }
}

// ── Usage ─────────────────────────────────────────────────────────────────────
static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\nModes:\n"
        "  (default)  Voice follow — scans for DECT calls, auto-tunes and decodes voice\n"
        "  -W         Wideband monitor (no auto-follow)\n"
        "  -c <ch>    Fixed channel scan\n"
        "\nOptions:\n"
        "  -S <src>   SDR source: auto | sdrplay | pluto | hackrf (default: auto)\n"
        "  -g <lna>   LNA gain  0-40 (default: 32)\n"
        "  -v <vga>   VGA gain  0-62 (default: 20)\n"
        "  -a         Enable RF amplifier (HackRF +14 dB; ignored by SDRplay)\n"
        "  -d <secs>  Dwell time per channel, seconds (default: 3)\n"
        "  -l         Loop continuously until Ctrl-C\n"
        "  -e         EU band (1880 MHz) instead of US (1920 MHz)\n"
        "  -h         Show this help\n"
        "\nKeyboard (voice follow mode):\n"
        "  n          Next voice channel\n"
        "  p          Previous voice channel\n"
        "  q          Quit\n",
        prog);
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    setup_console();

    uint32_t   lna_gain    = 32;
    uint32_t   vga_gain    = 20;
    bool       amp_enable  = false;
    int        dwell_secs  = 3;
    bool       dwell_set   = false;
    int        fixed_chan  = -1;
    bool       loop        = false;
    bool       wideband    = false;
    bool       voice_en    = false;
    DectBand   band        = DectBand::US;
    SourceType source_type = SourceType::Auto;

    for (int i = 1; i < argc; ++i) {
        if      (strcmp(argv[i], "-S") == 0 && i+1 < argc) {
            if (!parse_source_type(argv[++i], source_type)) {
                std::fprintf(stderr, "Unknown source: %s\n", argv[i]);
                return 1;
            }
        }
        else if (strcmp(argv[i], "-g") == 0 && i+1 < argc) lna_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-v") == 0 && i+1 < argc) vga_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-d") == 0 && i+1 < argc) { dwell_secs = atoi(argv[++i]); dwell_set = true; }
        else if (strcmp(argv[i], "-c") == 0 && i+1 < argc) fixed_chan  = atoi(argv[++i]);
        else if (strcmp(argv[i], "-a") == 0)               amp_enable  = true;
        else if (strcmp(argv[i], "-l") == 0)               loop        = true;
        else if (strcmp(argv[i], "-W") == 0)               wideband    = true;
        else if (strcmp(argv[i], "-V") == 0)               voice_en    = true;
        else if (strcmp(argv[i], "-e") == 0)               band        = DectBand::EU;
        else if (strcmp(argv[i], "-h") == 0) { print_usage(argv[0]); return 0; }
        else { std::fprintf(stderr, "Unknown option: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    // ── Legal disclaimer ──
    std::printf(
        "════════════════════════════════════════════════════════════════\n"
        "  DeDECTive — DECT Scanner & Voice Decoder\n"
        "════════════════════════════════════════════════════════════════\n"
        "\n"
        "  WARNING: Intercepting or decoding wireless communications\n"
        "  you are not authorized to access may violate federal, state,\n"
        "  and local laws. This software is intended for use in\n"
        "  controlled lab/test environments only.\n"
        "\n"
        "  The author takes no responsibility for how this software\n"
        "  is used. By proceeding, you accept full legal responsibility\n"
        "  for your actions.\n"
        "\n"
        "════════════════════════════════════════════════════════════════\n"
        "\n"
        "  Do you accept these terms? [y/N]: ");
    std::fflush(stdout);
    {
        int response = read_char_blocking();
        if (response != 'y' && response != 'Y') {
            std::printf("\nDisclaimer not accepted. Exiting.\n");
            return 0;
        }
#ifndef _WIN32
        char drain;
        while (::read(STDIN_FILENO, &drain, 1) == 1 && drain != '\n') {}
#endif
    }
    std::printf("\n");

    // Open the requested SDR source (Auto tries each backend in turn)
    std::printf("DeDECTive — Opening %s source...\n",
                source_type == SourceType::Auto ? "auto-selected"
                                                : source_type_name(source_type));
    std::fflush(stdout);

    std::string open_error;
    std::unique_ptr<IqSource> source = open_source(source_type, open_error);
    if (!source) {
        std::fprintf(stderr, "Unable to open an SDR source:\n%s\n", open_error.c_str());
        auto names = available_sources();
        if (!names.empty()) {
            std::fprintf(stderr, "Compiled-in backends:");
            for (auto& n : names) std::fprintf(stderr, " %s", n.c_str());
            std::fprintf(stderr, "\n");
        }
        return 1;
    }
    std::printf("DeDECTive — Using %s\n", source->name());

    // ── Default mode: voice follow ──
    if (!wideband && fixed_chan < 0) {
        return voice_follow_mode(*source, lna_gain, vga_gain, amp_enable, band);
    }

    // ── Legacy wideband monitor ──
    if (wideband) {
        if (!dwell_set && !loop) dwell_secs = 10;

        const auto& channels = dect_channels(band);
        AudioOutput audio;

        if (source->supports_full_wideband()) {
            if (!source->set_sample_rate(WIDEBAND_SAMPLE_RATE) ||
                !source->set_lna_gain(lna_gain) ||
                !source->set_vga_gain(vga_gain) ||
                !source->set_amp_enable(amp_enable)) {
                std::fprintf(stderr, "Config failed: %s\n", source->last_error());
                return 1;
            }

            WidebandMonitor monitor;
            monitor.set_band(band);
            if (voice_en && audio.start()) {
                monitor.set_audio_output(&audio);
                std::printf("Voice decode enabled\n");
            }
            if (!source->set_freq(dect_center_freq(band)) ||
                !source->start([&monitor](const std::complex<float>* s, size_t n) {
                    monitor.ingest(s, n);
                })) {
                std::fprintf(stderr, "Start failed: %s\n", source->last_error());
                return 1;
            }

            auto deadline = Clock::now() + std::chrono::seconds(dwell_secs);
            while (g_running && (loop || Clock::now() < deadline)) {
                monitor.render_frame();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            source->stop();
        } else {
            if (!source->set_lna_gain(lna_gain) ||
                !source->set_vga_gain(vga_gain) ||
                !source->set_amp_enable(amp_enable)) {
                std::fprintf(stderr, "Config failed: %s\n", source->last_error());
                return 1;
            }

            HoppingScanner hopper(*source);
            hopper.set_band(band);
            if (voice_en && audio.start()) {
                hopper.set_audio_output(&audio);
                std::printf("Voice decode enabled\n");
            }
            if (!hopper.start()) {
                std::fprintf(stderr, "Start failed: %s\n", source->last_error());
                return 1;
            }

            auto deadline = Clock::now() + std::chrono::seconds(dwell_secs);
            while (g_running && (loop || Clock::now() < deadline)) {
                hopper.render_frame();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            hopper.stop();
        }

        std::printf("\nDone.\n");
        return 0;
    }

    // ── Legacy fixed-channel scan ──
    const auto& channels = dect_channels(band);
    if (!source->set_sample_rate(SAMPLE_RATE) ||
        !source->set_lna_gain(lna_gain) ||
        !source->set_vga_gain(vga_gain) ||
        !source->set_amp_enable(amp_enable)) {
        std::fprintf(stderr, "Config failed: %s\n", source->last_error());
        return 1;
    }

    std::vector<DectChannel> scan_list;
    if (fixed_chan >= 0 && fixed_chan < (int)channels.size())
        scan_list.push_back(channels[fixed_chan]);
    else
        for (auto& ch : channels) scan_list.push_back(ch);

    AudioOutput scan_audio;
    if (voice_en && scan_audio.start())
        std::printf("Voice decode enabled\n");

    do {
        for (auto& chan : scan_list) {
            if (!g_running) break;
            std::printf("Scanning channel %d  %s  (%d s)...\n",
                        chan.number, chan.label, dwell_secs);
            AudioOutput* aptr = (voice_en && scan_audio.is_running()) ? &scan_audio : nullptr;
            ScanContext ctx(chan.number, chan.freq_hz, aptr);

            if (!source->set_freq(chan.freq_hz) ||
                !source->start([&ctx](const std::complex<float>* s, size_t n) {
                    iq_callback(&ctx, s, n);
                })) {
                std::fprintf(stderr, "  Failed: %s — skipping\n", source->last_error());
                continue;
            }

            auto deadline = Clock::now() + std::chrono::seconds(dwell_secs);
            bool reported = false;
            while (g_running && Clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                if (!reported && ctx.packets_seen > 0) {
                    reported = true;
                    std::printf("  *** DECT activity! Extending dwell...\n");
                    deadline = Clock::now() + std::chrono::seconds(dwell_secs);
                }
            }
            source->stop();

            if (ctx.packets_seen > 0)
                std::printf("  Summary ch%d: %llu sync locks, %d active device(s)\n",
                            chan.number, (unsigned long long)ctx.packets_seen,
                            ctx.decoder.active_count());
            else
                std::printf("  No DECT sync detected.\n");
        }
    } while (loop && g_running);

    std::printf("\nDone.\n");
    return 0;
}
