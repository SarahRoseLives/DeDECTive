# DeDECTive

> **⚠️ Legal Warning**
>
> Intercepting or decoding wireless communications you are not authorized to access may violate federal, state, and local laws, including the U.S. Electronic Communications Privacy Act (18 U.S.C. § 2511) and equivalent laws in other jurisdictions. **This software is intended for use in controlled lab/test environments only.** The author takes no responsibility for how this software is used. By using DeDECTive, you accept full legal responsibility for your actions.

`DeDECTive` is a cross-platform DECT 6.0 scanner and voice decoder for the HackRF One and SDRplay RSP receivers. It includes both a Dear ImGui GUI and a terminal CLI — no GNU Radio required. It builds on Linux and Windows.

## GUI

### Wideband Scanner

Captures the entire DECT band (US or EU) in a single HackRF pass at 18.432 Msps with live FFT, waterfall, and per-channel activity/voice detection.

![GUI — Wideband Scanner](main.png)

- Live FFT and waterfall display with adjustable dB range sliders
- Per-channel activity detection with hysteresis for stable readings
- Per-channel DECT packet decode showing RFP/PP parts, voice presence, and Qt sync status
- Channel details table with one-click **Tune** buttons to switch to narrowband

### Narrowband Voice Decode

Tunes to a single DECT channel for full G.721 ADPCM voice decode with RFP + PP audio mixed into one stream.

![GUI — Narrowband Voice Decode](call_following.png)

- G.721 ADPCM decoding of both RFP (base station) and PP (handset) audio
- RFP + PP audio mixed so both sides of the call are heard simultaneously
- Real-time PulseAudio playback with volume control
- Timeslot tracking (slots 0–11 downlink, 12–23 uplink)
- Sequence gap filling with G.721 comfort noise for smooth audio
- **Follow Call** — auto-retunes when a call moves to a different channel; returns to wideband scan after 2 seconds of silence

## CLI

### Voice Follow (default)

Run `dedective` with no flags to enter voice-follow mode. It scans the full band for DECT calls and automatically tunes to the first active voice channel.

![CLI — Scanning for Calls](cli_default_start.png)

Once a call is found, it switches to narrowband and plays decoded audio via PulseAudio. Both RFP and PP sides are mixed. When the call ends, it returns to scanning.

![CLI — Active Call Decode](cli_active_call.png)

**Keyboard:** `n` next voice channel · `p` previous · `q` quit

### Wideband Monitor (`-W`)

A text-based wideband monitor showing per-channel power bars, part counts, voice status, and packet counts.

![CLI — Wideband Monitor](cli_wideband_scanner.png)

## Features

- **Multiple SDR backends** — HackRF One, SDRplay RSP, and ADALM-PLUTO (PlutoSDR) receivers, auto-detected and selectable at runtime (`-S` flag / GUI dropdown)
- **Wideband + channel-hopping scanning** — full-band FFT capture on HackRF; automatic channel-hopping scan on bandwidth-limited SDRplay receivers
- **US and EU band support** — selectable in GUI dropdown or CLI (`-e` flag)
- **Call following** — automatic channel handoff tracking in both GUI and CLI
- **DC spike correction** — IIR high-pass filter removes the receiver's DC offset
- **FFT smoothing** — fast attack / slow decay for clean spectrum display
- **Audio mixing** — RFP + PP decoded and mixed into a single stream
- **Gap filling** — G.721 comfort noise across missed frames prevents audio clicks
- **Cross-platform audio** — miniaudio output (WASAPI on Windows, ALSA/PulseAudio on Linux)
- **Configurable gains** — LNA/VGA controls mapped per backend

## Build

The only hard dependency is a C++17 compiler; audio uses the vendored
[miniaudio](https://github.com/mackron/miniaudio) single header. SDR backends
and the GUI are enabled automatically when their SDKs/dependencies are found.

### Linux

Dependencies: `libhackrf` (optional), `SDL2`, `imgui`, `OpenGL` (GUI only).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Windows (MSYS2 MinGW-w64)

Install the toolchain from an MSYS2 **MINGW64** shell, then build. The vendored
SDL2 + imgui in `third_party/` and the installed SDRplay API are picked up
automatically. All required runtime DLLs (MinGW runtime, SDL2, SDRplay, libusb)
are copied next to the executables, so `build-win\` is self-contained.

```powershell
cmake -S . -B build-win -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-win -j
```

If the SDRplay API is installed elsewhere, pass `-DSDRPLAY_ROOT=<path>`.
Useful options: `-DBUILD_GUI=OFF`, `-DBUILD_CLI=OFF`,
`-DDEDECTIVE_WITH_SDRPLAY=OFF`, `-DDEDECTIVE_WITH_HACKRF=OFF`,
`-DDEDECTIVE_WITH_PLUTO=OFF`.

#### PlutoSDR on Windows

PlutoSDR support uses `libiio` (`pacman -S mingw-w64-x86_64-libiio`). The
Pluto's USB interfaces need a driver before libiio can reach them — without it
Windows lists the device's `RNDIS` and `IIO` interfaces with an install error.
Install one of:

- the Analog Devices **PlutoSDR USB driver** package (recommended; provides both
  the RNDIS network link and the WinUSB IIO interface), or
- **WinUSB** on the `IIO` interface (e.g. via Zadig), or
- the built-in Windows **Remote NDIS** driver on the `RNDIS` interface, then use
  the network link.

The backend scans for contexts automatically; you can force a URI with
`PLUTO_URI`, e.g. `PLUTO_URI=ip:192.168.2.1` or `PLUTO_URI=usb:<bus>.<port>`.

## Usage

```bash
# GUI (recommended)
./build/dedective_gui

# CLI — voice follow (default), auto-select SDR
./build/dedective

# CLI — force a specific SDR backend
./build/dedective -S sdrplay
./build/dedective -S pluto
./build/dedective -S hackrf

# CLI — EU band
./build/dedective -e

# CLI — wideband / hopping monitor
./build/dedective -W -l

# CLI — fixed channel scan with voice decode
./build/dedective -c 0 -V -l
```

## Architecture

The codebase is split into a reusable core library and two frontends:

- **dedective_core** — static library: SDR source abstraction (`IqSource`) with
  HackRF/SDRplay backends, DECT packet receiver/decoder, wideband monitor,
  channel-hopping scanner, miniaudio output, G.721 codec
- **dedective** — CLI frontend with voice-follow, wideband monitor, and channel scan modes
- **dedective_gui** — Dear ImGui + SDL2 + OpenGL frontend

`IqSource` hides the radio hardware; `Scanner` hides whether capture is a
single full-band pass (`WidebandMonitor`) or a per-channel hop (`HoppingScanner`).

## Reference

The DECT protocol pipeline (phase-difference demodulation, packet reception, A/B-field decode, scramble tables) was ported and adapted from [gr-dect2](https://github.com/pavelyazev/gr-dect2) by Pavel Yazev.
