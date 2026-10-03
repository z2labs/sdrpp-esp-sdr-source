# Wideband (Turbo) spectrum in SDR++

## What the ESP32-S3 can do

Besides the narrow real-IQ stream (250 / 125 / 62.5 kS/s, limited by the ~0.65 MB/s of the S3's USB Serial/JTAG port), ESP-SDR's **Turbo Mode (SPEC)** runs the FFT on the chip and streams only the spectrum: **16, 40 or 80 MHz wide**, 256 / 1024 / 2048 bins, up to ~1300 spectra per second, gapless, CRC-checked (`SPEC` command, `SPC1` frames; see `docs/spectrum.md` in [ESP-SDR](https://github.com/ESPARGOS/esp-sdr)).

80 MHz of IQ would need 160 MB/s; 80 MHz of spectrum at 256 bins and ~1300 spectra/s needs ~375 kB/s, which fits the same USB link. This is what the ESP-WebSDR browser viewer already shows. It is a *display-only* mode: there is no IQ to demodulate.

## What SDR++ is missing

In SDR++ a source module can only deliver **IQ samples**. The waterfall and spectrum are always computed by the core (`IQFrontEnd`) from that IQ. There is no way for a source to hand over a **precomputed spectrum**, so the 80 MHz Turbo view cannot be shown, even though the data is available.

The same limitation blocks an FFT-only mode for SDR++ Server (a server streaming the spectrum instead of full IQ to save bandwidth):

- SDR++ issue [#1356 "Add FFT+VFO Mode to SDR++ Server"](https://github.com/AlexandreRouma/SDRPlusPlus/issues/1356) (open).

## The missing piece (proposed core API)

A small addition to `IQFrontEnd` lets a source take over the spectrum branch. It is implemented and working on branch `feat/server-fft-stream` (SDR++ Server FFT mode, for #1356) and is planned as a pull request to SDR++:

```cpp
// core/src/signal_path/iq_frontend.h
void setExternalFFTInput(bool enabled, int binCount = 0); // stop the internal FFT path, use binCount bins
bool getExternalFFTInput();
int getExternalFFTBinCount();
float* acquireExternalFFTBuffer();   // write binCount dB values, low to high frequency
void releaseExternalFFTBuffer();
```

While enabled, the internal FFT path is stopped and the waterfall takes whatever the source writes into the FFT buffer.

## Implementation in this module

| Mode | Data from the S3 | SDR++ shows | Demodulation |
| --- | --- | --- | --- |
| IQ (demodulation) | `IQS` stream, 250 / 125 / 62.5 kS/s | spectrum + waterfall from IQ | yes |
| Spectrum 16 / 40 / 80 MHz | `SPEC` stream, 256 / 1024 / 2048 bins | on-chip spectrum + waterfall via `setExternalFFTInput` | no (display only) |

How the spectrum mode works:

- `SPECINFO?` gives the supported profiles (rate, rate code, bins, stride, updates per frame). Older firmware without it falls back to the same table as the ESP-WebSDR viewer.
- The S3 is tuned to the centre frequency, the analog filter is opened (`BANDWIDTH 0`) at 40 / 80 MS/s, and `SPEC` is started with mean or max-hold detection.
- The SDR++ input sample rate is set to the span, so the frequency axis and the VFO ruler are right, and `sigpath::iqFrontEnd.setExternalFFTInput(true, bins)` stops the internal FFT path.
- Each `SPC1` frame is CRC-checked and gap-checked; the codes become dBFS (`v / step - 84.3`, 0 = no data), they are reordered from natural FFT order to low-to-high frequency, and copied into `acquireExternalFFTBuffer()` / `releaseExternalFFTBuffer()`.
- Stopping or switching back to IQ ends the stream (`SPECEND`), restores `BANDWIDTH 20` and calls `setExternalFFTInput(false)`.

Tested on hardware with a CW tone from a VSG at centre + 10 MHz: 80 MHz / 256 bins ~50 spectra/s and 40 MHz / 1024 bins, peak exactly at +10.000 MHz, 0 CRC errors, 0 frame gaps.

## Building

CMake option `ESP_SDR_EXTERNAL_FFT`:

- `AUTO` (default): the spectrum mode is built if `core/src/signal_path/iq_frontend.h` declares `setExternalFFTInput`.
- `ON`: require it.
- `OFF`: IQ mode only. Use this for a module that has to load in a stock SDR++ release or nightly: a module built against the extended core does not load in a stock one (the `IQFrontEnd` layout differs).

Until the API is merged upstream, the spectrum mode needs SDR++ built from the branch that has it.

Tracking: [issue #1](https://github.com/z2labs/sdrpp-esp-sdr-source/issues/1).
