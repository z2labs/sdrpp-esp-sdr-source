# ESP-SDR source for SDR++ (ESP32-S3)

Native [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) source module for an ESP32-S3 running [ESP-SDR](https://github.com/ESPARGOS/esp-sdr): real IQ over the plain USB cable, straight into SDR++, with no bridge, plugin stack or network server in between.

- 250 / 125 / 62.5 kS/s gapless complex IQ (two-stage FIR DDC on the S3's second core)
- 2204–2804 MHz in 1 kHz steps, gain index 0–82, ppm correction
- CRC and sample-index continuity checked on every frame, shown in the source menu

For SDR#, Android or remote use over the network, see [esp-sdr-bridge](https://github.com/z2labs/esp-sdr-bridge) (SpyServer + rtl_tcp).

## Use

1. Flash the ESP32-S3 with current [ESP-SDR](https://github.com/ESPARGOS/esp-sdr) firmware (IQS stream, ESP32-S3 target), e.g. `esp-sdr-s3-iq-stream.bin` from the [esp-sdr-bridge releases](https://github.com/z2labs/esp-sdr-bridge/releases/latest).
2. Copy `esp_sdr_source.dll` into the SDR++ `modules` folder.
3. Start SDR++, open *Module Manager*, add an instance of `esp_sdr_source` (or add `"ESP-SDR Source": {"enabled": true, "module": "esp_sdr_source"}` to `moduleInstances` in `config.json`).
4. Source: **ESP-SDR (ESP32-S3)**, pick the board's native USB port (COMx / `/dev/ttyACM0`), sample rate, press play.

If the board does not answer, it may still be in its bootloader after flashing: press RESET or replug it.

## Wideband 16 / 40 / 80 MHz spectrum (needs an SDR++ core API)

ESP-SDR's Turbo Mode streams **16–80 MHz wide spectra** computed on the chip (as in the ESP-WebSDR browser viewer). The module has a display-only *Spectrum 16 / 40 / 80 MHz* mode (256 / 1024 / 2048 bins, optional max hold) that feeds these spectra straight into the SDR++ waterfall. There is no IQ in this mode, so nothing can be demodulated.

Stock SDR++ cannot take a precomputed spectrum from a source: the core always computes the waterfall from IQ. The same gap blocks an FFT-only SDR++ Server mode, [SDR++ #1356](https://github.com/AlexandreRouma/SDRPlusPlus/issues/1356). The wideband mode therefore needs the small `IQFrontEnd::setExternalFFTInput` API, implemented on branch `feat/server-fft-stream` and planned as an SDR++ pull request. CMake detects it (`-DESP_SDR_EXTERNAL_FFT=AUTO|ON|OFF`); against a stock SDR++ the module builds with the IQ mode only, so it loads in official releases and nightlies. Details: [docs/wideband-spectrum.md](docs/wideband-spectrum.md).

## Build

The module builds either inside the SDR++ tree or out-of-tree against an existing SDR++ build. The module uses SDR++'s C++ API, so build it against the same SDR++ version you run.

**In-tree** (the way it would go upstream): copy this folder to `SDRPlusPlus/source_modules/esp_sdr_source` and add to the SDR++ top-level `CMakeLists.txt`:

```cmake
option(OPT_BUILD_ESP_SDR_SOURCE "Build ESP-SDR (ESP32-S3) Source Module (no dependencies required)" ON)
if (OPT_BUILD_ESP_SDR_SOURCE)
add_subdirectory("source_modules/esp_sdr_source")
endif (OPT_BUILD_ESP_SDR_SOURCE)
```

**Out-of-tree** (Windows example, same toolchain as the SDR++ build):

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
      -DSDRPP_SOURCE_DIR=D:/src/SDRPlusPlus -DSDRPP_BUILD_DIR=D:/src/SDRPlusPlus/build
cmake --build build --config Release
```

No dependencies beyond SDR++ itself; the serial port code is plain Win32 / POSIX.

`esp_sdr_test` (built alongside) streams from the board without SDR++ and prints rate, CRC and gap counters:

```
esp_sdr_test COM4 2350e6 60 250000 10 capture.cf32
esp_sdr_test COM4 2350e6 40 80000000 10 - 0 256     # Turbo spectrum: peak and median per second
```

## How it works

The module tunes the S3 4 MHz (fs/4) below the wanted frequency and starts the `IQS` stream in mode 2: the chip shifts the samples by +fs/4 before its FIR filters, so the LO leakage and the 1/f noise at 0 Hz IF are filtered out instead of sitting in the middle of the spectrum. IQS1 frames (64-bit sample index, CRC32) are checked, scaled to full scale ±1.0, the residual DC is tracked and the spectrum is oriented like any other SDR. Protocol: `docs/iq-stream.md` in ESP-SDR.

## Credits

- [ESP-SDR](https://github.com/ESPARGOS/esp-sdr) by Florian Euchner / ESPARGOS.
- [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) by Alexandre Rouma (Ryzerth); module structure follows its source modules.
- Turbo Mode developed by Zoltan Doczi from https://www.z2labs.io

## License

GPL-3.0, like SDR++. Copyright (c) 2026 Zoltan Doczi.
