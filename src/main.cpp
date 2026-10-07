// SDR++ source module for ESP-SDR on the ESP32-S3: real IQ over the USB Serial/JTAG port.
// Firmware: ESP-SDR with the IQS stream (ESPARGOS/esp-sdr, ESP32-S3 target).
// Turbo Mode developed by Zoltan Doczi from https://www.z2labs.io
#include "esp_sdr_client.h"
#include "esp_flasher.h"
#include <gui/main_window.h>
#include <atomic>
#include <thread>
#include <imgui.h>
#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/smgui.h>
#include <gui/tuner.h>
#include <algorithm>
#include <gui/style.h>
#include <config.h>
#include <cstring>
#ifdef __ANDROID__
#include <android_backend.h>
// ESP32-S3 USB Serial/JTAG; the app asks for USB permission at start-up, the fd comes from the backend
static const std::vector<backend::DevVIDPID> ESP_VIDPIDS = {{0x303A, 0x1001}};
#endif

#define CONCAT(a, b) ((std::string(a) + b).c_str())

// Tuning range. Measured (VSG60 CW sweep 0.15-6 GHz, 2026-10-01): the S3's PLL locks with the
// LO between 2196 and 2806 MHz, with hard edges and no reception outside. In IQ mode the LO sits
// 4 MHz below the wanted frequency and the firmware accepts LO 2200-2800 MHz, hence 2204-2804 MHz.
static const double TUNE_MIN_HZ = 2204e6, TUNE_MAX_HZ = 2804e6;
// RF range with measured front-end response; outside it is shaded on the spectrum
static const double RF_MIN_HZ = 2200e6, RF_MAX_HZ = 2800e6;
// 2.4 GHz ISM band (marked on the spectrum); first start tunes to its centre
static const double ISM_MIN_HZ = 2400e6, ISM_MAX_HZ = 2483.5e6;
static const double DEFAULT_HZ = 2441.75e6;

SDRPP_MOD_INFO{
    /* Name:            */ "esp_sdr_source",
    /* Description:     */ "ESP-SDR (ESP32-S3) source module for SDR++",
    /* Author:          */ "Zoltan Doczi (z2labs)",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

static const int RATES[] = {250000, 125000, 62500};
static const char* RATES_TXT = "250 kHz\0" "125 kHz\0" "62.5 kHz\0";

// Wideband on-chip spectrum needs a core that lets a source supply the FFT
// (IQFrontEnd::setExternalFFTInput, see docs/wideband-spectrum.md). Detected by CMake.
#ifdef ESP_SDR_HAVE_EXTERNAL_FFT
static const int MODES[] = {0, 16000000, 40000000, 80000000};
static const char* MODES_TXT = "IQ (demodulation)\0" "Spectrum 16 MHz\0" "Spectrum 40 MHz\0" "Spectrum 80 MHz\0";
static const int NMODES = 4;
#else
static const int MODES[] = {0};
static const char* MODES_TXT = "IQ (demodulation)\0";
static const int NMODES = 1;
#endif
static const int BINS[] = {256, 1024, 2048};
static const char* BINS_TXT = "256\0" "1024\0" "2048\0";

class ESPSDRSourceModule : public ModuleManager::Instance {
public:
    ESPSDRSourceModule(std::string name) {
        this->name = name;
        config.acquire();
        if (config.conf.contains("port")) port = config.conf["port"];
        if (config.conf.contains("rateId")) rateId = std::clamp<int>(config.conf["rateId"], 0, 2);
        if (config.conf.contains("gain")) gain = std::clamp<int>(config.conf["gain"], 0, 82);
        if (config.conf.contains("ppm")) ppm = config.conf["ppm"];
        if (config.conf.contains("modeId")) modeId = std::clamp<int>(config.conf["modeId"], 0, NMODES - 1);
        if (config.conf.contains("binsId")) binsId = std::clamp<int>(config.conf["binsId"], 0, 2);
        if (config.conf.contains("maxHold")) maxHold = config.conf["maxHold"];
        config.release();
        refreshPorts();

        // Firmware shipped with SDR++ (res/esp_sdr_fw): offered when the dongle has none, an
        // older one, or one that does not answer as ESP-SDR
        std::string fwErr;
        core::configManager.acquire();
        std::string resDir = core::configManager.conf["resourcesDirectory"];
        core::configManager.release();
        haveBundle = bundle.load(resDir + "/esp_sdr_fw", fwErr);
        if (haveBundle) { flog::info("ESP-SDR: bundled firmware {} ({})", bundle.buildTimestamp, bundle.revision); }
        else { flog::warn("ESP-SDR: {}", fwErr); }

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("ESP-SDR (ESP32-S3)", &handler);

        fftRedrawHandler.ctx = this;
        fftRedrawHandler.handler = fftRedraw;
        gui::waterfall.onFFTRedraw.bindHandler(&fftRedrawHandler);
    }

    ~ESPSDRSourceModule() {
        gui::waterfall.onFFTRedraw.unbindHandler(&fftRedrawHandler);
        if (flashThread.joinable()) { flashThread.join(); }
        stop(this);
        sigpath::sourceManager.unregisterSource("ESP-SDR (ESP32-S3)");
    }

    void postInit() {}
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() { return enabled; }

private:
    void refreshPorts() {
        ports = espsdr::SerialPort::list();
        portsTxt.clear();
        portId = 0;
        for (size_t i = 0; i < ports.size(); i++) {
            portsTxt += ports[i];
            portsTxt += '\0';
            if (ports[i] == port) portId = (int)i;
        }
        if (!ports.empty() && port.empty()) port = ports[0];
    }

    espsdr::Settings settings() {
        espsdr::Settings s;
        s.freqHz = freq > 0 ? (uint64_t)freq : 0;
        s.gain = gain;
        s.rate = RATES[rateId];
        s.ppm = ppm;
        s.specRate = MODES[modeId];
        s.bins = BINS[binsId];
        s.maxHold = maxHold;
        return s;
    }

    double inputRate() { return MODES[modeId] ? MODES[modeId] : RATES[rateId]; }

    static void menuSelected(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        _this->selected = true;
        // Coming from another SDR (or a fresh install at the generic default): go back to where
        // this receiver was last used, or to the middle of the 2.4 GHz band
        double cf = gui::waterfall.getCenterFrequency();
        if (cf < TUNE_MIN_HZ || cf > TUNE_MAX_HZ) {
            double last = DEFAULT_HZ;
            config.acquire();
            if (config.conf.contains("lastFreq")) { last = config.conf["lastFreq"]; }
            config.release();
            _this->pendingTuneHz = std::clamp<double>(last, TUNE_MIN_HZ, TUNE_MAX_HZ);
            _this->pendingRestore = true;
        }
        core::setInputSampleRate(_this->inputRate());
        flog::info("ESPSDRSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        _this->selected = false;
        _this->pendingTuneHz = 0;
        _this->pendingRestore = false;
        flog::info("ESPSDRSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        if (_this->running || _this->flashing) { return; }
        _this->stream.clearWriteStop();
        std::string err;
        espsdr::SpectrumCallback scb = nullptr;
#ifdef ESP_SDR_HAVE_EXTERNAL_FFT
        if (MODES[_this->modeId]) {
            // Display-only: the chip's spectrum goes straight into the waterfall
            core::setInputSampleRate(MODES[_this->modeId]);
            sigpath::iqFrontEnd.setExternalFFTInput(true, BINS[_this->binsId]);
            scb = [](const float* db, int bins, int) {
                if (bins != sigpath::iqFrontEnd.getExternalFFTBinCount()) { sigpath::iqFrontEnd.setExternalFFTInput(true, bins); }
                float* fft = sigpath::iqFrontEnd.acquireExternalFFTBuffer();
                if (fft) { memcpy(fft, db, sizeof(float) * bins); }
                sigpath::iqFrontEnd.releaseExternalFFTBuffer();
            };
        }
#endif
        std::string openName = _this->port;
#ifdef __ANDROID__
        {
            int vid = 0, pid = 0;
            int fd = backend::getDeviceFD(vid, pid, ESP_VIDPIDS);
            if (fd < 0) {
#ifdef ESP_SDR_HAVE_EXTERNAL_FFT
                sigpath::iqFrontEnd.setExternalFFTInput(false);
#endif
                _this->error = "connect the ESP32-S3 via USB OTG and allow USB access";
                flog::error("ESP-SDR: {}", _this->error);
                return;
            }
            openName = "fd:" + std::to_string(fd);
        }
#endif
        bool ok = _this->client.start(openName, _this->settings(), [_this](const float* iq, int n) {
            memcpy(_this->stream.writeBuf, iq, sizeof(float) * 2 * n);
            _this->stream.swap(n);
        }, err, scb);
        if (!ok) {
#ifdef ESP_SDR_HAVE_EXTERNAL_FFT
            sigpath::iqFrontEnd.setExternalFFTInput(false);
#endif
            _this->error = err;
            flog::error("ESP-SDR: {}", err);
            return;
        }
        _this->error.clear();
        _this->running = true;
        flog::info("ESPSDRSourceModule '{0}': Start on {1}", _this->name, _this->port);
    }

    static void stop(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        if (!_this->running) { return; }
        _this->stream.stopWriter();
        _this->client.stop();
        _this->stream.clearWriteStop();
#ifdef ESP_SDR_HAVE_EXTERNAL_FFT
        sigpath::iqFrontEnd.setExternalFFTInput(false);   // hand the spectrum back to the core
#endif
        _this->running = false;
        flog::info("ESPSDRSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        // Outside the PLL lock range the chip cannot receive at all: stay at the edge, and put the
        // GUI (frequency display, scale) back on the frequency that is really tuned (next frame)
        double f = std::clamp<double>(freq, TUNE_MIN_HZ, TUNE_MAX_HZ);
        if (f != freq && _this->pendingRestore) {
            // Start right after selecting this source still carries the previous SDR's frequency:
            // use the restored one, no warning
            f = _this->pendingTuneHz;
        }
        else if (f != freq) {
            _this->pendingTuneHz = f;
            _this->limitNoticeUntil = ImGui::GetTime() + 4.0;
            flog::warn("ESP-SDR: {0:.0f} Hz is outside the tuning range, using {1:.0f} Hz", freq, f);
        }
        freq = f;
        config.acquire();
        config.conf["lastFreq"] = freq;
        config.release(true);
        _this->freq = freq;
        if (_this->running) { _this->client.update(_this->settings()); }
        flog::info("ESPSDRSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;

        // Nothing greys out while running: a change here restarts the stream
        bool restart = false;
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_espsdr_port_", _this->name), &_this->portId, _this->portsTxt.c_str())) {
            if (_this->portId < (int)_this->ports.size()) {
                _this->port = _this->ports[_this->portId];
                config.acquire();
                config.conf["port"] = _this->port;
                config.release(true);
                restart = true;
            }
        }
        SmGui::FillWidth();
        if (SmGui::Button(CONCAT("Refresh ports##_espsdr_refr_", _this->name))) { _this->refreshPorts(); }

        if (NMODES > 1) {
            SmGui::LeftLabel("Mode");
            SmGui::FillWidth();
            if (SmGui::Combo(CONCAT("##_espsdr_mode_", _this->name), &_this->modeId, MODES_TXT)) {
                core::setInputSampleRate(_this->inputRate());
                config.acquire();
                config.conf["modeId"] = _this->modeId;
                config.release(true);
                restart = true;
            }
        }
        if (MODES[_this->modeId] == 0) {
            SmGui::LeftLabel("Samplerate");
            SmGui::FillWidth();
            if (SmGui::Combo(CONCAT("##_espsdr_sr_", _this->name), &_this->rateId, RATES_TXT)) {
                core::setInputSampleRate(RATES[_this->rateId]);
                config.acquire();
                config.conf["rateId"] = _this->rateId;
                config.release(true);
                restart = true;
            }
        }
        else {
            SmGui::LeftLabel("FFT bins");
            SmGui::FillWidth();
            if (SmGui::Combo(CONCAT("##_espsdr_bins_", _this->name), &_this->binsId, BINS_TXT)) {
                config.acquire();
                config.conf["binsId"] = _this->binsId;
                config.release(true);
                restart = true;
            }
            if (SmGui::Checkbox(CONCAT("Max hold##_espsdr_mh_", _this->name), &_this->maxHold)) {
                config.acquire();
                config.conf["maxHold"] = _this->maxHold;
                config.release(true);
                restart = true;
            }
        }
        if (restart && _this->running) {
            stop(_this);
            start(_this);
        }

        SmGui::LeftLabel("Gain");
        SmGui::FillWidth();
        if (SmGui::SliderInt(CONCAT("##_espsdr_gain_", _this->name), &_this->gain, 0, 82)) {
            if (_this->running) { _this->client.update(_this->settings()); }
            config.acquire();
            config.conf["gain"] = _this->gain;
            config.release(true);
        }

        SmGui::LeftLabel("PPM");
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_espsdr_ppm_", _this->name), &_this->ppm, -10.0f, 10.0f, 0.01f, SmGui::FMT_STR_FLOAT_TWO_DECIMAL)) {
            if (_this->running) { _this->client.update(_this->settings()); }
            config.acquire();
            config.conf["ppm"] = _this->ppm;
            config.release(true);
        }

        if (!_this->error.empty()) {
            SmGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), _this->error.c_str());
        }
        else if (_this->running) {
            if (!_this->client.running()) {
                SmGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Device lost - press stop");
            }
            else {
                char buf[256];
                auto& st = _this->client.stats;
                snprintf(buf, sizeof(buf), "frames %llu  crc %llu  gaps %llu",
                         (unsigned long long)st.frames.load(), (unsigned long long)st.crcErrors.load(),
                         (unsigned long long)st.gaps.load());
                SmGui::Text(buf);
                SmGui::Text(_this->client.lastTune().c_str());
                std::string fw = _this->client.firmwareInfo();
                if (!fw.empty()) { _this->lastFirmware = fw; }
            }
        }
        else {
            SmGui::Text("Tunes 2204-2804 MHz (PLL lock range), 1 kHz steps");
        }
        // Firmware update / install
        _this->drawFirmwareUpdate();

        // Dongle firmware (known after the first start)
        if (!_this->lastFirmware.empty()) {
            std::string fwText = "Firmware: " + _this->lastFirmware;
            if (_this->lastFirmware.rfind("old", 0) == 0) {
                SmGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), fwText.c_str());
            }
            else {
                SmGui::Text(fwText.c_str());
            }
        }
    }

    // Spectrum overlay (GUI thread): applies a pending re-tune, shades the frequencies outside the
    // receiver's measured range, marks the 2.4 GHz ISM band and shows the out-of-range notice.
    static void fftRedraw(ImGui::WaterFall::FFTRedrawArgs args, void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        if (!_this->selected) { return; }
        if (_this->pendingTuneHz > 0) {
            double f = _this->pendingTuneHz;
            _this->pendingTuneHz = 0;
            _this->pendingRestore = false;
            tuner::centerTuning(gui::waterfall.selectedVFO, f);
            core::configManager.acquire();
            core::configManager.conf["frequency"] = f;
            core::configManager.release(true);
        }

        ImDrawList* dl = args.window->DrawList;
        auto toX = [&args](double hz) { return (float)(args.min.x + (hz - args.lowFreq) * args.freqToPixelRatio); };
        float s = style::uiScale;
        if (args.lowFreq < RF_MIN_HZ) {
            float x = std::min<float>(toX(RF_MIN_HZ), args.max.x);
            dl->AddRectFilled(args.min, ImVec2(x, args.max.y), IM_COL32(140, 0, 0, 60));
            if (x < args.max.x) { dl->AddLine(ImVec2(x, args.min.y), ImVec2(x, args.max.y), IM_COL32(255, 60, 60, 200), 2.0f * s); }
        }
        if (args.highFreq > RF_MAX_HZ) {
            float x = std::max<float>(toX(RF_MAX_HZ), args.min.x);
            dl->AddRectFilled(ImVec2(x, args.min.y), args.max, IM_COL32(140, 0, 0, 60));
            if (x > args.min.x) { dl->AddLine(ImVec2(x, args.min.y), ImVec2(x, args.max.y), IM_COL32(255, 60, 60, 200), 2.0f * s); }
        }
        const double ism[2] = { ISM_MIN_HZ, ISM_MAX_HZ };
        for (double e : ism) {
            if (e <= args.lowFreq || e >= args.highFreq) { continue; }
            float x = toX(e);
            dl->AddLine(ImVec2(x, args.min.y), ImVec2(x, args.max.y), IM_COL32(255, 200, 0, 110), 1.0f * s);
        }
        if (ISM_MAX_HZ > args.lowFreq && ISM_MIN_HZ < args.highFreq) {
            float x0 = std::max<float>(toX(ISM_MIN_HZ), args.min.x), x1 = std::min<float>(toX(ISM_MAX_HZ), args.max.x);
            const char* lbl = "2.4 GHz ISM";
            ImVec2 ts = ImGui::CalcTextSize(lbl);
            if (x1 - x0 > ts.x + 8.0f * s) {
                dl->AddText(ImVec2((x0 + x1 - ts.x) / 2.0f, args.max.y - ts.y - 2.0f * s), IM_COL32(255, 200, 0, 170), lbl); // bottom: the band plan uses the top
            }
        }
        if (ImGui::GetTime() < _this->limitNoticeUntil) {
            const char* msg = "ESP32-S3 tunes 2204 - 2804 MHz only";
            ImVec2 ts = ImGui::CalcTextSize(msg);
            ImVec2 p((args.min.x + args.max.x - ts.x) / 2.0f, args.min.y + (args.max.y - args.min.y) / 2.0f - ts.y / 2.0f);
            dl->AddRectFilled(ImVec2(p.x - 10.0f * s, p.y - 6.0f * s), ImVec2(p.x + ts.x + 10.0f * s, p.y + ts.y + 6.0f * s), IM_COL32(120, 0, 0, 220), 6.0f * s);
            dl->AddText(p, IM_COL32(255, 255, 255, 255), msg);
        }
    }

    std::string name;
    bool enabled = true;
    bool running = false;
    std::string lastFirmware;   // dongle firmware seen on the last start (shown in the menu)
    bool selected = false;
    double pendingTuneHz = 0;
    bool pendingRestore = false;
    double limitNoticeUntil = 0;
    EventHandler<ImGui::WaterFall::FFTRedrawArgs> fftRedrawHandler;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    espsdr::Client client;

    // ---- firmware update (ROM bootloader over the same USB port) ----
    espsdr::FirmwareBundle bundle;
    bool haveBundle = false;
    std::atomic<bool> flashing{false};
    std::thread flashThread;
    std::mutex flashMtx;
    std::string flashStage, flashResult;
    float flashFrac = 0.0f;
    double confirmUntil = 0.0;

    // Does the dongle need the bundled firmware? (known after a start attempt)
    bool firmwareOutdated() {
        if (!haveBundle) { return false; }
        if (error.find("no ESP-SDR firmware") != std::string::npos) { return true; }
        if (lastFirmware.rfind("old", 0) == 0) { return true; }
        std::string b = client.firmwareBuild();
        return !b.empty() && !bundle.buildTimestamp.empty() && b < bundle.buildTimestamp;
    }

    void startFlash() {
        if (flashing) { return; }
        if (flashThread.joinable()) { flashThread.join(); }
        if (running) { gui::mainWindow.setPlayState(false); }
        flashing = true;
        gui::mainWindow.usbAutoStartPaused = true;
        { std::lock_guard<std::mutex> l(flashMtx); flashStage = "Starting"; flashFrac = 0; flashResult.clear(); }
        flashThread = std::thread([this] {
            std::string err, portName = port;
            espsdr::SerialPort sp;
            bool ok = false;
#ifdef __ANDROID__
            int vid = 0, pid = 0;
            int fd = backend::getDeviceFD(vid, pid, ESP_VIDPIDS);
            if (fd < 0) { err = "connect the ESP32-S3 (USB / JTAG port) first"; }
            portName = "fd:" + std::to_string(fd);
            auto reopen = [fd](espsdr::SerialPort& p, std::string& e) {
                // The board re-enumerates in its ROM bootloader: wait for Android to report it gone,
                // then for the new device (the user may have to allow USB access again).
                // Without a detach within 4 s the reset did not re-enumerate: try the same fd.
                bool gone = false;
                for (int i = 0; i < 300; i++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    int v = 0, pi = 0;
                    int nfd = backend::getDeviceFD(v, pi, ESP_VIDPIDS);
                    if (nfd < 0 && !gone) { gone = true; flog::info("ESP-SDR flash: board left the bus ({:.1f} s)", i * 0.1); }
                    if (nfd >= 0 && (gone || nfd != fd)) {
                        flog::info("ESP-SDR flash: board is back as fd {} after {:.1f} s", nfd, i * 0.1);
                        std::this_thread::sleep_for(std::chrono::milliseconds(300));
                        return p.open("fd:" + std::to_string(nfd), e);
                    }
                    if (!gone && i == 40) {
                        flog::warn("ESP-SDR flash: no re-enumeration after the reset, trying the same fd {}", fd);
                        return p.open("fd:" + std::to_string(fd), e);
                    }
                }
                e = std::string(gone ? "the board left the bus but did not come back (allow USB access if Android asks)"
                                     : "the board did not reset into its bootloader");
                return false;
            };
#else
            auto reopen = [portName](espsdr::SerialPort& p, std::string& e) {
                for (int i = 0; i < 40; i++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                    if (p.open(portName, e)) { return true; }
                }
                return false;
            };
#endif
            if (err.empty() && sp.open(portName, err)) {
                espsdr::Flasher f;
                flog::info("ESP-SDR flash: start on {} ({} images)", portName, bundle.images.size());
                ok = f.flash(sp, true, bundle.images, reopen, [this](const std::string& s, float frac) {
                    std::lock_guard<std::mutex> l(flashMtx);
                    if (s != flashStage) { flog::info("ESP-SDR flash: {} ({:.0f} %)", s, frac * 100.0f); }
                    flashStage = s; flashFrac = frac;
                }, err);
            }
            sp.close();
            {
                std::lock_guard<std::mutex> l(flashMtx);
                flashResult = ok ? "Firmware " + bundle.buildDate + " installed. The board restarts and SDR++ starts it."
                                 : "Firmware update failed: " + err;
            }
            flog::info("ESP-SDR: {}", ok ? "firmware installed" : "firmware update failed: " + err);
            if (ok) { error.clear(); lastFirmware.clear(); }
            gui::mainWindow.usbAutoStartPaused = false;
            flashing = false;
        });
    }

    void drawFirmwareUpdate() {
        if (!haveBundle) { return; }
        if (flashing) {
            std::lock_guard<std::mutex> l(flashMtx);
            ImGui::TextUnformatted(flashStage.c_str());
            ImGui::ProgressBar(flashFrac, ImVec2(-FLT_MIN, 0));
            ImGui::TextDisabled("Keep the board connected. Allow USB access if Android asks.");
            return;
        }
        {
            std::lock_guard<std::mutex> l(flashMtx);
            if (!flashResult.empty()) {
                bool ok = flashResult.rfind("Firmware update failed", 0) != 0;
                ImGui::PushTextWrapPos(0.0f);
                ImGui::TextColored(ok ? ImVec4(0.4f, 0.9f, 0.4f, 1.0f) : ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", flashResult.c_str());
                ImGui::PopTextWrapPos();
            }
        }
        bool outdated = firmwareOutdated();
        double now = ImGui::GetTime();
        std::string label = (outdated ? "Install firmware " : "Reinstall firmware ") + bundle.buildDate;
        if (now < confirmUntil) { label = "Tap again to flash the board"; }
        if (outdated) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.45f, 0.05f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.55f, 0.10f, 1.0f));
        }
        if (ImGui::Button((label + "##_espsdr_flash").c_str(), ImVec2(-FLT_MIN, 0))) {
            if (now < confirmUntil) { confirmUntil = 0; startFlash(); }
            else { confirmUntil = now + 5.0; }
        }
        if (outdated) {
            ImGui::PopStyleColor(2);
            ImGui::TextDisabled("This board needs the ESP-SDR firmware that comes with SDR++.");
        }
    }
    double freq = DEFAULT_HZ;
    std::string port;
    std::vector<std::string> ports;
    std::string portsTxt;
    int portId = 0;
    int rateId = 0;
    int modeId = 0;
    int binsId = 0;
    bool maxHold = false;
    int gain = 60;
    float ppm = 0.0f;
    std::string error;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/esp_sdr_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new ESPSDRSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (ESPSDRSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
