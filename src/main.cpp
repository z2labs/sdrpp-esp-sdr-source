// SDR++ source module for ESP-SDR on the ESP32-S3: real IQ over the USB Serial/JTAG port.
// Firmware: ESP-SDR with the IQS stream (ESPARGOS/esp-sdr, ESP32-S3 target).
// Turbo Mode developed by Zoltan Doczi from https://www.z2labs.io
#include "esp_sdr_client.h"
#include <imgui.h>
#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/smgui.h>
#include <gui/style.h>
#include <config.h>
#include <cstring>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

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

class ESPSDRSourceModule : public ModuleManager::Instance {
public:
    ESPSDRSourceModule(std::string name) {
        this->name = name;
        config.acquire();
        if (config.conf.contains("port")) port = config.conf["port"];
        if (config.conf.contains("rateId")) rateId = std::clamp<int>(config.conf["rateId"], 0, 2);
        if (config.conf.contains("gain")) gain = std::clamp<int>(config.conf["gain"], 0, 82);
        if (config.conf.contains("ppm")) ppm = config.conf["ppm"];
        config.release();
        refreshPorts();

        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("ESP-SDR (ESP32-S3)", &handler);
    }

    ~ESPSDRSourceModule() {
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
        return s;
    }

    static void menuSelected(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        core::setInputSampleRate(RATES[_this->rateId]);
        flog::info("ESPSDRSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        flog::info("ESPSDRSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        if (_this->running) { return; }
        _this->stream.clearWriteStop();
        std::string err;
        bool ok = _this->client.start(_this->port, _this->settings(), [_this](const float* iq, int n) {
            memcpy(_this->stream.writeBuf, iq, sizeof(float) * 2 * n);
            _this->stream.swap(n);
        }, err);
        if (!ok) {
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
        _this->running = false;
        flog::info("ESPSDRSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;
        _this->freq = freq;
        if (_this->running) { _this->client.update(_this->settings()); }
        flog::info("ESPSDRSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        ESPSDRSourceModule* _this = (ESPSDRSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_espsdr_port_", _this->name), &_this->portId, _this->portsTxt.c_str())) {
            if (_this->portId < (int)_this->ports.size()) {
                _this->port = _this->ports[_this->portId];
                config.acquire();
                config.conf["port"] = _this->port;
                config.release(true);
            }
        }
        SmGui::FillWidth();
        if (SmGui::Button(CONCAT("Refresh ports##_espsdr_refr_", _this->name))) { _this->refreshPorts(); }

        SmGui::LeftLabel("Samplerate");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_espsdr_sr_", _this->name), &_this->rateId, RATES_TXT)) {
            core::setInputSampleRate(RATES[_this->rateId]);
            config.acquire();
            config.conf["rateId"] = _this->rateId;
            config.release(true);
        }
        if (_this->running) { SmGui::EndDisabled(); }

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
            }
        }
        else {
            SmGui::Text("2204-2804 MHz, 1 kHz steps");
        }
    }

    std::string name;
    bool enabled = true;
    bool running = false;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    espsdr::Client client;
    double freq = 2450e6;
    std::string port;
    std::vector<std::string> ports;
    std::string portsTxt;
    int portId = 0;
    int rateId = 0;
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
