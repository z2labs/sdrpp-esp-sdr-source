// ESP-SDR (ESP32-S3) IQ stream client: serial port, IQS1 frames, retune handling.
// No SDR++ dependency, so the CLI test tool can use it on its own.
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace espsdr {

// Minimal blocking serial port (Win32 / POSIX). The S3 uses its USB Serial/JTAG
// port, so the baud rate is irrelevant; DTR and RTS are asserted like pyserial
// does, which does not reset the chip.
class SerialPort {
public:
    ~SerialPort();
    bool open(const std::string& name, std::string& error);   // Android: name = "fd:<n>" (USB device fd)
    void close();
    bool isOpen() const;
    int read(uint8_t* buf, int len);   // waits at most ~50 ms; <0 on error
    bool write(const std::string& s);
    void flushInput();
    // Modem lines (CDC SET_CONTROL_LINE_STATE on Android): the firmware flasher uses them to
    // reset the chip into its ROM bootloader. setBaud: UART bridges only (USB-JTAG ignores it).
    bool setLines(bool dtr, bool rts);
    bool setBaud(int baud);
    static std::vector<std::string> list();
#ifdef __ANDROID__
    // Android has no /dev/ttyACM for apps: CDC-ACM over libusb on the fd the USB permission gave us
    struct Usb;
#endif

private:
#if defined(_WIN32)
    void* h = nullptr;
#elif defined(__ANDROID__)
    Usb* usb = nullptr;
#else
    int fd = -1;
#endif
};

struct Settings {
    uint64_t freqHz = 2450000000ull; // wanted centre frequency
    int gain = 60;                   // S3 gain index 0..82
    int rate = 250000;               // 250000, 125000 or 62500
    double ppm = 0.0;                // crystal correction, positive = board reads high
    int specRate = 0;                // 0: IQ stream; 16/40/80e6: on-chip spectrum (SPEC, display only)
    int bins = 256;                  // SPEC FFT size: 256, 1024, 2048
    bool maxHold = false;            // SPEC detector: mean (false) or max hold
    bool specDcFix = true;           // SPEC: per-FFT DC removal on the chip (DC 0) + centre bin filled
    std::string extra;               // diagnostics: ';'-separated commands sent before each stream start
    bool operator!=(const Settings& o) const {
        return freqHz != o.freqHz || gain != o.gain || rate != o.rate || ppm != o.ppm ||
               specRate != o.specRate || bins != o.bins || maxHold != o.maxHold || specDcFix != o.specDcFix ||
               extra != o.extra;
    }
};

struct Stats {
    std::atomic<uint64_t> frames{0}, samples{0}, crcErrors{0}, gaps{0}, lost{0}, retunes{0};
    std::atomic<uint64_t> runEnds{0};   // the firmware ended a stream on its own (restarted)
};

// Samples are delivered as interleaved float I/Q, full scale +-1.0 (int16 FIR units / 32768),
// residual DC removed and the spectrum oriented like any other SDR (RF above LO = positive).
using SampleCallback = std::function<void(const float* iq, int count)>;
// One spectrum in dBFS, low to high frequency, span = sample rate, centred on the tuned frequency.
using SpectrumCallback = std::function<void(const float* db, int bins, int sampleRate)>;

class Client {
public:
    Client();
    ~Client();
    bool start(const std::string& port, const Settings& s, SampleCallback cb, std::string& error,
               SpectrumCallback scb = nullptr);
    void stop();
    uint64_t update(const Settings& s);   // applied by the worker thread (stream restart); returns a sequence number
    bool applied(uint64_t seq) const { return appliedSeq >= seq; }   // stream restarted with those settings
    bool running() const { return run; }
    // A start() running on another thread gives up soon (true), or may run normally again (false)
    void cancelStart(bool c) { cancel = c; }
    // Diagnostics (worker thread): unexpected stream ends, fallbacks
    std::function<void(const std::string&)> log;
    Stats stats;
    std::string lastTune() { std::lock_guard<std::mutex> l(mtx); return tuneInfo; }
    // Firmware identity from VERSION? ("2026-10-07 8fdf468"), or a note for firmware
    // without it; empty until connected. smoothTuning(): firmware retunes inside the stream.
    std::string firmwareInfo() { std::lock_guard<std::mutex> l(mtx); return fwInfo; }
    std::string firmwareBuild() { std::lock_guard<std::mutex> l(mtx); return fwBuild; }
    bool smoothTuning() const { return hasIqTune; }

    static constexpr uint64_t FMIN = 2204000000ull, FMAX = 2804000000ull;
    static int linkBits(int rate) { return rate >= 250000 ? 8 : 16; }
    static int decimation(int rate) { return 16000000 / rate; }
    static int outShift(int gain);    // 8-bit link: keep the noise floor a few LSB above 0

private:
    void worker();
    bool command(const std::string& c, const char* expect, int timeoutMs);
    void stopStream();
    void startStream(const Settings& s);
    void parse();
    void parseSpec();
    bool specProfile(int rate, int bins, int& code, int& stride, int& upf);

    SerialPort port;
    SampleCallback cb;
    SpectrumCallback scb;
    std::string specInfo;            // SPECINFO? reply (profiles), empty if not supported
    std::string reply;               // last line matched by command()
    std::vector<float> spec;
    int specFs = 0, specBins = 0;
    std::thread thr;
    std::atomic<bool> run{false};
    std::atomic<bool> cancel{false};
    std::mutex mtx;
    Settings want, cur;
    bool dirty = true, streaming = false, fresh = true;
    uint64_t wantSeq = 0;
    std::atomic<uint64_t> appliedSeq{0};
    std::string tuneInfo;
    std::string fwInfo, fwBuild;     // fwBuild: UTC build timestamp from VERSION?, "" if unknown
    std::atomic<bool> hasIqTune{false};
    std::string textTail;            // last raw bytes, to spot the firmware's end-of-run report
    int tuneFailures = 0;            // stream ends right after an in-stream retune
    int endBurst = 0;                // stream ends in a row (restart loop guard)
    std::chrono::steady_clock::time_point endBurstStart{};
    bool checkRunEnd(const uint8_t* b, int n);
    void loPlan(const Settings& s, int& mhz, int& khz, double& resid);
    std::vector<uint8_t> buf;
    std::vector<float> out;
    uint64_t nextIdx = 0; bool haveNext = false;
    double dcI = 0, dcQ = 0;
    double ncoPh = 0, ncoStep = 0;   // fine tuning below the S3's 1 kHz LO step
};

uint32_t crc32(const uint8_t* p, size_t n);

} // namespace espsdr
