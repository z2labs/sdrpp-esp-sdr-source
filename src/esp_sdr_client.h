// ESP-SDR (ESP32-S3) IQ stream client: serial port, IQS1 frames, retune handling.
// No SDR++ dependency, so the CLI test tool can use it on its own.
#pragma once
#include <atomic>
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
    bool open(const std::string& name, std::string& error);
    void close();
    bool isOpen() const;
    int read(uint8_t* buf, int len);   // waits at most ~50 ms; <0 on error
    bool write(const std::string& s);
    void flushInput();
    static std::vector<std::string> list();

private:
#ifdef _WIN32
    void* h = nullptr;
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
    bool operator!=(const Settings& o) const {
        return freqHz != o.freqHz || gain != o.gain || rate != o.rate || ppm != o.ppm ||
               specRate != o.specRate || bins != o.bins || maxHold != o.maxHold;
    }
};

struct Stats {
    std::atomic<uint64_t> frames{0}, samples{0}, crcErrors{0}, gaps{0}, lost{0}, retunes{0};
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
    void update(const Settings& s);   // applied by the worker thread (stream restart)
    bool running() const { return run; }
    Stats stats;
    std::string lastTune() { std::lock_guard<std::mutex> l(mtx); return tuneInfo; }

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
    std::mutex mtx;
    Settings want, cur;
    bool dirty = true, streaming = false, fresh = true;
    std::string tuneInfo;
    std::vector<uint8_t> buf;
    std::vector<float> out;
    uint64_t nextIdx = 0; bool haveNext = false;
    double dcI = 0, dcQ = 0;
};

uint32_t crc32(const uint8_t* p, size_t n);

} // namespace espsdr
