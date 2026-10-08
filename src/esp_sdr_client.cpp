#include "esp_sdr_client.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

static constexpr double kTwoPi = 6.283185307179586;
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <setupapi.h>
#ifdef _MSC_VER
#pragma comment(lib, "setupapi.lib")
#endif
#elif !defined(__ANDROID__)
#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <sys/ioctl.h>
#endif

namespace espsdr {

// ------------------------------------------------------------------ CRC32 (zlib)
uint32_t crc32(const uint8_t* p, size_t n) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        init = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) c = table[(c ^ p[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

// ------------------------------------------------------------------ serial port
SerialPort::~SerialPort() { close(); }

#ifdef _WIN32
bool SerialPort::open(const std::string& name, std::string& error) {
    close();
    std::string path = "\\\\.\\" + name;
    HANDLE hp = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hp == INVALID_HANDLE_VALUE) { error = "cannot open " + name + " (in use?)"; return false; }
    SetupComm(hp, 1 << 20, 1 << 16);
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hp, &dcb);
    dcb.BaudRate = 2000000; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    // DTR/RTS stay deasserted: on the ESP32-S3 USB-JTAG port (and on a UART auto-reset circuit)
    // these lines drive reset / boot mode, and asserting them can reboot the board into the ROM loader.
    dcb.fBinary = TRUE; dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE; dcb.fOutX = FALSE; dcb.fInX = FALSE;
    SetCommState(hp, &dcb);
    COMMTIMEOUTS t{};
    t.ReadIntervalTimeout = MAXDWORD; t.ReadTotalTimeoutMultiplier = MAXDWORD; t.ReadTotalTimeoutConstant = 50;
    t.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(hp, &t);
    h = hp;
    return true;
}
void SerialPort::close() { if (h) { CloseHandle((HANDLE)h); h = nullptr; } }
bool SerialPort::isOpen() const { return h != nullptr; }
int SerialPort::read(uint8_t* b, int len) {
    DWORD n = 0;
    if (!ReadFile((HANDLE)h, b, (DWORD)len, &n, NULL)) return -1;
    return (int)n;
}
bool SerialPort::write(const std::string& s) {
    DWORD n = 0;
    return WriteFile((HANDLE)h, s.data(), (DWORD)s.size(), &n, NULL) && n == s.size();
}
void SerialPort::flushInput() { PurgeComm((HANDLE)h, PURGE_RXCLEAR); }
bool SerialPort::setLines(bool dtr, bool rts) {
    if (!h) return false;
    bool a = EscapeCommFunction((HANDLE)h, dtr ? SETDTR : CLRDTR);
    bool b = EscapeCommFunction((HANDLE)h, rts ? SETRTS : CLRRTS);
    return a && b;
}
bool SerialPort::setBaud(int baud) {
    if (!h) return false;
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)h, &dcb)) return false;
    dcb.BaudRate = (DWORD)baud;
    return SetCommState((HANDLE)h, &dcb);
}
std::vector<std::string> SerialPort::list() {
    std::vector<std::string> r;
    char target[512];
    for (int i = 1; i <= 64; i++) {
        std::string n = "COM" + std::to_string(i);
        if (QueryDosDeviceA(n.c_str(), target, sizeof(target))) r.push_back(n);
    }
    return r;
}
int SerialPort::usbId(const std::string& name, int& pid) {
    // Ports class {4d36e978-e325-11ce-bfc1-08002be10318}: match PortName, read VID_/PID_ from the hardware ID
    static const GUID portsClass = {0x4d36e978, 0xe325, 0x11ce, {0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18}};
    pid = 0;
    HDEVINFO set = SetupDiGetClassDevsA(&portsClass, NULL, NULL, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return 0;
    int vid = 0;
    SP_DEVINFO_DATA d{};
    d.cbSize = sizeof(d);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &d); i++) {
        HKEY k = SetupDiOpenDevRegKey(set, &d, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (k == INVALID_HANDLE_VALUE) continue;
        char pn[64] = {0};
        DWORD sz = sizeof(pn) - 1, type = 0;
        bool match = RegQueryValueExA(k, "PortName", NULL, &type, (LPBYTE)pn, &sz) == ERROR_SUCCESS && _stricmp(pn, name.c_str()) == 0;
        RegCloseKey(k);
        if (!match) continue;
        char hw[512] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(set, &d, SPDRP_HARDWAREID, NULL, (PBYTE)hw, sizeof(hw) - 1, NULL)) {
            std::string h(hw);
            for (auto& c : h) c = (char)toupper((unsigned char)c);
            size_t v = h.find("VID_"), p = h.find("PID_");
            if (v != std::string::npos) vid = (int)strtol(h.substr(v + 4, 4).c_str(), NULL, 16);
            if (p != std::string::npos) pid = (int)strtol(h.substr(p + 4, 4).c_str(), NULL, 16);
        }
        break;
    }
    SetupDiDestroyDeviceInfoList(set);
    return vid;
}
#elif defined(__ANDROID__)
// see serial_android.cpp
#else
bool SerialPort::open(const std::string& name, std::string& error) {
    close();
    int f = ::open(name.c_str(), O_RDWR | O_NOCTTY);
    if (f < 0) { error = "cannot open " + name; return false; }
    termios t{};
    tcgetattr(f, &t);
    cfmakeraw(&t);
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    tcsetattr(f, TCSANOW, &t);
    // The tty layer raises DTR/RTS on open. Drop both in one call (dropping DTR first would leave
    // RTS alone asserted, which resets an ESP32-S3 on its USB-JTAG port).
    int mbits = 0;
    if (ioctl(f, TIOCMGET, &mbits) == 0) { mbits &= ~(TIOCM_DTR | TIOCM_RTS); ioctl(f, TIOCMSET, &mbits); }
    fd = f;
    return true;
}
void SerialPort::close() { if (fd >= 0) { ::close(fd); fd = -1; } }
bool SerialPort::isOpen() const { return fd >= 0; }
int SerialPort::read(uint8_t* b, int len) {
    pollfd p{fd, POLLIN, 0};
    int r = poll(&p, 1, 50);
    if (r < 0) return -1;
    if (r == 0) return 0;
    return (int)::read(fd, b, len);
}
bool SerialPort::write(const std::string& s) { return ::write(fd, s.data(), s.size()) == (ssize_t)s.size(); }
void SerialPort::flushInput() { tcflush(fd, TCIFLUSH); }
bool SerialPort::setLines(bool dtr, bool rts) {
    if (fd < 0) return false;
    int bits = 0;
    if (ioctl(fd, TIOCMGET, &bits) < 0) return false;
    bits = dtr ? (bits | TIOCM_DTR) : (bits & ~TIOCM_DTR);
    bits = rts ? (bits | TIOCM_RTS) : (bits & ~TIOCM_RTS);
    return ioctl(fd, TIOCMSET, &bits) == 0;
}
bool SerialPort::setBaud(int baud) {
    if (fd < 0) return false;
    termios t{};
    if (tcgetattr(fd, &t) < 0) return false;
    speed_t sp = B115200;
    if (baud >= 230400) sp = B230400;
#ifdef B460800
    if (baud >= 460800) sp = B460800;
#endif
#ifdef B921600
    if (baud >= 921600) sp = B921600;
#endif
    cfsetispeed(&t, sp); cfsetospeed(&t, sp);
    return tcsetattr(fd, TCSANOW, &t) == 0;
}
int SerialPort::usbId(const std::string& name, int& pid) {
    pid = 0;
#ifdef __linux__
    // /sys/class/tty/ttyACM0/device -> USB interface; idVendor / idProduct sit one or two levels up
    std::string base = name.substr(name.rfind('/') + 1);
    char real[4096];
    if (!realpath(("/sys/class/tty/" + base + "/device").c_str(), real)) return 0;
    std::string dir = real;
    for (int up = 0; up < 4 && dir.size() > 1; up++) {
        FILE* fv = fopen((dir + "/idVendor").c_str(), "r");
        if (fv) {
            unsigned v = 0, p = 0;
            int ok = fscanf(fv, "%x", &v);
            fclose(fv);
            FILE* fp = fopen((dir + "/idProduct").c_str(), "r");
            if (fp) { if (fscanf(fp, "%x", &p) == 1) pid = (int)p; fclose(fp); }
            return ok == 1 ? (int)v : 0;
        }
        dir = dir.substr(0, dir.rfind('/'));
    }
#else
    (void)name;
#endif
    return 0;
}
std::vector<std::string> SerialPort::list() {
    std::vector<std::string> r;
    glob_t g{};
    for (const char* pat : {"/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*"}) {
        if (glob(pat, 0, NULL, &g) == 0)
            for (size_t i = 0; i < g.gl_pathc; i++) r.push_back(g.gl_pathv[i]);
        globfree(&g);
    }
    return r;
}
#endif

// ------------------------------------------------------------------ client
Client::Client() { buf.reserve(1 << 20); out.reserve(4096); }
Client::~Client() { stop(); }

int Client::outShift(int gain) {
    // S3 gain index -> dB (VSG60 calibration); one bit less shift per 6 dB less gain
    static const int gi[] = {0, 10, 20, 40, 50, 60, 70, 82};
    static const double gd[] = {0.0, 10.3, 21.1, 30.2, 41.9, 51.3, 61.3, 73.3};
    double db = gd[7];
    for (int i = 0; i < 7; i++)
        if (gain <= gi[i + 1]) { db = gd[i] + (gd[i + 1] - gd[i]) * (gain - gi[i]) / double(gi[i + 1] - gi[i]); break; }
    int s = (int)std::lround(4 - (51.3 - db) / 6.02);
    return std::max(0, std::min(4, s));
}

bool Client::start(const std::string& name, const Settings& s, SampleCallback callback, std::string& error,
                   SpectrumCallback scallback) {
    stop();
    if (!port.open(name, error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    port.flushInput();
    // Make sure no stream is still running from an earlier session (an 80 MHz SPEC run keeps
    // sending for a while after the stop byte): stop, then wait for the line to go quiet
    bool capsOk = false;
    for (int attempt = 0; attempt < 3 && !capsOk && !cancel; attempt++) {
        port.write("\n");
        uint8_t junk[16384];
        auto quietSince = std::chrono::steady_clock::now();
        auto giveUp = quietSince + std::chrono::seconds(3);
        while (std::chrono::steady_clock::now() < giveUp && !cancel) {
            int n = port.read(junk, sizeof(junk));
            if (n < 0) break;
            if (n > 0) quietSince = std::chrono::steady_clock::now();
            else if (std::chrono::steady_clock::now() - quietSince > std::chrono::milliseconds(250)) break;
        }
        port.flushInput();
        capsOk = command("CAPS", "CAPS", 1500);
    }
    if (cancel) { error = "connect cancelled"; port.close(); return false; }
    if (!capsOk) { error = "no ESP-SDR firmware answering on " + name; port.close(); return false; }
    std::string caps = reply + " ";
    bool hasSpec = caps.find(" SPEC ") != std::string::npos || caps.find(" SPEC") != std::string::npos;
    hasIqTune = caps.find(" IQTUNE ") != std::string::npos;
    {
        std::string info = "old firmware (no version info)", build;
        if (caps.find(" VERSION ") != std::string::npos && command("VERSION?", "VERSION", 1500)) {
            // VERSION {"revision":"<sha>[-dirty]","build_date":"...","build_timestamp":"...","profile":"..."}
            auto field = [this](const char* key) {
                std::string k = std::string("\"") + key + "\":\"";
                size_t a = reply.find(k);
                if (a == std::string::npos) return std::string();
                a += k.size();
                size_t e = reply.find('"', a);
                return e == std::string::npos ? std::string() : reply.substr(a, e - a);
            };
            std::string rev = field("revision"), date = field("build_date");
            build = field("build_timestamp");
            info = (date.empty() ? std::string("?") : date) + " " + rev.substr(0, std::min<size_t>(7, rev.size()));
            if (rev.find("-dirty") != std::string::npos) info += "+";
        }
        std::lock_guard<std::mutex> l(mtx);
        fwInfo = info + (hasIqTune ? ", smooth tuning" : "");
        fwBuild = build;
    }
    specInfo = (hasSpec && command("SPECINFO?", "SPECINFO", 1500)) ? reply : std::string();
    if (s.specRate && !hasSpec) { error = "this firmware has no SPEC (spectrum) mode"; port.close(); return false; }
    cb = callback;
    scb = scallback;
    {
        std::lock_guard<std::mutex> l(mtx);
        want = s; dirty = true; wantSeq++;
    }
    streaming = false;
    run = true;
    thr = std::thread(&Client::worker, this);
    return true;
}

void Client::stop() {
    run = false;   // the worker may have ended by itself (unplugged): still join it
    if (thr.joinable()) thr.join();
    if (port.isOpen()) {
        if (streaming) stopStream();
        if (cur.specRate) { command("BANDWIDTH 20", "OK", 800); command("DC 1", "", 800); }
        port.close();
    }
    streaming = false;
}

uint64_t Client::update(const Settings& s) {
    std::lock_guard<std::mutex> l(mtx);
    if (s != want) { want = s; dirty = true; wantSeq++; }
    return wantSeq;
}

bool Client::command(const std::string& c, const char* expect, int timeoutMs) {
    port.write(c + "\n");
    std::string line;
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    uint8_t b[256];
    while (std::chrono::steady_clock::now() < end && !cancel) {
        int n = port.read(b, sizeof(b));
        for (int i = 0; i < n; i++) {
            if (b[i] == '\n') {
                if (line.rfind(expect, 0) == 0 || line.rfind("ERR", 0) == 0) { reply = line; return line.rfind("ERR", 0) != 0; }
                line.clear();
            }
            else if (line.size() < 512) line.push_back((char)b[i]);
        }
    }
    return false;
}

void Client::stopStream() {
    // Any host byte ends the run; the firmware answers with an IQSEND line
    port.write("\n");
    std::string tail;
    uint8_t b[65536];
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < end) {
        int n = port.read(b, sizeof(b));
        if (n > 0) {
            tail.append((const char*)b, n);
            if (tail.size() > 8192) tail.erase(0, tail.size() - 8192);
            size_t p = tail.rfind(cur.specRate ? "SPECEND" : "IQSEND");
            if (p != std::string::npos && tail.find('\n', p) != std::string::npos) break;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    port.flushInput();
    streaming = false;
}

// SPECINFO? profiles are [sample_rate_hz, rate_code, fft_bins, stride, units_per_frame(, continuous)].
// Falls back to the strides the ESP-WebSDR viewer uses for the S3 dual-core firmware.
bool Client::specProfile(int rate, int bins, int& code, int& stride, int& upf) {
    for (size_t p = specInfo.find('['); p != std::string::npos; p = specInfo.find('[', p + 1)) {
        long v[6] = {0}; int n = 0; const char* c = specInfo.c_str() + p + 1;
        while (n < 6) {
            char* e; long x = strtol(c, &e, 10);
            if (e == c) break;
            v[n++] = x; c = e;
            while (*c == ',' || *c == ' ') c++;
        }
        if (n >= 5 && v[0] == rate && v[2] == bins) { code = (int)v[1]; stride = (int)v[3]; upf = (int)v[4]; return true; }
    }
    struct P { int rate, bins, code, stride, upf; };
    static const P table[] = {{16000000, 256, 6, 2, 1}, {16000000, 1024, 6, 2, 4}, {16000000, 2048, 6, 3, 7},
                              {40000000, 256, 1, 5, 3}, {40000000, 1024, 1, 5, 9}, {40000000, 2048, 1, 7, 17},
                              {80000000, 256, 0, 10, 5}, {80000000, 1024, 0, 14, 16}, {80000000, 2048, 0, 14, 30}};
    for (const P& t : table)
        if (t.rate == rate && t.bins == bins) { code = t.code; stride = t.stride; upf = t.upf; return true; }
    return false;
}

// LO fs/4 (4 MHz) below the wanted frequency in 1 kHz steps; resid: what the host NCO shifts
void Client::loPlan(const Settings& s, int& mhz, int& khz, double& resid) {
    int64_t fk = (int64_t)std::llround(double(s.freqHz) * (1.0 - s.ppm * 1e-6) / 1e3);
    fk = std::max<int64_t>(FMIN / 1000, std::min<int64_t>(FMAX / 1000, fk));
    int64_t lo = fk - 4000;
    mhz = (int)(lo / 1000); khz = (int)(lo % 1000);
    resid = double(s.freqHz) * (1.0 - s.ppm * 1e-6) - 4e6 - double(lo) * 1e3;
}

void Client::startStream(const Settings& s) {
    int64_t fk = (int64_t)std::llround(double(s.freqHz) * (1.0 - s.ppm * 1e-6) / 1e3);
    fk = std::max<int64_t>(FMIN / 1000, std::min<int64_t>(FMAX / 1000, fk));
    for (size_t a = 0; a < s.extra.size();) {
        size_t e = s.extra.find(';', a);
        if (e == std::string::npos) e = s.extra.size();
        if (e > a) command(s.extra.substr(a, e - a), "", 1500);
        a = e + 1;
    }
    if (s.specRate) {
        // On-chip spectrum: LO at the centre, analog filter open at 40/80 MS/s (its default
        // 20 MHz would only show filtered noise at the edges), ~50 spectra/s to the host
        int code = 0, stride = 1, upf = 1;
        if (!specProfile(s.specRate, s.bins, code, stride, upf)) { code = 6; stride = 2; upf = 1; }
        int unitUs = (int)(12288ll * 1000000 / s.specRate);           // one ring unit
        upf = std::max(upf, std::min(1000, 20000 / std::max(1, unitUs)));
        command("FREQ " + std::to_string(fk / 1000), "OK", 1500);
        command("FOFS " + std::to_string(fk % 1000), "OK", 1500);
        command("GAIN MANUAL " + std::to_string(s.gain), "OK", 1500);
        command(std::string("BANDWIDTH ") + (s.specRate > 16000000 ? "0" : "20"), "OK", 1500);
        // DC 0: the chip removes the DC bin of every FFT (the default averaged estimate leaves the
        // centre 8-20 dB high because the LO leakage changes from FFT to FFT); old firmware: ERR, ignored
        command(s.specDcFix ? "DC 0" : "DC 1", "", 800);
        port.write("SPEC 0 " + std::to_string(stride) + " " + std::to_string(upf) + " " + (s.maxHold ? "1" : "0") + " " +
                   std::to_string(code) + " " + std::to_string(s.bins) + "\n");
        {
            std::lock_guard<std::mutex> l(mtx);
            tuneInfo = "Spectrum " + std::to_string(s.specRate / 1000000) + " MHz, " + std::to_string(s.bins) + " bins, " +
                       (s.maxHold ? "max hold" : "mean");
        }
        specFs = s.specRate; specBins = s.bins;
        buf.clear();
        textTail.clear();
        streaming = true; fresh = true; haveNext = false;
        cur = s;
        stats.retunes++;
        return;
    }
    if (cur.specRate) {   // back to the firmware defaults the web viewer expects
        command("BANDWIDTH 20", "OK", 1500);
        command("DC 1", "", 800);
    }
    // LO fs/4 (4 MHz) below the wanted frequency; the chip shifts by +fs/4 before the FIR
    // (IQS mode 2), so LO leakage and the 1/f hump at 0 Hz IF stay outside the output band
    int64_t lo = fk - 4000;
    int mhz = (int)(lo / 1000), khz = (int)(lo % 1000);
    // The LO can only be set in 1 kHz steps: shift the residual out digitally (signals appear
    // 'resid' Hz too high when the LO sits below the wanted one)
    double resid = double(s.freqHz) * (1.0 - s.ppm * 1e-6) - 4e6 - double(lo) * 1e3;
    ncoStep = std::fabs(resid) < 1000 ? -kTwoPi * resid / s.rate : 0;
    ncoPh = 0;
    int bits = linkBits(s.rate), dec = decimation(s.rate);
    int sh = bits == 8 ? outShift(s.gain) : 0;
    command("FREQ " + std::to_string(mhz), "OK", 1500);
    command("FOFS " + std::to_string(khz), "OK", 1500);
    command("GAIN MANUAL " + std::to_string(s.gain), "OK", 1500);
    port.write("IQS 0 " + std::to_string(dec) + " " + std::to_string(bits) + " 6 " + std::to_string(sh) + " 2\n");
    {
        std::lock_guard<std::mutex> l(mtx);
        tuneInfo = "LO " + std::to_string(mhz) + " MHz + " + std::to_string(khz) + " kHz, " +
                   std::to_string(s.rate / 1000.0).substr(0, 5) + " kS/s, " + std::to_string(bits) + "-bit link";
    }
    buf.clear();
    textTail.clear();
    streaming = true; fresh = true; haveNext = false;
    cur = s;
    stats.retunes++;
}

void Client::worker() {
    uint8_t b[65536];
    // Every retune stops and restarts the IQ stream (any host byte ends a run), which costs a
    // short gap. Dragging the frequency axis sends a new frequency every frame, and applying
    // each one at once kept the stream restarting back to back: no audio at all while tuning.
    // While streaming, apply at most one change per MIN_RETUNE (the latest one wins).
    const auto MIN_RETUNE = std::chrono::milliseconds(350);
    auto lastApply = std::chrono::steady_clock::now() - MIN_RETUNE;
    while (run) {
        bool apply = false; Settings s; uint64_t seq = 0;
        auto now = std::chrono::steady_clock::now();
        if (!streaming || now - lastApply >= (hasIqTune ? std::chrono::milliseconds(100) : MIN_RETUNE)) {
            std::lock_guard<std::mutex> l(mtx);
            if (dirty) { s = want; seq = wantSeq; dirty = false; apply = true; }
        }
        if (apply) {
            Settings f = s; f.freqHz = cur.freqHz;
            if (streaming && hasIqTune && !cur.specRate && !s.specRate && !(f != cur)) {
                // Only the frequency changed and the firmware retunes inside the stream:
                // no restart, no gap (RTL-SDR-like smooth tuning)
                int mhz, khz; double resid;
                loPlan(s, mhz, khz, resid);
                port.write("T " + std::to_string(mhz) + " " + std::to_string(khz) + "\n");
                ncoStep = std::fabs(resid) < 1000 ? -kTwoPi * resid / s.rate : 0;
                cur = s;
                stats.retunes++;
                {
                    std::lock_guard<std::mutex> l(mtx);
                    tuneInfo = "LO " + std::to_string(mhz) + " MHz + " + std::to_string(khz) + " kHz, " +
                               std::to_string(s.rate / 1000.0).substr(0, 5) + " kS/s, smooth";
                }
            }
            else {
                if (streaming) stopStream();
                startStream(s);
            }
            appliedSeq = seq;
            lastApply = std::chrono::steady_clock::now();
        }
        int n = port.read(b, sizeof(b));
        if (n < 0) { run = false; break; }   // device unplugged
        if (n == 0) continue;
        if (streaming && checkRunEnd(b, n)) {
            // The firmware ended the run by itself (ring overrun, e.g. a retune that took too
            // long): start a new one with the current settings instead of going silent
            streaming = false;
            auto t = std::chrono::steady_clock::now();
            if (t - endBurstStart > std::chrono::seconds(5)) { endBurstStart = t; endBurst = 0; }
            if (++endBurst > 5) {
                if (log) log("the stream keeps ending, giving up");
                run = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            port.flushInput();
            startStream(cur);
            continue;
        }
        buf.insert(buf.end(), b, b + n);
        if (cur.specRate) parseSpec(); else parse();
    }
}

// End-of-run report: "<IQSEND|SPECEND> status detail units pairs elapsed_us late_max work_max
// frames drops abandoned ffts stopped_by_host retunes retune_max_cycles"
bool Client::checkRunEnd(const uint8_t* b, int n) {
    textTail.append((const char*)b, n);
    if (textTail.size() > 512) textTail.erase(0, textTail.size() - 512);
    const char* tag = cur.specRate ? "SPECEND " : "IQSEND ";
    size_t p = textTail.rfind(tag);
    if (p == std::string::npos) return false;
    size_t e = textTail.find('\n', p);
    if (e == std::string::npos) return false;
    std::string line = textTail.substr(p, e - p);
    textTail.clear();
    unsigned long v[15] = {0};
    const char* c = line.c_str() + strlen(tag);
    int k = 0;
    for (; k < 15; k++) {
        char* end;
        v[k] = strtoul(c, &end, 10);
        if (end == c) break;
        c = end;
    }
    if (k < 2) return false;   // not a report (binary data that happened to match)
    stats.runEnds++;
    unsigned long retunes = k > 13 ? v[13] : 0;
    if (log) log("firmware ended the stream: " + line);
    if (!cur.specRate && hasIqTune && retunes > 0 && v[0] != 0 && ++tuneFailures >= 2) {
        hasIqTune = false;
        if (log) log("in-stream retune disabled for this session (the stream ended after retunes twice)");
    }
    return true;
}

// SPC1: magic, u32 frame, u64 pair index, u32 pairs, u16 ffts, u8 flags, u8 gain, u16 drops,
// u8 log2(n), u8 dB step, n codes (dB*step of |X|^2 in natural FFT order), u32 CRC.
void Client::parseSpec() {
    size_t pos = 0;
    const uint8_t magic[4] = {'S', 'P', 'C', '1'};
    while (true) {
        auto it = std::search(buf.begin() + pos, buf.end(), magic, magic + 4);
        if (it == buf.end()) { pos = buf.size() > 3 ? buf.size() - 3 : 0; break; }
        size_t i = it - buf.begin();
        if (buf.size() < i + 28) { pos = i; break; }
        const uint8_t* h = buf.data() + i;
        int log2n = h[26];
        if (log2n < 6 || log2n > 12) { pos = i + 4; continue; }
        int n = 1 << log2n;
        size_t L = 28 + (size_t)n + 4;
        if (buf.size() < i + L) { pos = i; break; }
        uint32_t crc; memcpy(&crc, h + L - 4, 4);
        pos = i + L;
        if (crc32(h, L - 4) != crc) { stats.crcErrors++; continue; }
        if (n != specBins) continue;
        uint32_t fr; memcpy(&fr, h + 4, 4);
        if (haveNext && fr != (uint32_t)nextIdx) { stats.gaps++; stats.lost += (uint32_t)(fr - (uint32_t)nextIdx); }
        nextIdx = (uint32_t)(fr + 1); haveNext = true;
        stats.frames++;
        float step = h[27] ? (float)h[27] : 2.0f;
        const uint8_t* c = h + 28;
        spec.resize(n);
        // Same mapping as the ESP-WebSDR viewer: fftshift + mirror (S3: RF above LO is negative),
        // code/step = 10log10|X|^2, -84.3 dB to dBFS (full scale 512, Hann power normalisation)
        for (int j = 0; j < n; j++) {
            uint8_t v = c[(n / 2 - j + n) % n];
            spec[j] = v ? v / step - 84.3f : -140.0f;
        }
        if (cur.specDcFix) {   // DC removed per FFT on the chip: fill the empty centre bin from its neighbours
            double a = std::pow(10.0, spec[n / 2 - 1] / 10.0), b = std::pow(10.0, spec[n / 2 + 1] / 10.0);
            spec[n / 2] = (float)(10.0 * std::log10(0.5 * (a + b)));
        }
        if (scb) scb(spec.data(), n, specFs);
    }
    if (pos) buf.erase(buf.begin(), buf.begin() + std::min(pos, buf.size()));
}

void Client::parse() {
    size_t pos = 0;
    const uint8_t magic[4] = {'I', 'Q', 'S', '1'};
    while (true) {
        auto it = std::search(buf.begin() + pos, buf.end(), magic, magic + 4);
        if (it == buf.end()) { pos = buf.size() > 3 ? buf.size() - 3 : 0; break; }
        size_t i = it - buf.begin();
        if (buf.size() < i + 24) { pos = i; break; }
        const uint8_t* h = buf.data() + i;
        uint32_t fr; uint64_t sidx; uint16_t ns, dec; uint8_t bits, fl, sh;
        memcpy(&fr, h + 4, 4); memcpy(&sidx, h + 8, 8); memcpy(&ns, h + 16, 2);
        bits = h[18]; fl = h[19]; memcpy(&dec, h + 20, 2); sh = h[23];
        (void)fl;
        if ((bits != 8 && bits != 16) || ns == 0 || ns > 1024) { pos = i + 4; continue; }
        size_t L = 24 + (size_t)ns * bits / 4 + 4;
        if (buf.size() < i + L) { pos = i; break; }
        uint32_t crc; memcpy(&crc, h + L - 4, 4);
        pos = i + L;
        if (crc32(h, L - 4) != crc) { stats.crcErrors++; continue; }
        if (fresh) {   // a new run counts frames from 0: skip the old run's tail
            if (fr != 0 || dec != decimation(cur.rate)) continue;
            fresh = false;
        }
        if (haveNext && sidx != nextIdx) {
            stats.gaps++; stats.lost += sidx - nextIdx;
            ncoPh = std::fmod(ncoPh + ncoStep * double(sidx - nextIdx), kTwoPi);   // keep the NCO on the sample clock
        }
        nextIdx = sidx + ns; haveNext = true;
        stats.frames++; stats.samples += ns;
        // FIR units (10-bit sample * 32, int16 full scale) -> +-1.0
        out.resize(2 * ns);
        const float k = float(1 << sh) / 32768.0f;
        double si = 0, sq = 0;
        const uint8_t* p = h + 24;
        for (int j = 0; j < ns; j++) {
            float vi, vq;
            if (bits == 8) { vi = (int8_t)p[2 * j] * k; vq = (int8_t)p[2 * j + 1] * k; }
            else { int16_t a, c; memcpy(&a, p + 4 * j, 2); memcpy(&c, p + 4 * j + 2, 2); vi = a * k; vq = c * k; }
            out[2 * j] = vi; out[2 * j + 1] = vq; si += vi; sq += vq;
        }
        dcI = 0.98 * dcI + 0.02 * si / ns; dcQ = 0.98 * dcQ + 0.02 * sq / ns;   // residual DC
        for (int j = 0; j < ns; j++) {
            out[2 * j] -= (float)dcI;
            out[2 * j + 1] = -(out[2 * j + 1] - (float)dcQ);   // S3: RF above LO is negative -> conjugate
        }
        if (ncoStep != 0) {   // remove the sub-kHz tuning residual (the S3 tunes in 1 kHz steps)
            for (int j = 0; j < ns; j++) {
                float c = (float)std::cos(ncoPh), s = (float)std::sin(ncoPh);
                float a = out[2 * j], b = out[2 * j + 1];
                out[2 * j] = a * c - b * s; out[2 * j + 1] = a * s + b * c;
                ncoPh += ncoStep;
            }
            ncoPh = std::fmod(ncoPh, kTwoPi);
        }
        if (cb) cb(out.data(), ns);
    }
    if (pos) buf.erase(buf.begin(), buf.begin() + std::min(pos, buf.size()));
}

} // namespace espsdr
