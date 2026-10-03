#include "esp_sdr_client.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
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
    dcb.fBinary = TRUE; dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
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
std::vector<std::string> SerialPort::list() {
    std::vector<std::string> r;
    char target[512];
    for (int i = 1; i <= 64; i++) {
        std::string n = "COM" + std::to_string(i);
        if (QueryDosDeviceA(n.c_str(), target, sizeof(target))) r.push_back(n);
    }
    return r;
}
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

bool Client::start(const std::string& name, const Settings& s, SampleCallback callback, std::string& error) {
    stop();
    if (!port.open(name, error)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    port.flushInput();
    // Make sure no stream is still running from an earlier session, then check the firmware
    port.write("\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    port.flushInput();
    if (!command("CAPS", "CAPS", 1500)) { error = "no ESP-SDR firmware answering on " + name; port.close(); return false; }
    cb = callback;
    {
        std::lock_guard<std::mutex> l(mtx);
        want = s; dirty = true;
    }
    streaming = false;
    run = true;
    thr = std::thread(&Client::worker, this);
    return true;
}

void Client::stop() {
    if (run) {
        run = false;
        if (thr.joinable()) thr.join();
    }
    if (port.isOpen()) {
        if (streaming) stopStream();
        port.close();
    }
    streaming = false;
}

void Client::update(const Settings& s) {
    std::lock_guard<std::mutex> l(mtx);
    if (s != want) { want = s; dirty = true; }
}

bool Client::command(const std::string& c, const char* expect, int timeoutMs) {
    port.write(c + "\n");
    std::string line;
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    uint8_t b[256];
    while (std::chrono::steady_clock::now() < end) {
        int n = port.read(b, sizeof(b));
        for (int i = 0; i < n; i++) {
            if (b[i] == '\n') {
                if (line.rfind(expect, 0) == 0 || line.rfind("ERR", 0) == 0) return line.rfind("ERR", 0) != 0;
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
            size_t p = tail.rfind("IQSEND");
            if (p != std::string::npos && tail.find('\n', p) != std::string::npos) break;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    port.flushInput();
    streaming = false;
}

void Client::startStream(const Settings& s) {
    int64_t fk = (int64_t)std::llround(double(s.freqHz) * (1.0 - s.ppm * 1e-6) / 1e3);
    fk = std::max<int64_t>(FMIN / 1000, std::min<int64_t>(FMAX / 1000, fk));
    // LO fs/4 (4 MHz) below the wanted frequency; the chip shifts by +fs/4 before the FIR
    // (IQS mode 2), so LO leakage and the 1/f hump at 0 Hz IF stay outside the output band
    int64_t lo = fk - 4000;
    int mhz = (int)(lo / 1000), khz = (int)(lo % 1000);
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
    streaming = true; fresh = true; haveNext = false;
    cur = s;
    stats.retunes++;
}

void Client::worker() {
    uint8_t b[65536];
    while (run) {
        bool apply = false; Settings s;
        {
            std::lock_guard<std::mutex> l(mtx);
            if (dirty) { s = want; dirty = false; apply = true; }
        }
        if (apply) {
            if (streaming) stopStream();
            startStream(s);
        }
        int n = port.read(b, sizeof(b));
        if (n < 0) { run = false; break; }   // device unplugged
        if (n == 0) continue;
        buf.insert(buf.end(), b, b + n);
        parse();
    }
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
        if (haveNext && sidx != nextIdx) { stats.gaps++; stats.lost += sidx - nextIdx; }
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
        if (cb) cb(out.data(), ns);
    }
    if (pos) buf.erase(buf.begin(), buf.begin() + std::min(pos, buf.size()));
}

} // namespace espsdr
