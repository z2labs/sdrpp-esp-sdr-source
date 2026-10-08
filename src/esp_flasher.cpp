#include "esp_flasher.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

namespace espsdr {

// ------------------------------------------------------------------ MD5 (RFC 1321)
namespace {
struct Md5 {
    uint32_t a = 0x67452301, b = 0xefcdab89, c = 0x98badcfe, d = 0x10325476;
    static uint32_t rol(uint32_t x, int s) { return (x << s) | (x >> (32 - s)); }
    void block(const uint8_t* p) {
        static const uint32_t K[64] = {
            0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
            0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
            0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
            0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
            0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
            0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
            0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
            0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
        static const int S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                                  5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20,
                                  4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                                  6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
        uint32_t M[16];
        for (int i = 0; i < 16; i++) M[i] = p[i * 4] | (p[i * 4 + 1] << 8) | (p[i * 4 + 2] << 16) | ((uint32_t)p[i * 4 + 3] << 24);
        uint32_t A = a, B = b, C = c, D = d;
        for (int i = 0; i < 64; i++) {
            uint32_t F; int g;
            if (i < 16) { F = (B & C) | (~B & D); g = i; }
            else if (i < 32) { F = (D & B) | (~D & C); g = (5 * i + 1) % 16; }
            else if (i < 48) { F = B ^ C ^ D; g = (3 * i + 5) % 16; }
            else { F = C ^ (B | ~D); g = (7 * i) % 16; }
            F = F + A + K[i] + M[g];
            A = D; D = C; C = B;
            B = B + rol(F, S[i]);
        }
        a += A; b += B; c += C; d += D;
    }
};
}

std::string Flasher::md5Hex(const uint8_t* data, size_t len) {
    Md5 m;
    size_t i = 0;
    for (; i + 64 <= len; i += 64) m.block(data + i);
    uint8_t tail[128] = {0};
    size_t rest = len - i;
    memcpy(tail, data + i, rest);
    tail[rest] = 0x80;
    size_t tl = rest + 1 + 8 <= 64 ? 64 : 128;
    uint64_t bits = (uint64_t)len * 8;
    for (int k = 0; k < 8; k++) tail[tl - 8 + k] = (uint8_t)(bits >> (8 * k));
    m.block(tail);
    if (tl == 128) m.block(tail + 64);
    char out[33];
    uint32_t w[4] = {m.a, m.b, m.c, m.d};
    for (int k = 0; k < 16; k++) snprintf(out + 2 * k, 3, "%02x", (w[k / 4] >> (8 * (k % 4))) & 0xFF);
    return std::string(out, 32);
}

// ------------------------------------------------------------------ firmware bundle
static bool readFile(const std::string& path, std::vector<uint8_t>& out) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return false;
    out.clear();
    uint8_t b[16384];
    size_t n;
    while ((n = fread(b, 1, sizeof(b), f)) > 0) out.insert(out.end(), b, b + n);
    fclose(f);
    return true;
}

static std::string jsonField(const std::string& s, const char* key) {
    std::string k = std::string("\"") + key + "\"";
    size_t a = s.find(k);
    if (a == std::string::npos) return "";
    a = s.find('"', s.find(':', a + k.size()) + 1);
    if (a == std::string::npos) return "";
    size_t e = s.find('"', a + 1);
    return e == std::string::npos ? "" : s.substr(a + 1, e - a - 1);
}

bool FirmwareBundle::load(const std::string& dir, std::string& error) {
    images.clear();
    std::vector<uint8_t> args;
    if (!readFile(dir + "/flash_args", args)) { error = "no bundled firmware (" + dir + "/flash_args)"; return false; }
    // Whitespace-separated tokens: options (--flash_mode dio ...), then "<0xoffset> <file>" pairs
    std::vector<std::string> tok;
    std::string cur;
    for (uint8_t c : args) {
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t') { if (!cur.empty()) { tok.push_back(cur); cur.clear(); } }
        else cur.push_back((char)c);
    }
    if (!cur.empty()) tok.push_back(cur);
    for (size_t i = 0; i + 1 < tok.size(); i++) {
        if (tok[i].rfind("0x", 0) != 0) continue;
        FlashImage im;
        im.offset = (uint32_t)strtoul(tok[i].c_str(), nullptr, 16);
        im.name = tok[i + 1];
        if (!readFile(dir + "/" + im.name, im.data) || im.data.empty()) { error = "missing " + im.name; return false; }
        images.push_back(std::move(im));
        i++;
    }
    if (images.empty()) { error = "empty flash_args"; return false; }
    std::vector<uint8_t> v;
    if (readFile(dir + "/version.json", v)) {
        std::string s(v.begin(), v.end());
        buildTimestamp = jsonField(s, "build_timestamp");
        buildDate = jsonField(s, "build_date");
        revision = jsonField(s, "revision");
    }
    return true;
}

// ------------------------------------------------------------------ SLIP / commands
std::vector<uint8_t> Flasher::slipEncode(const std::vector<uint8_t>& p) {
    std::vector<uint8_t> o;
    o.reserve(p.size() + 16);
    o.push_back(0xC0);
    for (uint8_t b : p) {
        if (b == 0xC0) { o.push_back(0xDB); o.push_back(0xDC); }
        else if (b == 0xDB) { o.push_back(0xDB); o.push_back(0xDD); }
        else o.push_back(b);
    }
    o.push_back(0xC0);
    return o;
}

bool Flasher::readFrame(std::vector<uint8_t>& frame, int timeoutMs) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (true) {
        // A complete frame already buffered?
        size_t s = 0;
        while (s < rx.size() && rx[s] != 0xC0) s++;
        if (s < rx.size()) {
            size_t e = s + 1;
            while (e < rx.size() && rx[e] != 0xC0) e++;
            if (e < rx.size()) {
                if (e == s + 1) { rx.erase(rx.begin(), rx.begin() + s + 1); continue; }   // C0 C0: frame boundary
                frame.clear();
                for (size_t i = s + 1; i < e; i++) {
                    if (rx[i] == 0xDB && i + 1 < e) { frame.push_back(rx[i + 1] == 0xDC ? 0xC0 : 0xDB); i++; }
                    else frame.push_back(rx[i]);
                }
                rx.erase(rx.begin(), rx.begin() + e + 1);
                return true;
            }
        }
        else rx.clear();   // no frame start: drop the noise (boot log text etc.)
        if (std::chrono::steady_clock::now() > end) return false;
        uint8_t b[4096];
        int n = port->read(b, sizeof(b));
        if (n < 0) { lastError = "USB read failed"; return false; }
        rx.insert(rx.end(), b, b + n);
    }
}

static void put32(std::vector<uint8_t>& v, uint32_t x) {
    for (int i = 0; i < 4; i++) v.push_back((uint8_t)(x >> (8 * i)));
}

bool Flasher::command(uint8_t op, const std::vector<uint8_t>& data, uint32_t checksum, int timeoutMs,
                      std::vector<uint8_t>* response, uint32_t* value) {
    std::vector<uint8_t> pkt = {0x00, op, (uint8_t)(data.size() & 0xFF), (uint8_t)(data.size() >> 8)};
    put32(pkt, checksum);
    pkt.insert(pkt.end(), data.begin(), data.end());
    std::vector<uint8_t> enc = slipEncode(pkt);
    if (!port->write(std::string(enc.begin(), enc.end()))) { lastError = "USB write failed"; return false; }
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    std::vector<uint8_t> f;
    while (true) {
        int left = (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - std::chrono::steady_clock::now()).count();
        if (left <= 0 || !readFrame(f, left)) {
            if (lastError.empty()) {
                char b[64]; snprintf(b, sizeof(b), "no answer to command 0x%02x", op);
                lastError = b;
            }
            return false;
        }
        if (f.size() < 8 || f[0] != 0x01 || f[1] != op) continue;   // e.g. late SYNC replies
        size_t len = f[2] | (f[3] << 8);
        if (f.size() < 8 + len) continue;
        std::vector<uint8_t> d(f.begin() + 8, f.begin() + 8 + len);
        // ROM loader: data ends with 4 status bytes (status, error, 0, 0)
        if (d.size() >= 2) {
            size_t st = d.size() >= 4 ? d.size() - 4 : d.size() - 2;
            if (d[st] != 0) {
                char b[80]; snprintf(b, sizeof(b), "command 0x%02x failed (status %u, error 0x%02x)", op, d[st], d[st + 1]);
                lastError = b;
                return false;
            }
            d.resize(st);
        }
        if (value) *value = f[4] | (f[5] << 8) | (f[6] << 16) | ((uint32_t)f[7] << 24);
        if (response) *response = d;
        return true;
    }
}

bool Flasher::sync() {
    std::vector<uint8_t> d = {0x07, 0x07, 0x12, 0x20};
    d.insert(d.end(), 32, 0x55);
    for (int attempt = 0; attempt < 12; attempt++) {
        lastError.clear();
        if (command(0x08, d, 0, 120)) {
            // The ROM answers a SYNC several times: let the rest arrive, then drop it
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            uint8_t b[4096];
            while (port->read(b, sizeof(b)) > 0) {}
            rx.clear();
            return true;
        }
    }
    lastError = "the chip did not enter its bootloader (no SYNC answer)";
    return false;
}

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void Flasher::resetToBootloader(bool usbJtag) {
    if (usbJtag) {
        // esptool's USB-JTAG-Serial reset: IO0 low via DTR, chip reset via RTS
        port->setLines(false, false); sleepMs(100);
        port->setLines(true, false);  sleepMs(100);
        port->setLines(true, true);   // pass through (1,1), not (0,0)
        port->setLines(false, true);  sleepMs(100);
        port->setLines(false, false);
    }
    else {
        // Classic DevKit auto-reset (DTR -> IO0, RTS -> EN through transistors)
        port->setLines(false, true);  sleepMs(100);   // EN low (reset), IO0 high
        port->setLines(true, false);  sleepMs(50);    // EN high, IO0 low: boot into the ROM loader
        port->setLines(false, false);
    }
}

bool Flasher::writeReg(uint32_t addr, uint32_t value, uint32_t mask, int timeoutMs) {
    std::vector<uint8_t> d;
    put32(d, addr); put32(d, value); put32(d, mask); put32(d, 0);
    return command(0x09, d, 0, timeoutMs);
}

void Flasher::hardReset(bool usbJtag) {
    if (usbJtag) {
        // Entering the ROM loader over USB Serial/JTAG latches "force download boot": an RTS
        // reset would land in the loader again. Clear it first, as esptool does for the S3
        // (RTC_CNTL_OPTION1_REG bit 0), then reset with IO0 (DTR) released.
        writeReg(0x6000812C, 0, 0x1, 500);
        port->setLines(false, true); sleepMs(200);
        port->setLines(false, false); sleepMs(200);
        return;
    }
    port->setLines(false, true); sleepMs(100);   // RTS: reset with IO0 high -> runs the new firmware
    port->setLines(false, false);
}

bool Flasher::flash(SerialPort& p, bool usbJtag, const std::vector<FlashImage>& images, Reopen reopen,
                    Progress progress, std::string& error) {
    port = &p;
    rx.clear();
    lastError.clear();
    auto fail = [&](const std::string& what) { error = what + (lastError.empty() ? "" : ": " + lastError); return false; };
    auto prog = [&](const std::string& s, float f) { if (progress) progress(s, f); };

    prog("Resetting into the bootloader", 0.0f);
    if (!usbJtag) p.setBaud(115200);
    resetToBootloader(usbJtag);
    if (usbJtag) {
        // The USB device disappears and comes back as the ROM's own USB Serial/JTAG
        p.close();
        sleepMs(600);
        if (!reopen || !reopen(p, error)) { if (error.empty()) error = "the board did not come back after the reset"; return false; }
    }
    rx.clear();
    prog("Connecting to the bootloader", 0.02f);
    if (!sync()) return fail("bootloader");

    // Flash access: SPI_ATTACH (default pins) and the flash geometry the images were built for
    std::vector<uint8_t> d;
    put32(d, 0); put32(d, 0);
    if (!command(0x0D, d, 0, 3000)) return fail("SPI attach");
    uint32_t total = 0;
    for (auto& im : images) total += (uint32_t)im.data.size();
    d.clear();
    put32(d, 0); put32(d, 4u << 20); put32(d, 64u << 10); put32(d, 4u << 10); put32(d, 256); put32(d, 0xFFFF);
    if (!command(0x0B, d, 0, 3000)) return fail("SPI flash parameters");

    const uint32_t BLOCK = 0x400;   // ROM loader write size
    uint32_t done = 0;
    for (auto& im : images) {
        uint32_t size = (uint32_t)im.data.size();
        uint32_t blocks = (size + BLOCK - 1) / BLOCK;
        prog("Erasing " + im.name, 0.05f + 0.85f * done / std::max<uint32_t>(1, total));
        d.clear();
        put32(d, size); put32(d, blocks); put32(d, BLOCK); put32(d, im.offset);
        put32(d, 0);   // not encrypted (ESP32-S3 ROM takes this fifth word)
        int eraseMs = 10000 + (int)(size / 1024) * 60;
        if (!command(0x02, d, 0, eraseMs)) return fail("erase at " + std::to_string(im.offset));
        for (uint32_t seq = 0; seq < blocks; seq++) {
            std::vector<uint8_t> blk(im.data.begin() + seq * BLOCK,
                                     im.data.begin() + std::min<uint32_t>(size, (seq + 1) * BLOCK));
            blk.resize(BLOCK, 0xFF);
            uint32_t ck = 0xEF;
            for (uint8_t b : blk) ck ^= b;
            d.clear();
            put32(d, BLOCK); put32(d, seq); put32(d, 0); put32(d, 0);
            d.insert(d.end(), blk.begin(), blk.end());
            bool ok = false;
            for (int retry = 0; retry < 3 && !ok; retry++) { lastError.clear(); ok = command(0x03, d, ck, 3000); }
            if (!ok) return fail("write " + im.name + " block " + std::to_string(seq));
            uint32_t written = std::min<uint32_t>(size, (seq + 1) * BLOCK);
            prog("Writing " + im.name, 0.05f + 0.85f * (done + written) / std::max<uint32_t>(1, total));
        }
        done += size;

        // Read back the MD5 of what is in flash now
        prog("Verifying " + im.name, 0.05f + 0.85f * done / std::max<uint32_t>(1, total));
        d.clear();
        put32(d, im.offset); put32(d, size); put32(d, 0); put32(d, 0);
        std::vector<uint8_t> r;
        if (!command(0x13, d, 0, 8000 + (int)(size / 1024) * 10, &r)) return fail("verify " + im.name);
        std::string got;
        if (r.size() >= 32) got.assign(r.begin(), r.begin() + 32);
        else for (uint8_t b : r) { char x[3]; snprintf(x, 3, "%02x", b); got += x; }   // stub-style 16 raw bytes
        std::string want = md5Hex(im.data.data(), im.data.size());
        for (auto& ch : got) ch = (char)tolower(ch);
        if (got != want) { lastError.clear(); return fail("verify " + im.name + " (MD5 mismatch)"); }
    }

    prog("Starting the new firmware", 0.95f);
    d.clear();
    put32(d, 1);   // FLASH_END, stay in the loader; the hard reset below starts the firmware
    command(0x04, d, 0, 2000);
    hardReset(usbJtag);
    if (usbJtag) p.close();
    prog("Done", 1.0f);
    return true;
}

}
