// CLI check of the client without SDR++: streams, prints stats, optionally writes cf32.
//   esp_sdr_test <port> [freq_hz] [gain] [rate] [seconds] [out.cf32] [hop]
//   hop = 1: retune every second (alternating +0/+7 MHz) to exercise stream restarts
#include "esp_sdr_client.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: esp_sdr_test <port> [freq_hz] [gain] [rate] [seconds] [out.cf32] [hop]\nports:");
        for (auto& p : espsdr::SerialPort::list()) printf(" %s", p.c_str());
        printf("\n");
        return 1;
    }
    espsdr::Settings s;
    if (argc > 2) s.freqHz = (uint64_t)atof(argv[2]);
    if (argc > 3) s.gain = atoi(argv[3]);
    if (argc > 4) s.rate = atoi(argv[4]);
    double secs = argc > 5 ? atof(argv[5]) : 5;
    FILE* f = (argc > 6 && std::string(argv[6]) != "-") ? fopen(argv[6], "wb") : nullptr;
    bool hop = argc > 7 && atoi(argv[7]);
    espsdr::Client c;
    uint64_t got = 0;
    std::string err;
    if (!c.start(argv[1], s, [&](const float* iq, int n) {
            got += n;
            if (f) fwrite(iq, sizeof(float), 2 * n, f);
        }, err)) {
        printf("ERROR %s\n", err.c_str());
        return 2;
    }
    auto t0 = std::chrono::steady_clock::now();
    uint64_t last = 0; int k = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs && c.running()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint64_t now = c.stats.samples;
        printf("t=%2d s  %7.1f kS/s  frames %llu  crc %llu  gaps %llu  lost %llu  [%s]\n", ++k, (now - last) / 1e3,
               (unsigned long long)c.stats.frames.load(), (unsigned long long)c.stats.crcErrors.load(),
               (unsigned long long)c.stats.gaps.load(), (unsigned long long)c.stats.lost.load(), c.lastTune().c_str());
        fflush(stdout);
        last = now;
        if (hop) { espsdr::Settings h = s; h.freqHz += (k % 2) * 7000000ull; c.update(h); }
    }
    c.stop();
    if (f) fclose(f);
    printf("RESULT samples %llu frames %llu crc %llu gaps %llu lost %llu retunes %llu\n", (unsigned long long)got,
           (unsigned long long)c.stats.frames.load(), (unsigned long long)c.stats.crcErrors.load(),
           (unsigned long long)c.stats.gaps.load(), (unsigned long long)c.stats.lost.load(),
           (unsigned long long)c.stats.retunes.load());
    return 0;
}
