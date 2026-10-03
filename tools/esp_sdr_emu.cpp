// esp_sdr_emu: scripted stand-in for SDR++ when testing the client. It makes the same calls the
// SDR++ module makes (start / stop / tune / sample rate or mode change via Client::update) and
// is driven line by line on stdin, one reply line per command on stdout:
//   start | stop | set key=value ... | sync [timeout_s] | cap N file [timeout_s] | stats | quit
//   sync: wait until the stream has restarted with the last 'set' (replies SYNC <seconds>)
//   keys: freq gain rate ppm spec bins maxhold extra   (spec=0: IQ; spec=16e6/40e6/80e6: SPEC)
//   cap: next N complex samples (IQ, cf32) or N spectra (SPEC, float32 dBFS x bins) to file;
//        cap N file timeout 1 also writes file.ts: {double steady-clock s, int64 count} per callback
//        (steady_clock = QueryPerformanceCounter on Windows, same time base as Python's perf_counter)
#include "esp_sdr_client.h"
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

struct Capture {
    std::mutex m;
    std::condition_variable cv;
    FILE* f = nullptr;
    FILE* ts = nullptr;     // optional: <file>.ts, per callback {double steady_s, int64 count after it}
    long long left = 0, done = 0;
    int bins = 0;
    void stamp() {
        if (!ts) return;
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        long long d = done;
        fwrite(&t, sizeof(t), 1, ts); fwrite(&d, sizeof(d), 1, ts);
    }
};

int main(int argc, char** argv) {
    if (argc < 2) { printf("usage: esp_sdr_emu <port>   (commands on stdin)\n"); return 1; }
    std::string port = argv[1];
    espsdr::Settings s;
    espsdr::Client c;
    Capture cap;
    uint64_t seq = 0;
    auto iq = [&](const float* x, int n) {
        std::lock_guard<std::mutex> l(cap.m);
        if (!cap.f || cap.left <= 0) return;
        long long k = n < cap.left ? n : cap.left;
        fwrite(x, sizeof(float), (size_t)(2 * k), cap.f);
        cap.left -= k; cap.done += k; cap.stamp();
        if (!cap.left) cap.cv.notify_all();
    };
    auto spec = [&](const float* db, int n, int) {
        std::lock_guard<std::mutex> l(cap.m);
        if (!cap.f || cap.left <= 0) return;
        fwrite(db, sizeof(float), (size_t)n, cap.f);
        cap.bins = n; cap.left--; cap.done++; cap.stamp();
        if (!cap.left) cap.cv.notify_all();
    };
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        in >> cmd;
        auto t0 = std::chrono::steady_clock::now();
        if (cmd == "start") {
            std::string err;
            if (c.start(port, s, iq, err, spec)) { seq = c.update(s); printf("OK\n"); } else printf("ERR %s\n", err.c_str());
        }
        else if (cmd == "stop") { c.stop(); printf("OK\n"); }
        else if (cmd == "set") {
            std::string kv;
            while (in >> kv) {
                size_t e = kv.find('=');
                if (e == std::string::npos) continue;
                std::string k = kv.substr(0, e), v = kv.substr(e + 1);
                if (k == "freq") s.freqHz = (uint64_t)atof(v.c_str());
                else if (k == "gain") s.gain = atoi(v.c_str());
                else if (k == "rate") s.rate = atoi(v.c_str());
                else if (k == "ppm") s.ppm = atof(v.c_str());
                else if (k == "spec") s.specRate = (int)atof(v.c_str());
                else if (k == "bins") s.bins = atoi(v.c_str());
                else if (k == "maxhold") s.maxHold = atoi(v.c_str()) != 0;
                else if (k == "dcfix") s.specDcFix = atoi(v.c_str()) != 0;
                else if (k == "extra") { s.extra = v; for (char& ch : s.extra) if (ch == '_') ch = ' '; }
            }
            if (c.running()) seq = c.update(s);
            printf("OK %llu\n", (unsigned long long)seq);
        }
        else if (cmd == "sync") {   // wait until the stream runs with the last 'set'
            double timeout = 5; in >> timeout;
            while (c.running() && !c.applied(seq) &&
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            printf("%s %.4f\n", c.applied(seq) ? "SYNC" : "TIMEOUT", dt);
        }
        else if (cmd == "cap") {
            long long n = 0; std::string file; double timeout = 10; int stamps = 0;
            in >> n >> file >> timeout >> stamps;
            FILE* f = fopen(file.c_str(), "wb");
            if (!f) { printf("ERR cannot open %s\n", file.c_str()); fflush(stdout); continue; }
            FILE* ts = stamps ? fopen((file + ".ts").c_str(), "wb") : nullptr;
            std::unique_lock<std::mutex> l(cap.m);
            cap.f = f; cap.ts = ts; cap.left = n; cap.done = 0;
            cap.stamp();   // t of the request, count 0
            bool ok = cap.cv.wait_for(l, std::chrono::duration<double>(timeout), [&] { return cap.left == 0; });
            cap.f = nullptr; cap.ts = nullptr;
            fclose(f);
            if (ts) fclose(ts);
            double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            printf("%s %lld %.3f %d\n", ok ? "CAP" : "TIMEOUT", cap.done, dt, cap.bins);
        }
        else if (cmd == "stats") {
            printf("STATS frames=%llu samples=%llu crc=%llu gaps=%llu lost=%llu retunes=%llu running=%d tune=%s\n",
                   (unsigned long long)c.stats.frames.load(), (unsigned long long)c.stats.samples.load(),
                   (unsigned long long)c.stats.crcErrors.load(), (unsigned long long)c.stats.gaps.load(),
                   (unsigned long long)c.stats.lost.load(), (unsigned long long)c.stats.retunes.load(),
                   c.running() ? 1 : 0, c.lastTune().c_str());
        }
        else if (cmd == "quit") break;
        else if (!cmd.empty()) printf("ERR unknown %s\n", cmd.c_str());
        fflush(stdout);
    }
    c.stop();
    return 0;
}
