// Test tool for the flasher: esp_flash_cli <port> <firmware dir with flash_args> [--usb]
//   --usb: native USB Serial/JTAG port (the port is reopened after the reset), else a USB-UART bridge.
// Build (Windows, MinGW): x86_64-w64-mingw32-g++ -O2 -std=c++17 -static -Isrc tools/esp_flash_cli.cpp
//                         src/esp_flasher.cpp src/esp_sdr_client.cpp -o esp_flash_cli.exe
#include "esp_flasher.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 3) { printf("usage: esp_flash_cli <port> <firmware dir> [--usb]\n"); return 2; }
    std::string portName = argv[1], dir = argv[2];
    bool usb = argc > 3 && !strcmp(argv[3], "--usb");
    espsdr::FirmwareBundle fw;
    std::string err;
    if (!fw.load(dir, err)) { printf("firmware: %s\n", err.c_str()); return 1; }
    printf("firmware %s %s, %zu images\n", fw.buildTimestamp.c_str(), fw.revision.c_str(), fw.images.size());
    for (auto& im : fw.images) printf("  0x%06x %7zu %s md5 %s\n", im.offset, im.data.size(), im.name.c_str(),
                                       espsdr::Flasher::md5Hex(im.data.data(), im.data.size()).c_str());
    espsdr::SerialPort port;
    if (!port.open(portName, err)) { printf("open: %s\n", err.c_str()); return 1; }
    espsdr::Flasher f;
    auto t0 = std::chrono::steady_clock::now();
    bool ok = f.flash(port, usb, fw.images,
        [&](espsdr::SerialPort& p, std::string& e) {
            for (int i = 0; i < 40; i++) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                if (p.open(portName, e)) return true;
            }
            return false;
        },
        [](const std::string& s, float frac) {
            static std::string last;
            if (s != last) { printf("\n%s ", s.c_str()); last = s; }
            printf("\r%-40s %3d%%", s.c_str(), (int)(frac * 100)); fflush(stdout);
        }, err);
    double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("\n%s after %.1f s%s%s\n", ok ? "OK" : "FAILED", secs, ok ? "" : ": ", ok ? "" : err.c_str());
    return ok ? 0 : 1;
}
