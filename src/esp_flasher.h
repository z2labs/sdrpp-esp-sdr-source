// ESP32-S3 firmware flasher: the chip's ROM serial bootloader protocol (the one esptool speaks),
// over the native USB Serial/JTAG port (Android OTG, desktop COM / ttyACM) or a USB-UART bridge.
// No stub loader: plain ROM commands (SYNC, SPI_ATTACH, FLASH_BEGIN/DATA/END, SPI_FLASH_MD5).
// No SDR++ dependency (the CLI test tool uses it too).
#pragma once
#include "esp_sdr_client.h"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace espsdr {

struct FlashImage {
    uint32_t offset = 0;
    std::vector<uint8_t> data;
    std::string name;
};

// Firmware bundled with the app: <dir>/flash_args (esptool format: "0x0 bootloader/bootloader.bin"
// lines after the option line) plus <dir>/version.json ({"build_timestamp": "...", ...}).
struct FirmwareBundle {
    std::vector<FlashImage> images;
    std::string buildTimestamp;   // UTC, ISO 8601: compares as text
    std::string revision;
    std::string buildDate;
    bool load(const std::string& dir, std::string& error);
};

class Flasher {
public:
    // stage: short text for the UI; frac: 0..1 for the whole job
    using Progress = std::function<void(const std::string& stage, float frac)>;
    // The USB Serial/JTAG port re-enumerates when the chip resets into the bootloader (and after
    // flashing): open the port again (Android: wait for the new USB fd). false: give up.
    using Reopen = std::function<bool(SerialPort& port, std::string& error)>;

    // usbJtag: native USB port (reset sequence for the USB Serial/JTAG peripheral, port comes back
    // after the reset); false: USB-UART bridge with the usual EN / IO0 auto-reset transistors.
    bool flash(SerialPort& port, bool usbJtag, const std::vector<FlashImage>& images, Reopen reopen,
               Progress progress, std::string& error);

    // Exposed for tests
    static std::vector<uint8_t> slipEncode(const std::vector<uint8_t>& packet);
    static std::string md5Hex(const uint8_t* data, size_t len);

private:
    bool command(uint8_t op, const std::vector<uint8_t>& data, uint32_t checksum, int timeoutMs,
                 std::vector<uint8_t>* response = nullptr, uint32_t* value = nullptr);
    bool readFrame(std::vector<uint8_t>& frame, int timeoutMs);
    bool sync();
    void resetToBootloader(bool usbJtag);
    void hardReset(bool usbJtag);

    SerialPort* port = nullptr;
    std::vector<uint8_t> rx;    // bytes received but not yet framed
    std::string lastError;
};

}
