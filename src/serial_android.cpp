// Android serial backend: Android apps cannot open /dev/ttyACM*, so the S3's USB Serial/JTAG
// (a CDC-ACM device, 303A:1001) is driven with libusb on the file descriptor that the app's
// USB permission returned (SDR++: backend::getDeviceFD). Port name: "fd:<n>".
// Reads use 8 queued asynchronous bulk transfers and an event thread, so nothing is lost
// between reads at any stream rate; read() takes bytes from a ring buffer.
#ifdef __ANDROID__
#include "esp_sdr_client.h"
#include <libusb.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace espsdr {

struct SerialPort::Usb {
    libusb_context* ctx = nullptr;
    libusb_device_handle* h = nullptr;
    uint8_t epIn = 0, epOut = 0;
    int ifComm = -1, ifData = -1;
    std::vector<libusb_transfer*> xfers;
    std::vector<std::vector<uint8_t>> bufs;
    std::thread events;
    bool run = false;
    std::mutex m;
    std::condition_variable cv;
    std::vector<uint8_t> ring;        // received bytes not yet read
    size_t head = 0;                  // read position in ring
    int pending = 0;                  // transfers in flight
    bool dead = false;                // device gone / fatal error
    static constexpr size_t RING_MAX = 8u << 20;
};

static void LIBUSB_CALL onIn(libusb_transfer* t) {
    auto* u = (SerialPort::Usb*)t->user_data;
    std::lock_guard<std::mutex> l(u->m);
    if (t->status == LIBUSB_TRANSFER_COMPLETED || t->status == LIBUSB_TRANSFER_TIMED_OUT) {
        if (t->actual_length > 0) {
            if (u->head > (1u << 20)) { u->ring.erase(u->ring.begin(), u->ring.begin() + u->head); u->head = 0; }
            if (u->ring.size() - u->head + t->actual_length <= SerialPort::Usb::RING_MAX)
                u->ring.insert(u->ring.end(), t->buffer, t->buffer + t->actual_length);
            u->cv.notify_all();
        }
        if (u->run && libusb_submit_transfer(t) == 0) return;
    }
    else if (t->status == LIBUSB_TRANSFER_NO_DEVICE || t->status == LIBUSB_TRANSFER_ERROR) {
        u->dead = true; u->cv.notify_all();
    }
    u->pending--;
    u->cv.notify_all();
}

bool SerialPort::open(const std::string& name, std::string& error) {
    close();
    if (name.rfind("fd:", 0) != 0) { error = "no USB device (allow USB access for the ESP32-S3)"; return false; }
    int fd = atoi(name.c_str() + 3);
    // No device enumeration: Android apps may only use the fd they were given
    libusb_set_option(NULL, (libusb_option)2 /* LIBUSB_OPTION_NO_DEVICE_DISCOVERY (WEAK_AUTHORITY) */);
    Usb* u = new Usb();
    if (libusb_init(&u->ctx) < 0) { delete u; error = "libusb_init failed"; return false; }
    if (libusb_wrap_sys_device(u->ctx, (intptr_t)fd, &u->h) < 0 || !u->h) {
        libusb_exit(u->ctx); delete u; error = "cannot open the USB device"; return false;
    }
    // CDC-ACM: communication interface (class 2) for the line state, data interface (class 10) with bulk IN/OUT
    libusb_config_descriptor* cfg = nullptr;
    if (libusb_get_active_config_descriptor(libusb_get_device(u->h), &cfg) == 0 && cfg) {
        for (int i = 0; i < cfg->bNumInterfaces; i++) {
            if (cfg->interface[i].num_altsetting < 1) continue;
            const libusb_interface_descriptor& d = cfg->interface[i].altsetting[0];
            if (d.bInterfaceClass == LIBUSB_CLASS_COMM && u->ifComm < 0) u->ifComm = d.bInterfaceNumber;
            if (d.bInterfaceClass == LIBUSB_CLASS_DATA && u->ifData < 0) {
                uint8_t ei = 0, eo = 0;
                for (int e = 0; e < d.bNumEndpoints; e++) {
                    const libusb_endpoint_descriptor& ep = d.endpoint[e];
                    if ((ep.bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK) continue;
                    if (ep.bEndpointAddress & LIBUSB_ENDPOINT_IN) ei = ep.bEndpointAddress; else eo = ep.bEndpointAddress;
                }
                if (ei && eo) { u->ifData = d.bInterfaceNumber; u->epIn = ei; u->epOut = eo; }
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    if (u->ifData < 0) {
        libusb_close(u->h); libusb_exit(u->ctx); delete u; error = "USB device is not a CDC-ACM serial port"; return false;
    }
    libusb_set_auto_detach_kernel_driver(u->h, 1);
    if (u->ifComm >= 0) libusb_claim_interface(u->h, u->ifComm);
    if (libusb_claim_interface(u->h, u->ifData) < 0) {
        libusb_close(u->h); libusb_exit(u->ctx); delete u; error = "cannot claim the USB interface"; return false;
    }
    // SET_CONTROL_LINE_STATE DTR|RTS in one request, like the desktop backends (no reset into the bootloader)
    if (u->ifComm >= 0) libusb_control_transfer(u->h, 0x21, 0x22, 0x0003, (uint16_t)u->ifComm, nullptr, 0, 500);
    // 8 x 16 kB bulk IN transfers in flight
    u->run = true;
    for (int i = 0; i < 8; i++) {
        u->bufs.emplace_back(16384);
        libusb_transfer* t = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(t, u->h, u->epIn, u->bufs.back().data(), 16384, onIn, u, 0);
        u->xfers.push_back(t);
        if (libusb_submit_transfer(t) == 0) { std::lock_guard<std::mutex> l(u->m); u->pending++; }
    }
    u->events = std::thread([u] {
        while (true) {
            { std::lock_guard<std::mutex> l(u->m); if (!u->run && u->pending <= 0) break; }
            timeval tv{0, 100000};
            libusb_handle_events_timeout_completed(u->ctx, &tv, nullptr);
        }
    });
    usb = u;
    return true;
}

void SerialPort::close() {
    Usb* u = usb;
    if (!u) return;
    usb = nullptr;
    { std::lock_guard<std::mutex> l(u->m); u->run = false; }
    for (libusb_transfer* t : u->xfers) libusb_cancel_transfer(t);
    if (u->events.joinable()) u->events.join();          // returns when every transfer has come back
    for (libusb_transfer* t : u->xfers) libusb_free_transfer(t);
    if (u->ifData >= 0) libusb_release_interface(u->h, u->ifData);
    if (u->ifComm >= 0) libusb_release_interface(u->h, u->ifComm);
    libusb_close(u->h);
    libusb_exit(u->ctx);
    delete u;
}

bool SerialPort::isOpen() const { return usb != nullptr; }

int SerialPort::read(uint8_t* b, int len) {
    Usb* u = usb;
    if (!u) return -1;
    std::unique_lock<std::mutex> l(u->m);
    u->cv.wait_for(l, std::chrono::milliseconds(50), [u] { return u->ring.size() > u->head || u->dead; });
    if (u->ring.size() == u->head) return u->dead ? -1 : 0;
    size_t n = std::min((size_t)len, u->ring.size() - u->head);
    memcpy(b, u->ring.data() + u->head, n);
    u->head += n;
    if (u->head == u->ring.size()) { u->ring.clear(); u->head = 0; }
    return (int)n;
}

bool SerialPort::write(const std::string& s) {
    Usb* u = usb;
    if (!u) return false;
    int n = 0;
    int r = libusb_bulk_transfer(u->h, u->epOut, (unsigned char*)s.data(), (int)s.size(), &n, 1000);
    return r == 0 && n == (int)s.size();
}

void SerialPort::flushInput() {
    Usb* u = usb;
    if (!u) return;
    std::lock_guard<std::mutex> l(u->m);
    u->ring.clear(); u->head = 0;
}

bool SerialPort::setLines(bool dtr, bool rts) {
    Usb* u = usb;
    if (!u || u->ifComm < 0) return false;
    uint16_t v = (uint16_t)((dtr ? 1 : 0) | (rts ? 2 : 0));
    return libusb_control_transfer(u->h, 0x21, 0x22, v, (uint16_t)u->ifComm, nullptr, 0, 500) >= 0;
}

bool SerialPort::setBaud(int) { return true; }   // USB Serial/JTAG: no baud rate

std::vector<std::string> SerialPort::list() { return {"USB"}; }
int SerialPort::usbId(const std::string&, int& pid) { pid = 0; return 0; }

} // namespace espsdr
#endif
