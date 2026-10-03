cmake_minimum_required(VERSION 3.13)
project(esp_sdr_source)

# Two ways to build:
#  1. In-tree: copy/clone this folder to SDRPlusPlus/source_modules/esp_sdr_source and add
#       option(OPT_BUILD_ESP_SDR_SOURCE "Build ESP-SDR (ESP32-S3) Source Module (no dependencies required)" ON)
#       if (OPT_BUILD_ESP_SDR_SOURCE) add_subdirectory("source_modules/esp_sdr_source") endif ()
#     to the SDR++ top-level CMakeLists.txt.
#  2. Out-of-tree against an existing SDR++ build:
#       cmake -S . -B build -DSDRPP_SOURCE_DIR=<SDRPlusPlus> -DSDRPP_BUILD_DIR=<SDRPlusPlus/build> [vcpkg toolchain]

file(GLOB SRC "src/*.cpp")

if (TARGET sdrpp_core)
    include(${SDRPP_MODULE_CMAKE})
else ()
    if (NOT SDRPP_SOURCE_DIR OR NOT SDRPP_BUILD_DIR)
        message(FATAL_ERROR "Out-of-tree build: set SDRPP_SOURCE_DIR and SDRPP_BUILD_DIR")
    endif ()
    set(CMAKE_CXX_STANDARD 17)
    add_library(sdrpp_core SHARED IMPORTED)
    if (MSVC)
        set_target_properties(sdrpp_core PROPERTIES
            IMPORTED_IMPLIB "${SDRPP_BUILD_DIR}/core/Release/sdrpp_core.lib"
            IMPORTED_LOCATION "${SDRPP_BUILD_DIR}/core/Release/sdrpp_core.dll")
    else ()
        find_library(SDRPP_CORE_LIB sdrpp_core PATHS "${SDRPP_BUILD_DIR}/core" NO_DEFAULT_PATH REQUIRED)
        set_target_properties(sdrpp_core PROPERTIES IMPORTED_LOCATION "${SDRPP_CORE_LIB}")
    endif ()
    set(CORE "${SDRPP_SOURCE_DIR}/core")
    set_target_properties(sdrpp_core PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
        "${CORE}/src;${CORE}/src/imgui;${CORE}/backends/glfw;${CORE}/backends/glfw/imgui")
    if (MSVC)
        set(POTHOS "C:/Program Files/PothosSDR")
        set_property(TARGET sdrpp_core APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${POTHOS}/include")
        set_property(TARGET sdrpp_core APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${POTHOS}/lib/volk.lib")
        if (DEFINED VCPKG_INSTALLED_DIR)
            set_property(TARGET sdrpp_core APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${VCPKG_INSTALLED_DIR}/x64-windows/include")
        endif ()
        set(SDRPP_MODULE_COMPILER_FLAGS /O2 /Ob2 /std:c++17 /EHsc)
    else ()
        find_package(PkgConfig)
        pkg_check_modules(VOLK REQUIRED volk)
        set_property(TARGET sdrpp_core APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${VOLK_INCLUDE_DIRS}")
        set_property(TARGET sdrpp_core APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${VOLK_LIBRARIES}")
        set(SDRPP_MODULE_COMPILER_FLAGS -O3 -std=c++17)
    endif ()
    set(SDRPP_CORE_ROOT "${CORE}")
    include("${SDRPP_SOURCE_DIR}/sdrpp_module.cmake")
endif ()

target_include_directories(esp_sdr_source PRIVATE "src/")

# Android: no /dev/ttyACM for apps, the serial port is CDC-ACM over libusb on the USB fd
# (src/serial_android.cpp); libusb comes from the android-sdr-kit used by the SDR++ Android build
if (ANDROID)
    target_include_directories(esp_sdr_source PRIVATE /sdr-kit/${ANDROID_ABI}/include/libusb-1.0 /sdr-kit/${ANDROID_ABI}/include)
    target_link_libraries(esp_sdr_source PRIVATE /sdr-kit/${ANDROID_ABI}/lib/libusb1.0.so)
endif ()

# Wideband (on-chip spectrum) mode needs IQFrontEnd::setExternalFFTInput in the SDR++ core
# (docs/wideband-spectrum.md). AUTO: use it if the core headers have it. Build with OFF for
# SDR++ releases without it, otherwise the module would not load there.
set(ESP_SDR_EXTERNAL_FFT "AUTO" CACHE STRING "Wideband spectrum mode: AUTO, ON or OFF")
if (TARGET sdrpp_core AND NOT SDRPP_SOURCE_DIR)
    set(_core_root "${CMAKE_SOURCE_DIR}/core")
else ()
    set(_core_root "${SDRPP_SOURCE_DIR}/core")
endif ()
set(_have_fft OFF)
if (ESP_SDR_EXTERNAL_FFT STREQUAL "ON")
    set(_have_fft ON)
elseif (ESP_SDR_EXTERNAL_FFT STREQUAL "AUTO" AND EXISTS "${_core_root}/src/signal_path/iq_frontend.h")
    file(READ "${_core_root}/src/signal_path/iq_frontend.h" _iqfe)
    string(FIND "${_iqfe}" "setExternalFFTInput" _pos)
    if (NOT _pos EQUAL -1)
        set(_have_fft ON)
    endif ()
endif ()
if (_have_fft)
    target_compile_definitions(esp_sdr_source PRIVATE ESP_SDR_HAVE_EXTERNAL_FFT)
endif ()
message(STATUS "esp_sdr_source: wideband spectrum mode ${_have_fft}")

# Stand-alone CLI check of the device client (no SDR++ needed at run time)
option(ESP_SDR_BUILD_TEST "Build the esp_sdr_test command-line tool" ON)
if (ESP_SDR_BUILD_TEST AND NOT ANDROID)
    add_executable(esp_sdr_test tools/esp_sdr_test.cpp src/esp_sdr_client.cpp)
    target_include_directories(esp_sdr_test PRIVATE "src/")
    if (NOT MSVC)
        target_compile_options(esp_sdr_test PRIVATE -O2 -std=c++17)
        find_package(Threads REQUIRED)
        target_link_libraries(esp_sdr_test PRIVATE Threads::Threads)
    endif ()
    # Scripted SDR++ stand-in for hardware tests (tools/hwtest.py)
    add_executable(esp_sdr_emu tools/esp_sdr_emu.cpp src/esp_sdr_client.cpp)
    target_include_directories(esp_sdr_emu PRIVATE "src/")
    if (NOT MSVC)
        target_compile_options(esp_sdr_emu PRIVATE -O2 -std=c++17)
        target_link_libraries(esp_sdr_emu PRIVATE Threads::Threads)
    endif ()
endif ()
