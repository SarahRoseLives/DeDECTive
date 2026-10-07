cmake_minimum_required(VERSION 3.16)

project(DeDECTive VERSION 0.3.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_CLI   "Build the CLI frontend" ON)
option(BUILD_GUI   "Build the GUI frontend (requires SDL2 + imgui + OpenGL)" ON)
option(DEDECTIVE_WITH_HACKRF  "Build the HackRF backend if the SDK is found" ON)
option(DEDECTIVE_WITH_SDRPLAY "Build the SDRplay backend if the SDK is found" ON)
option(DEDECTIVE_WITH_PLUTO   "Build the PlutoSDR (libiio) backend if found" ON)

set(THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

find_package(PkgConfig QUIET)

# Toolchain root (e.g. C:/msys64/mingw64) for locating MinGW-provided SDKs.
get_filename_component(_mingw_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
get_filename_component(_mingw_root "${_mingw_bin}" DIRECTORY)

# ── HackRF backend ────────────────────────────────────────────────────────────
set(HAVE_HACKRF OFF)
set(HACKRF_FROM_PKG OFF)
if(DEDECTIVE_WITH_HACKRF)
    find_path(HACKRF_INCLUDE_DIR hackrf.h
        HINTS "${_mingw_root}/include/libhackrf" "${_mingw_root}/include"
              /usr/include/libhackrf /usr/include /usr/local/include/libhackrf)
    find_library(HACKRF_LIBRARY NAMES hackrf
        HINTS "${_mingw_root}/lib" /usr/lib /usr/local/lib)
    if(HACKRF_INCLUDE_DIR AND HACKRF_LIBRARY)
        set(HAVE_HACKRF ON)
    elseif(PkgConfig_FOUND)
        pkg_check_modules(HACKRF QUIET IMPORTED_TARGET libhackrf)
        if(HACKRF_FOUND)
            set(HAVE_HACKRF ON)
            set(HACKRF_FROM_PKG ON)
        endif()
    endif()
    if(HAVE_HACKRF)
        message(STATUS "HackRF: ${HACKRF_INCLUDE_DIR} / ${HACKRF_LIBRARY}")
    else()
        message(STATUS "HackRF SDK not found — building without HackRF support")
    endif()
endif()

# ── SDRplay backend ───────────────────────────────────────────────────────────
set(HAVE_SDRPLAY OFF)
if(DEDECTIVE_WITH_SDRPLAY)
    set(SDRPLAY_ROOT "$ENV{SDRPLAY_ROOT}" CACHE PATH "SDRplay API install directory")
    if(NOT SDRPLAY_ROOT)
        if(WIN32)
            set(SDRPLAY_ROOT "C:/Program Files/SDRplay/API")
        else()
            set(SDRPLAY_ROOT "/usr/local")
        endif()
    endif()

    find_path(SDRPLAY_INCLUDE_DIR sdrplay_api.h
        HINTS "${SDRPLAY_ROOT}/inc" "${SDRPLAY_ROOT}/include")
    find_library(SDRPLAY_LIBRARY NAMES sdrplay_api
        HINTS "${SDRPLAY_ROOT}/x64" "${SDRPLAY_ROOT}/x86" "${SDRPLAY_ROOT}/lib")

    if(SDRPLAY_INCLUDE_DIR AND SDRPLAY_LIBRARY)
        set(HAVE_SDRPLAY ON)
        message(STATUS "SDRplay API: ${SDRPLAY_INCLUDE_DIR} / ${SDRPLAY_LIBRARY}")
    else()
        message(STATUS "SDRplay API not found — building without SDRplay support")
    endif()
endif()

# ── PlutoSDR (libiio) backend ─────────────────────────────────────────────────
set(HAVE_PLUTO OFF)
if(DEDECTIVE_WITH_PLUTO)
    if(PkgConfig_FOUND)
        pkg_check_modules(IIO QUIET IMPORTED_TARGET libiio)
        if(IIO_FOUND)
            set(HAVE_PLUTO ON)
        endif()
    endif()
    if(NOT HAVE_PLUTO)
        get_filename_component(_mingw_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
        get_filename_component(_mingw_root "${_mingw_bin}" DIRECTORY)
        find_path(IIO_INCLUDE_DIR iio.h
            HINTS "${_mingw_root}/include" /usr/include /usr/local/include)
        find_library(IIO_LIBRARY NAMES iio
            HINTS "${_mingw_root}/lib" /usr/lib /usr/local/lib)
        if(IIO_INCLUDE_DIR AND IIO_LIBRARY)
            set(HAVE_PLUTO ON)
        endif()
    endif()
    if(HAVE_PLUTO)
        message(STATUS "PlutoSDR (libiio) backend enabled")
    else()
        message(STATUS "libiio not found — building without PlutoSDR support")
    endif()
endif()

# ── Core library ──────────────────────────────────────────────────────────────
set(CORE_SOURCES
    src/source_factory.cpp
    src/packet_receiver.cpp
    src/packet_decoder.cpp
    src/wideband_monitor.cpp
    src/hopping_scanner.cpp
    src/audio_output.cpp
    src/g72x.c
)
if(HAVE_HACKRF)
    list(APPEND CORE_SOURCES src/hackrf_source.cpp)
endif()
if(HAVE_SDRPLAY)
    list(APPEND CORE_SOURCES src/sdrplay_source.cpp)
endif()
if(HAVE_PLUTO)
    list(APPEND CORE_SOURCES src/pluto_source.cpp)
endif()

add_library(dedective_core ${CORE_SOURCES})

target_include_directories(dedective_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${THIRD_PARTY}/miniaudio"
)

if(HAVE_HACKRF)
    target_compile_definitions(dedective_core PRIVATE DEDECTIVE_HAVE_HACKRF)
    if(HACKRF_FROM_PKG)
        target_link_libraries(dedective_core PUBLIC PkgConfig::HACKRF)
    else()
        target_include_directories(dedective_core PRIVATE "${HACKRF_INCLUDE_DIR}")
        target_link_libraries(dedective_core PUBLIC "${HACKRF_LIBRARY}")
    endif()
endif()

if(HAVE_SDRPLAY)
    target_compile_definitions(dedective_core PRIVATE DEDECTIVE_HAVE_SDRPLAY)
    target_include_directories(dedective_core PRIVATE "${SDRPLAY_INCLUDE_DIR}")
    target_link_libraries(dedective_core PUBLIC "${SDRPLAY_LIBRARY}")
endif()

if(HAVE_PLUTO)
    target_compile_definitions(dedective_core PRIVATE DEDECTIVE_HAVE_PLUTO)
    if(IIO_FOUND)
        target_link_libraries(dedective_core PUBLIC PkgConfig::IIO)
    else()
        target_include_directories(dedective_core PRIVATE "${IIO_INCLUDE_DIR}")
        target_link_libraries(dedective_core PUBLIC "${IIO_LIBRARY}")
    endif()
endif()

# ── Platform audio/system libraries (miniaudio) ───────────────────────────────
if(WIN32)
    target_link_libraries(dedective_core PUBLIC ole32 winmm uuid)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(dedective_core PUBLIC Threads::Threads ${CMAKE_DL_LIBS} m)
endif()

# ── CLI frontend ──────────────────────────────────────────────────────────────
if(BUILD_CLI)
    add_executable(dedective src/main.cpp)
    target_link_libraries(dedective PRIVATE dedective_core)
endif()

# ── GUI frontend ──────────────────────────────────────────────────────────────
if(BUILD_GUI)
    set(GUI_SOURCES src/gui_main.cpp)
    set(GUI_INCLUDE_DIRS)
    set(GUI_LINK_LIBS)
    set(GUI_OK OFF)

    # Linux (pkg-config) imgui backend source directory
    set(IMGUI_BACKENDS_DIR "/usr/share/doc/libimgui-dev/examples/backends"
        CACHE PATH "Path to imgui backend .cpp files")

    if(WIN32 AND EXISTS "${THIRD_PARTY}/SDL2/include/SDL2" AND EXISTS "${THIRD_PARTY}/imgui/imgui.cpp")
        list(APPEND GUI_SOURCES
            "${THIRD_PARTY}/imgui/imgui.cpp"
            "${THIRD_PARTY}/imgui/imgui_draw.cpp"
            "${THIRD_PARTY}/imgui/imgui_tables.cpp"
            "${THIRD_PARTY}/imgui/imgui_widgets.cpp"
            "${THIRD_PARTY}/imgui/backends/imgui_impl_sdl2.cpp"
            "${THIRD_PARTY}/imgui/backends/imgui_impl_opengl3.cpp"
        )
        list(APPEND GUI_INCLUDE_DIRS
            "${THIRD_PARTY}/SDL2/include/SDL2"
            "${THIRD_PARTY}/imgui"
            "${THIRD_PARTY}/imgui/backends"
        )
        list(APPEND GUI_LINK_LIBS
            "${THIRD_PARTY}/SDL2/lib/libSDL2.dll.a"
            opengl32
        )
        set(GUI_OK ON)
    elseif(PkgConfig_FOUND)
        pkg_check_modules(SDL2 QUIET IMPORTED_TARGET sdl2)
        pkg_check_modules(IMGUI QUIET IMPORTED_TARGET imgui)
        if(SDL2_FOUND AND IMGUI_FOUND)
            find_package(OpenGL REQUIRED)
            list(APPEND GUI_SOURCES
                "${IMGUI_BACKENDS_DIR}/imgui_impl_sdl2.cpp"
                "${IMGUI_BACKENDS_DIR}/imgui_impl_opengl3.cpp"
            )
            list(APPEND GUI_INCLUDE_DIRS /usr/include/imgui /usr/include/imgui/backends)
            list(APPEND GUI_LINK_LIBS PkgConfig::IMGUI PkgConfig::SDL2 OpenGL::GL)
            set(GUI_OK ON)
        endif()
    endif()

    if(GUI_OK)
        add_executable(dedective_gui ${GUI_SOURCES})
        target_include_directories(dedective_gui PRIVATE ${GUI_INCLUDE_DIRS})
        target_link_libraries(dedective_gui PRIVATE dedective_core ${GUI_LINK_LIBS})

        if(WIN32)
            # Ship runtime DLLs next to the executable
            add_custom_command(TARGET dedective_gui POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${THIRD_PARTY}/SDL2/bin/SDL2.dll"
                    "$<TARGET_FILE_DIR:dedective_gui>"
                VERBATIM)
        endif()
    else()
        message(STATUS "GUI dependencies not found — not building dedective_gui")
    endif()
endif()

# ── Runtime DLLs (Windows) ────────────────────────────────────────────────────
# Copy every DLL the executables need next to them so the build directory is
# self-contained and does not depend on the MinGW toolchain or SDRplay being on
# PATH.
if(WIN32)
    get_filename_component(_mingw_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)

    set(_runtime_dlls
        "${_mingw_bin}/libgcc_s_seh-1.dll"
        "${_mingw_bin}/libstdc++-6.dll"
        "${_mingw_bin}/libwinpthread-1.dll"
    )

    if(HAVE_SDRPLAY)
        list(APPEND _runtime_dlls "${SDRPLAY_ROOT}/x64/sdrplay_api.dll")
    endif()

    if(HAVE_HACKRF AND WIN32)
        if(EXISTS "${_mingw_bin}/libhackrf.dll")
            list(APPEND _runtime_dlls "${_mingw_bin}/libhackrf.dll")
        endif()
        if(EXISTS "${_mingw_bin}/libusb-1.0.dll")
            list(APPEND _runtime_dlls "${_mingw_bin}/libusb-1.0.dll")
        endif()
    endif()

    if(HAVE_PLUTO)
        foreach(_pluto_dll
                "${_mingw_bin}/libiio.dll"
                "${_mingw_bin}/libxml2-16.dll"
                "${_mingw_bin}/libiconv-2.dll"
                "${_mingw_bin}/zlib1.dll")
            if(EXISTS "${_pluto_dll}")
                list(APPEND _runtime_dlls "${_pluto_dll}")
            endif()
        endforeach()
        # libiio's USB backend needs a 64-bit libusb. (The SDRplay install ships
        # a 32-bit one, so always use the MinGW build.)
        if(EXISTS "${_mingw_bin}/libusb-1.0.dll")
            list(APPEND _runtime_dlls "${_mingw_bin}/libusb-1.0.dll")
        endif()
    endif()

    foreach(_tgt dedective dedective_gui)
        if(TARGET ${_tgt})
            foreach(_dll ${_runtime_dlls})
                if(EXISTS "${_dll}")
                    add_custom_command(TARGET ${_tgt} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            "${_dll}"
                            "$<TARGET_FILE_DIR:${_tgt}>"
                        VERBATIM)
                endif()
            endforeach()
        endif()
    endforeach()
endif()
