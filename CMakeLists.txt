cmake_minimum_required(VERSION 3.16)

project(DeDECTive VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)

pkg_check_modules(HACKRF REQUIRED IMPORTED_TARGET libhackrf)
pkg_check_modules(IIO REQUIRED IMPORTED_TARGET libiio)
pkg_check_modules(PULSE REQUIRED IMPORTED_TARGET libpulse-simple)

option(BUILD_GUI "Build the GUI frontend (requires imgui, SDL2, OpenGL)" ON)

if(BUILD_GUI)
    find_package(OpenGL REQUIRED)
    pkg_check_modules(IMGUI REQUIRED IMPORTED_TARGET imgui)
    pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)

    # Where to find imgui backend .cpp files
    # Default: Debian/Ubuntu libimgui-dev location; override with -DIMGUI_BACKENDS_DIR=...
    set(IMGUI_BACKENDS_DIR "/usr/share/doc/libimgui-dev/examples/backends"
        CACHE PATH "Path to imgui backend .cpp files (imgui_impl_sdl.cpp etc.)")
    # Detect whether the SDL backend uses the SDL2-era name (imgui_impl_sdl.cpp)
    # or the newer name (imgui_impl_sdl2.cpp)
    if(EXISTS "${IMGUI_BACKENDS_DIR}/imgui_impl_sdl2.cpp")
        set(IMGUI_SDL_IMPL "${IMGUI_BACKENDS_DIR}/imgui_impl_sdl2.cpp")
    elseif(EXISTS "${IMGUI_BACKENDS_DIR}/imgui_impl_sdl.cpp")
        set(IMGUI_SDL_IMPL "${IMGUI_BACKENDS_DIR}/imgui_impl_sdl.cpp")
    else()
        message(FATAL_ERROR "Cannot find imgui SDL backend in ${IMGUI_BACKENDS_DIR}")
    endif()
endif()

add_library(dedective_core
    src/hackrf_source.cpp
    src/pluto_source.cpp
    src/packet_receiver.cpp
    src/packet_decoder.cpp
    src/wideband_monitor.cpp
    src/audio_output.cpp
    src/g72x.c
)

target_link_libraries(dedective_core PUBLIC
    PkgConfig::HACKRF
    PkgConfig::PULSE
    PkgConfig::IIO
)

target_include_directories(dedective_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

add_executable(dedective
    src/main.cpp
)

target_link_libraries(dedective PRIVATE dedective_core)

target_include_directories(dedective PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if(BUILD_GUI)

add_executable(dedective_gui
    src/gui_main.cpp
    ${IMGUI_SDL_IMPL}
    ${IMGUI_BACKENDS_DIR}/imgui_impl_opengl3.cpp
)

target_include_directories(dedective_gui PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    /usr/include/imgui
    /usr/include/imgui/backends
    "${IMGUI_BACKENDS_DIR}"
)

target_link_libraries(dedective_gui PRIVATE
    dedective_core
    PkgConfig::IMGUI
    PkgConfig::SDL2
    OpenGL::GL
)

endif() # BUILD_GUI
