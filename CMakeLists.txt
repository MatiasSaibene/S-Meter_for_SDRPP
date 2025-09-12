cmake_minimum_required(VERSION 4.1.0)
project(SMeterModule)

file(GLOB_RECURSE SRC 
    "src/*.cpp"
    "src/*.hpp"
)

if (NOT SDRPP_MODULE_CMAKE)
    set(SDRPP_MODULE_CMAKE "/usr/share/cmake/Modules/sdrpp_module.cmake")
endif ()

include(${SDRPP_MODULE_CMAKE})

target_link_libraries(SMeterModule PRIVATE fftw3)
target_include_directories(SMeterModule PRIVATE "src/")