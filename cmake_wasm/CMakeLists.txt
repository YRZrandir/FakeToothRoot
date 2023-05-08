cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project("FakeToothRoot" C CXX)
set(EMROOT /home/yrz/dev/emsdk/upstream/emscripten)
set(CMAKE_TOOLCHAIN_FILE ${EMROOT}/cmake/Modules/Platform/Emscripten.cmake)
set(CMAKE_CXX_COMPILER ${EMROOT}/em++)
set(CMAKE_C_COMPILER ${EMROOT}/emcc)
set(CMAKE_BUILD_TYPE Release)
set(EMLIBS "../../dev/emlibs")

include_directories(../dep/json/include)
include_directories(../dep/CGAL)
include_directories(../dep/boost)


add_executable(FakeToothRoot "../main.cpp" "../Polyhedron.cpp" "../FakeToothRoot.cpp")
set_target_properties(FakeToothRoot
        PROPERTIES SUFFIX ".mjs"
        COMPILE_FLAGS "-O3 --no-entry"
        LINK_FLAGS " -O3\
        -s \"EXPORTED_RUNTIME_METHODS=['cwrap','ccall', 'getValue', 'setValue']\" \
        -s \"EXPORTED_FUNCTIONS=['_GenFakeToothRoot', '_malloc', '_free']\" \
        -sINITIAL_MEMORY=512MB -sMAXIMUM_MEMORY=2GB -sALLOW_MEMORY_GROWTH \
        -sSINGLE_FILE=1\
        --no-entry -s ENVIRONMENT='web' -s EXPORT_NAME='GetModule'")