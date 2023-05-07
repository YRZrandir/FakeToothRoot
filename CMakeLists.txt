cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project("FakeToothRoot" C CXX)

find_package(CGAL REQUIRED)
find_package(assimp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(FakeToothRoot "main.cpp" "Polyhedron.cpp" "Polyhedron.h" "FakeToothRoot.cpp" "FakeToothRoot.h")

target_link_libraries(FakeToothRoot CGAL::CGAL assimp::assimp nlohmann_json::nlohmann_json OpenMP::OpenMP_CXX)