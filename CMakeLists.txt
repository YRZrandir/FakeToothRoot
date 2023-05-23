cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 20)
project("FakeToothRoot" C CXX)

set(CMAKE_PREFIX_PATH "D:/DEP/")

set(Eigen3_DIR "D:/DEP/share/eigen3/cmake")
find_package(CGAL REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(assimp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(FakeToothRoot "main.cpp" "Polyhedron.cpp" "Polyhedron.h" "FakeToothRoot.cpp" "FakeToothRoot.h")

target_link_libraries(FakeToothRoot PRIVATE CGAL::CGAL assimp::assimp nlohmann_json::nlohmann_json OpenMP::OpenMP_CXX Eigen3::Eigen)