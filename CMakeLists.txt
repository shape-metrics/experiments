cmake_minimum_required(VERSION 3.16)
project(OGDF_experiments LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(domus)
add_subdirectory(OGDF)

add_executable(experiments
    src/experiments/ogdf-drawer/drawer.cpp
    src/experiments/shape_metrics.cpp
    src/experiments/ogdf.cpp
    src/main.cpp
)

add_executable(gen
    src/gen.cpp
)

target_compile_features(experiments PRIVATE cxx_std_26)

target_link_libraries(experiments
    PRIVATE DOMUS::core OGDF
)

target_link_libraries(gen
    PRIVATE DOMUS::core
)