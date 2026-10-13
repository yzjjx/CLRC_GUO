cmake_minimum_required(VERSION 3.10)
project(panda_clrc_guo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
option(BUILD_FRANKA_EXPERIMENTS "Build Panda hardware experiment programs" ON)
include(CTest)

add_library(clrc_guo
    src/LSCLRC_config.cpp
    src/LSCLRC_control.cpp
    src/History_window.cpp
    src/Panda_regressor.cpp
    src/generated/Panda_inverse_regressor.cpp
    src/generated/Panda_parameters.cpp
)
target_include_directories(clrc_guo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(clrc_guo PUBLIC Eigen3::Eigen)

if(BUILD_FRANKA_EXPERIMENTS)
    find_package(Franka REQUIRED)
    add_executable(lsclrc_fig3 src/lsclrc_fig3.cpp)
    target_link_libraries(lsclrc_fig3 PRIVATE clrc_guo Franka::Franka)
    add_executable(lsclrc_repeat src/lsclrc_fig3.cpp)
    target_compile_definitions(lsclrc_repeat PRIVATE LSCLRC_EXPERIMENT_ROUNDS=10 LSCLRC_ASYNC_COMPUTE=1)
    target_link_libraries(lsclrc_repeat PRIVATE clrc_guo Franka::Franka Threads::Threads)
    if(BUILD_TESTING)
        add_executable(lsclrc_startup_test tests/lsclrc_startup_test.cpp)
        target_link_libraries(lsclrc_startup_test PRIVATE clrc_guo Franka::Franka)
        add_test(NAME lsclrc_startup COMMAND lsclrc_startup_test)
    endif()
endif()

if(BUILD_TESTING)
    foreach(check IN ITEMS timing async parameter)
        add_executable(lsclrc_${check}_test tests/lsclrc_${check}_test.cpp)
        target_link_libraries(lsclrc_${check}_test PRIVATE clrc_guo Threads::Threads)
        add_test(NAME lsclrc_${check} COMMAND lsclrc_${check}_test)
    endforeach()
endif()
