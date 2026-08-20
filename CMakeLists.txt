cmake_minimum_required(VERSION 3.16)

project(hagie_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(
    hagie_gui

    main.cpp

    gui/mainwindow.cpp
    gui/mainwindow.h

    core/hagie_state.cpp
    core/hagie_state.h
    core/height_target_controller.cpp

    stm32/protocol.cpp
    stm32/protocol.h

    stm32/stm32canbusif.cpp
    stm32/stm32canbusif.h

    stm32/stm32_worker.cpp
    stm32/stm32_worker.h

    vision/vision_height_source.cpp
    vision/vision_3d_processor.cpp
    vision/vision_3d_worker.cpp
    vision/simulated_point_cloud_source.cpp
)

target_include_directories(
    hagie_gui
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    hagie_gui
    PRIVATE
    Qt6::Widgets
)


add_executable(
    vision_3d_processor_test

    tests/vision_3d_processor_test.cpp

    vision/vision_3d_processor.cpp
)

target_include_directories(
    vision_3d_processor_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)


add_executable(
    vision_3d_worker_test

    tests/vision_3d_worker_test.cpp
    tests/synthetic_point_cloud_source_test.cpp

    core/hagie_state.cpp

    vision/vision_height_source.cpp
    vision/vision_3d_processor.cpp
    vision/vision_3d_worker.cpp
)

target_include_directories(
    vision_3d_worker_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)