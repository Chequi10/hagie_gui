cmake_minimum_required(VERSION 3.16)

project(hagie_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


# ============================================================
# OPCIONES
# ============================================================

option(
    HAGIE_ENABLE_ZED_SDK
    "Enable ZED SDK support"
    OFF
)


find_package(Qt6 REQUIRED COMPONENTS Widgets)


# ============================================================
# EJECUTABLE PRINCIPAL
# ============================================================

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

    vision/zed_gmsl_point_cloud_source.cpp
    vision/zed_gmsl_point_cloud_source.h

    vision/rgb_camera_worker.cpp
    vision/rgb_camera_worker.h
    vision/rgb_frame_source.h
    vision/simulated_rgb_frame_source.cpp
    vision/simulated_rgb_frame_source.h
    ai/tassel_detector.cpp
    ai/tassel_detector.h
    ai/tassel_counter.cpp
    ai/tassel_counter.h
    ai/tassel_verifier.cpp
    ai/tassel_verifier.h
    vision/zed_gmsl_rgb_frame_source.cpp
    vision/zed_gmsl_rgb_frame_source.h
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


# ============================================================
# ZED SDK
# ============================================================

if(HAGIE_ENABLE_ZED_SDK)

    message(
        STATUS
        "ZED SDK support ENABLED"
    )

    find_package(
        ZED REQUIRED
    )

    find_package(
        CUDA REQUIRED
    )

    target_compile_definitions(
        hagie_gui
        PRIVATE
        HAGIE_ENABLE_ZED_SDK
    )

    target_include_directories(
        hagie_gui
        PRIVATE
        ${ZED_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
    )

    target_link_libraries(
        hagie_gui
        PRIVATE
        ${ZED_LIBRARIES}
        ${CUDA_LIBRARIES}
    )

else()

    message(
        STATUS
        "ZED SDK support DISABLED"
    )

endif()

# ============================================================
# TEST VISION 3D PROCESSOR
# ============================================================

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


# ============================================================
# TEST VISION 3D WORKER
# ============================================================

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
# ============================================================
# TEST RGB CAMERA WORKER
# ============================================================

add_executable(
    rgb_camera_worker_test

    tests/rgb_camera_worker_test.cpp

    vision/rgb_camera_worker.cpp
    vision/simulated_rgb_frame_source.cpp
)


target_include_directories(
    rgb_camera_worker_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# ============================================================
# TEST TASSEL DETECTOR
# ============================================================

add_executable(
    tassel_detector_test

    tests/tassel_detector_test.cpp

    ai/tassel_detector.cpp

    vision/simulated_rgb_frame_source.cpp
)


target_include_directories(
    tassel_detector_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ============================================================
# TEST TASSEL COUNTER
# ============================================================

add_executable(
    tassel_counter_test

    tests/tassel_counter_test.cpp

    ai/tassel_counter.cpp
)


target_include_directories(
    tassel_counter_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# ============================================================
# TEST TASSEL VERIFIER
# ============================================================

add_executable(
    tassel_verifier_test

    tests/tassel_verifier_test.cpp

    ai/tassel_verifier.cpp
)


target_include_directories(
    tassel_verifier_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)