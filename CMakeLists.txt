cmake_minimum_required(VERSION 3.16)
project(green_glow_tracker VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# MSVC: source files contain UTF-8 comments / Chinese strings.
if(MSVC)
  add_compile_options(/utf-8 /W3)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()

find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs calib3d videoio)
message(STATUS "OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

# ---------------------------------------------------------------------------
# core library: all algorithm modules + io (src/*.cpp except main.cpp)
# ---------------------------------------------------------------------------
file(GLOB GT_CORE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM GT_CORE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

add_library(glowtrack_core STATIC ${GT_CORE_SOURCES})
target_include_directories(glowtrack_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(glowtrack_core PUBLIC ${OpenCV_LIBS})

# ---------------------------------------------------------------------------
# executables
# ---------------------------------------------------------------------------
add_executable(glowtrack src/main.cpp)
target_link_libraries(glowtrack PRIVATE glowtrack_core)

add_executable(glowtrack_synth tools/synth_dataset.cpp)
target_link_libraries(glowtrack_synth PRIVATE glowtrack_core)

add_executable(glowtrack_eval tools/eval_metrics.cpp)
target_link_libraries(glowtrack_eval PRIVATE glowtrack_core)

add_executable(glowtrack_devdet tools/dev_detector.cpp)
target_link_libraries(glowtrack_devdet PRIVATE glowtrack_core)

# ---------------------------------------------------------------------------
# unit tests (tiny built-in framework, no external dependency)
# ---------------------------------------------------------------------------
enable_testing()
file(GLOB GT_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")
list(REMOVE_ITEM GT_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_main.cpp")

add_executable(glowtrack_tests tests/test_main.cpp ${GT_TEST_SOURCES})
target_include_directories(glowtrack_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tests")
target_link_libraries(glowtrack_tests PRIVATE glowtrack_core)
add_test(NAME glowtrack_unit_tests COMMAND glowtrack_tests)

# ---------------------------------------------------------------------------
# Windows: copy the OpenCV runtime DLLs next to every executable so that the
# build output directory is directly runnable (no PATH fiddling).
# ---------------------------------------------------------------------------
if(WIN32)
  get_filename_component(_ocv_bin_dir "${OpenCV_DIR}/../bin" ABSOLUTE)
  file(GLOB _ocv_runtime_dlls
       "${_ocv_bin_dir}/opencv_world*.dll"
       "${_ocv_bin_dir}/opencv_videoio_ffmpeg*.dll")
  if(_ocv_runtime_dlls)
    foreach(_tgt glowtrack glowtrack_synth glowtrack_eval glowtrack_devdet glowtrack_tests)
      add_custom_command(TARGET ${_tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_ocv_runtime_dlls}
                "$<TARGET_FILE_DIR:${_tgt}>")
    endforeach()
  else()
    message(WARNING "OpenCV runtime DLLs not found next to ${OpenCV_DIR}/../bin")
  endif()
endif()
