cmake_minimum_required(VERSION 3.16)
project(video-stabilization-boost)

enable_language(C)
enable_language(CXX)

include(${CMAKE_SOURCE_DIR}/conanfile.cmake)
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
conan_basic_setup(KEEP_RPATHS)

# fmt
find_package(fmt REQUIRED)
function(use_fmt_package TARGET_NAME)
  target_include_directories(${TARGET_NAME} PRIVATE ${fmt_INCLUDE_DIRS})
  target_link_libraries(${TARGET_NAME} PRIVATE ${fmt_LIBRARIES})
endfunction()

# opencv
find_package(OpenCV REQUIRED)
function(use_opencv_package TARGET_NAME)
  # target_include_directories(${TARGET_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
  # target_link_libraries(${TARGET_NAME} PRIVATE ${OpenCV_LIBRARIES})
  target_link_libraries(${TARGET_NAME} PRIVATE opencv::opencv)
endfunction()

# rapidjson
find_package(RapidJSON REQUIRED)
function(use_rapidjson_package TARGET_NAME)
  target_link_libraries(${TARGET_NAME} PRIVATE RapidJSON::RapidJSON)
endfunction()

set(PROJECT_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/include")
set(PROJECT_SOURCES_PATH "${CMAKE_SOURCE_DIR}/source")

include_directories(${PROJECT_INCLUDE_PATH})
add_subdirectory(${PROJECT_SOURCES_PATH})
