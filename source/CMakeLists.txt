cmake_minimum_required(VERSION 3.16)

# dummy target
add_executable(dummy ${CMAKE_CURRENT_SOURCE_DIR}/dummy.cpp)
target_precompile_headers(dummy PUBLIC ${PROJECT_INCLUDE_PATH}/precomp.h)
set_property(TARGET dummy PROPERTY CXX_STANDARD 17)
use_fmt_package(dummy)
use_opencv_package(dummy)
use_rapidjson_package(dummy)

function(add_custom_executable TARGET_NAME TARGET_SOURCE)
  add_executable(${TARGET_NAME} ${TARGET_SOURCE})
  set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 17)
  target_precompile_headers(${TARGET_NAME} REUSE_FROM dummy)
  use_fmt_package(${TARGET_NAME})
  use_opencv_package(${TARGET_NAME})
  use_rapidjson_package(${TARGET_NAME})
endfunction()

set(TARGET_NAME vstab-basic)
set(TARGET_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/vstab-basic.cpp)
add_custom_executable(${TARGET_NAME} ${TARGET_SOURCE})

set(TARGET_NAME vstab-boost)
set(TARGET_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/vstab-boost.cpp)
add_custom_executable(${TARGET_NAME} ${TARGET_SOURCE})
