project(Marker3_detector_gpu)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
find_package(OpenCV REQUIRED)

# ------------------ User setup ------------------ #
add_definitions("-std=c++11") # Enable C++11.
# ------------------------------------------------ #

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
