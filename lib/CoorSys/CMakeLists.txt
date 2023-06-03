cmake_minimum_required(VERSION 3.4)
project(COORSYSTEM VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

# 添加依赖矩阵的目录和共享库
get_filename_component(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
message(${LIB_DIR})
set(LIB_MATRIX "${LIB_DIR}/Matrix")
set(LIB_MATRIX_SHARED "${LIB_DIR}/Matrix/build/libMatrix.dylib")

set(COORSYSTEM_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB COORSYSTEM_SOURCES "${COORSYSTEM_DIR}/*.cpp" "${COORSYSTEM_DIR}/*.cxx"
        "${COORSYSTEM_DIR}/*.cc" "${COORSYSTEM_DIR}/*.c")


include_directories(${COORSYSTEM_DIR} ${LIB_MATRIX})
add_library(COORSYSTEM SHARED "${COORSYSTEM_SOURCES}")

target_link_libraries(COORSYSTEM
        # 添加矩阵库依赖
        ${LIB_MATRIX_SHARED}
)