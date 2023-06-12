cmake_minimum_required(VERSION 3.20)
project(CombinedNavigation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

############################################ 设置资源路径 ###############################################################
set(SRC_DIR "${PROJECT_SOURCE_DIR}/src")
file(GLOB SRC_SUB_DIR "${SRC_DIR}/*")
# IO
set(FILEIO_CPP_DIR "${SRC_DIR}/FileIO/")
file(GLOB FILEIO_CPP_SOURCES "${FILEIO_CPP_DIR}/*.cpp" "${FILEIO_CPP_DIR}/*.cxx"
        "${FILEIO_CPP_DIR}/*.cc" "${FILEIO_CPP_DIR}/*.c")
# INS
set(INS_CPP_DIR "${SRC_DIR}/INS")
file(GLOB INS_CPP_SOURCES "${INS_CPP_DIR}/*.cpp" "${INS_CPP_DIR}/*.cxx"
        "${INS_CPP_DIR}/*.cc" "${INS_CPP_DIR}/*.c")
# Common
set(COMMON_CPP_DIR "${SRC_DIR}/Common")
file(GLOB COMMON_CPP_SOURCES "${COMMON_CPP_DIR}/*.cpp" "${COMMON_CPP_DIR}/*.cxx"
        "${COMMON_CPP_DIR}/*.cc" "${COMMON_CPP_DIR}/*.c")
# GNSS
set(GNSS_CPP_DIR "${SRC_DIR}/GNSS")
file(GLOB GNSS_CPP_SOURCES "${GNSS_CPP_DIR}/*.cpp" "${GNSS_CPP_DIR}/*.cxx"
        "${GNSS_CPP_DIR}/*.cc" "${GNSS_CPP_DIR}/*.c")
# LC
set(LC_CPP_DIR "${SRC_DIR}/LC")
file(GLOB LC_CPP_SOURCES "${LC_CPP_DIR}/*.cpp" "${LC_CPP_DIR}/*.cxx"
        "${LC_CPP_DIR}/*.cc" "${LC_CPP_DIR}/*.c")


########################################### 第三方库路径设置 #############################################################
set(THIRD_LIB_DIR "${PROJECT_SOURCE_DIR}/lib")
file(GLOB THIRD_LIB_SUB_DIR "${THIRD_LIB_DIR}/*")
# yaml-cpp库路径设置
set(YAML_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/yaml-cpp/include")
set(YAML_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/yaml-cpp/build/libyaml-cpp.a")
# 矩阵库路径设置
set(MATRIX_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/Matrix")
set(MATRIX_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/Matrix/build/libMatrix.dylib")
# 时间系统路径设置
set(TIME_SYSTEM_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/TimeSys")
set(TIME_SYSTEM_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/TimeSys/build/libTIME_SYSTEM_CPP.dylib")
# 坐标系统路径设置
set(COORSYSTEM_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/CoorSys")
set(COORSYSTEM_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/CoorSys/build/libCOORSYSTEM.dylib")
# 终端输出库路径设置
set(TERMINAL_CPP_LIB "${THIRD_LIB_DIR}/TerminalMessage/build/libTerminalMessage.dylib")



# 找到QT解释器
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/Sources/UIHeaders
        ${PROJECT_SOURCE_DIR}/Sources/UI
        # 第三方库
        ${PROJECT_SOURCE_DIR}/lib
        ${THIRD_LIB_SUB_DIR}
        ${YAML_CPP_INCLUDE_DIR}
        # 资源库
        ${SRC_SUB_DIR}
        ${SRC_DIR}
)

add_executable(INS src/Process/ins.cpp
        ${FILEIO_CPP_SOURCES} ${INS_CPP_SOURCES} ${COMMON_CPP_SOURCES} ${GNSS_CPP_SOURCES}
        include/BasicFuns.h Sources/BasicFuns.cpp)

add_executable(LC src/Process/lc.cpp
        # 资源文件
        ${FILEIO_CPP_SOURCES} ${INS_CPP_SOURCES} ${COMMON_CPP_SOURCES} ${GNSS_CPP_SOURCES} ${LC_CPP_SOURCES}
        include/BasicFuns.h Sources/BasicFuns.cpp
        # 依赖文件
        ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.h ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.cpp
        )

add_executable(plt src/Process/plt.cpp
        # 读取文件
        ${FILEIO_CPP_SOURCES} ${INS_CPP_SOURCES} ${COMMON_CPP_SOURCES} ${GNSS_CPP_SOURCES}
        include/BasicFuns.h Sources/BasicFuns.cpp
        # 画图依赖
        ${PROJECT_SOURCE_DIR}/lib/sciplot/ ${PROJECT_SOURCE_DIR}/lib/sciplot/specs)

add_executable(CombinedNavigation Sources/main.cpp
        # UI文件
        Sources/mainwindow.cpp Sources/UIHeaders/mainwindow.h Sources/UI/mainwindow.ui
        # 资源文件
        ${FILEIO_CPP_SOURCES} ${INS_CPP_SOURCES} ${COMMON_CPP_SOURCES} ${GNSS_CPP_SOURCES} ${LC_CPP_SOURCES}
        include/BasicFuns.h Sources/BasicFuns.cpp
        # 依赖文件
        ${PROJECT_SOURCE_DIR}/lib/sciplot/ ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
        ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.h ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.cpp
        )

set_target_properties(INS PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set_target_properties(LC PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set_target_properties(plt PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

target_link_libraries(LC
        Qt::Core
        # yaml-cpp库链接
        ${YAML_CPP_LIB}
        # 矩阵库链接
        ${MATRIX_CPP_LIB}
        # 时间系统库链接
        ${TIME_SYSTEM_CPP_LIB}
        # 坐标系统库链接
        ${COORSYSTEM_CPP_LIB}
        # 终端输出库
        ${TERMINAL_CPP_LIB}
        )

target_link_libraries(INS
        Qt::Core
        # yaml-cpp库链接
        ${YAML_CPP_LIB}
        # 矩阵库链接
        ${MATRIX_CPP_LIB}
        # 时间系统库链接
        ${TIME_SYSTEM_CPP_LIB}
        # 坐标系统库链接
        ${COORSYSTEM_CPP_LIB}
        # 终端输出库
        ${TERMINAL_CPP_LIB}
        )

target_link_libraries(plt
        Qt::Core
        # yaml-cpp库链接
        ${YAML_CPP_LIB}
        # 终端输出库
        ${TERMINAL_CPP_LIB}
        # 矩阵库链接
        ${MATRIX_CPP_LIB}
        # 时间系统库链接
        ${TIME_SYSTEM_CPP_LIB}
        # 坐标系统库链接
        ${COORSYSTEM_CPP_LIB})


# 链接qt库文件
target_link_libraries(CombinedNavigation
        # qt动态库链接
        Qt::Core
        Qt::Gui
        Qt::Widgets
        # yaml-cpp库链接
        ${YAML_CPP_LIB}
        # 矩阵库链接
        ${MATRIX_CPP_LIB}
        # 时间系统库链接
        ${TIME_SYSTEM_CPP_LIB}
        # 坐标系统库链接
        ${COORSYSTEM_CPP_LIB}
        )



