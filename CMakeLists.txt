cmake_minimum_required(VERSION 3.20)
project(CombinedNavigation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 设置C标准
#set(CMAKE_C_STANDARD 99)
#set(CMAKE_C_EXTENSIONS OFF)
#set(CMAKE_C_STANDARD_REQUIRED ON)

# 找到Python解释器
#find_package (Python2 COMPONENTS Interpreter NumPy)
#find_package (Python COMPONENTS Interpreter Development)
#find_package(PythonInterp REQUIRED)
#find_package(PythonLibs ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} EXACT REQUIRED)

# 第三方库路径设置

# yaml-cpp库路径设置
set(YAML_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/yaml-cpp/include")
set(YAML_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/yaml-cpp/build/libyaml-cpp.a")
# 矩阵库路径设置
set(MATRIX_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/Matrix")
set(MATRIX_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/Matrix/build/libMatrix.dylib")
# 时间系统路径设置
set(TIME_SYSTEM_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/lib/TimeSys")
set(TIME_SYSTEM_CPP_LIB "${PROJECT_SOURCE_DIR}/lib/TimeSys/build/libTIME_SYSTEM_CPP.dylib")

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
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/lib/sciplot
        ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
        ${PROJECT_SOURCE_DIR}/lib/TimeSys
        ${PROJECT_SOURCE_DIR}/lib/Filter
        ${MATRIX_CPP_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
)

add_executable(CombinedNavigation Sources/main.cpp
        # UI文件
        Sources/mainwindow.cpp Sources/UIHeaders/mainwindow.h Sources/UI/mainwindow.ui
        # 主文件
        include/INSData.h Sources/INSData.cpp
        include/BasicFuns.h Sources/BasicFuns.cpp
        include/LooseCombination.h Sources/LooseCombination.cpp
        # 依赖文件
        ${PROJECT_SOURCE_DIR}/lib/sciplot/ ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
        ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.h ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.cpp)


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
        )



