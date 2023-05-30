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
        ${PROJECT_SOURCE_DIR}/executable
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/lib/sciplot
        ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
        ${PROJECT_SOURCE_DIR}/lib/Matrix
        ${PROJECT_SOURCE_DIR}/lib/TimeSys
        ${PROJECT_SOURCE_DIR}/lib/Filter
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
        ${PROJECT_SOURCE_DIR}/lib/Matrix/Matrix.h ${PROJECT_SOURCE_DIR}/lib/Matrix/Matrix.cpp
        ${PROJECT_SOURCE_DIR}/lib/TimeSys/TimeSys.h ${PROJECT_SOURCE_DIR}/lib/TimeSys/TimeSys.cpp
        ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.h ${PROJECT_SOURCE_DIR}/lib/Filter/cKalman.cpp)


# 链接qt库文件
target_link_libraries(CombinedNavigation
        Qt::Core
        Qt::Gui
        Qt::Widgets
        )



