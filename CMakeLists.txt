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
        ${PROJECT_SOURCE_DIR}/Sources/Headers
        ${PROJECT_SOURCE_DIR}/ObsData
        ${PROJECT_SOURCE_DIR}/executable
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/lib/sciplot
        ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
)

add_executable(CombinedNavigation Sources/main.cpp Sources/mainwindow.cpp Sources/Headers/mainwindow.h Sources/UI/mainwindow.ui
        include/Matrix.h Sources/Matrix.cpp include/INSData.h Sources/INSData.cpp include/TimeSys.h Sources/TimeSys.cpp
        include/INSSolution.h Sources/INSSolution.cpp
        ${PROJECT_SOURCE_DIR}/lib/sciplot/ ${PROJECT_SOURCE_DIR}/lib/sciplot/specs
        )


# 链接qt库文件
target_link_libraries(CombinedNavigation
        Qt::Core
        Qt::Gui
        Qt::Widgets
        )



