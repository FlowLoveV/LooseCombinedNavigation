cmake_minimum_required(VERSION 3.20)
project(CombinedNavigation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/Sources/Headers
        ${PROJECT_SOURCE_DIR}/ObsData
)

add_executable(CombinedNavigation Sources/main.cpp Sources/mainwindow.cpp Sources/Headers/mainwindow.h Sources/UI/mainwindow.ui
        include/Matrix.h Sources/Matrix.cpp include/INSData.h Sources/INSData.cpp include/TimeSys.h Sources/TimeSys.cpp include/INSSolution.h Sources/INSSolution.cpp)
target_link_libraries(CombinedNavigation
        Qt::Core
        Qt::Gui
        Qt::Widgets
        )

