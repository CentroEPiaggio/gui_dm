cmake_minimum_required(VERSION 2.8.12)
project(dual_manipulation_gui)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS rviz roscpp tf_conversions dual_manipulation_shared pacman_vision_comm)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(GUI_MOC_FILES
    include/dual_manipulation_gui.h
)
set(VISUALIZATION_GUI_MOC_FILES
    include/dual_manipulation_visualization_gui.h
)
set(STOP_WINDOW_MOC_FILES
    include/stop_window.h
)
set(WIDGETS_MOC_FILES
    include/widgets/render_3d_widget.h
    include/widgets/camera_widget.h
    include/widgets/graph_widget.h
    include/widgets/state_machine_widget.h
    include/widgets/state_widget.h
    include/widgets/control_widget.h
    include/widgets/target_widget.h
    include/widgets/image_widget.h
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
    find_package(Qt4 COMPONENTS QtCore QtGui QtSvg REQUIRED)
    include(${QT_USE_FILE})
elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    find_package(Qt5Core)
    find_package(Qt5Gui)
    find_package(Qt5Svg)
    find_package(Qt5Widgets)
    include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Svg_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
    set(QT_LIBRARIES
        ${Qt5Core_LIBRARIES}
        ${Qt5Gui_LIBRARIES}
        ${Qt5Svg_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
    )
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(WIDGET_SOURCE_FILES
  src/widgets/render_3d_widget.cpp
  src/widgets/camera_widget.cpp
  src/widgets/graph_widget.cpp
  src/widgets/state_machine_widget.cpp
  src/widgets/state_widget.cpp
  src/widgets/control_widget.cpp
  src/widgets/target_widget.cpp
  src/widgets/image_widget.cpp
  src/widgets/message_widget.cpp
)

set(SOURCE_FILES
  src/dual_manipulation_gui.cpp
  src/main.cpp
  ${GUI_MOC_FILES}
  ${WIDGETS_MOC_FILES}
  ${WIDGET_SOURCE_FILES}
)

set(VISUALIZATION_SOURCE_FILES
  src/dual_manipulation_visualization_gui.cpp
  src/main_visualization.cpp
  ${VISUALIZATION_GUI_MOC_FILES}
  ${WIDGETS_MOC_FILES}
  ${WIDGET_SOURCE_FILES}
)

set(STOP_WINDOW_SOURCE_FILES
  src/stop_window.cpp
  src/main_stop_window.cpp
  ${STOP_WINDOW_MOC_FILES}
  ${WIDGETS_MOC_FILES}
  ${WIDGET_SOURCE_FILES}
)

add_executable(dual_manipulation_gui ${SOURCE_FILES})
target_link_libraries(dual_manipulation_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(visualization_gui ${VISUALIZATION_SOURCE_FILES})
target_link_libraries(visualization_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_executable(stop_window ${STOP_WINDOW_SOURCE_FILES})
target_link_libraries(stop_window ${QT_LIBRARIES} ${catkin_LIBRARIES})
