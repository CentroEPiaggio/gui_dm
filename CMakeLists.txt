cmake_minimum_required(VERSION 2.8.3)
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

find_package(catkin REQUIRED COMPONENTS rviz roscpp tf_conversions dual_manipulation_shared)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui QtSvg REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(GUI_MOC_FILES
  include/dual_manipulation_gui.h
)

qt4_wrap_cpp(WIDGETS_MOC_FILES
  include/widgets/render_3d_widget.h
  include/widgets/camera_widget.h
  include/widgets/state_machine_widget.h
  include/widgets/state_widget.h
  include/widgets/control_widget.h
  include/widgets/target_widget.h
  include/widgets/image_widget.h
)

set(WIDGET_SOURCE_FILES
  src/widgets/render_3d_widget.cpp
  src/widgets/camera_widget.cpp
  src/widgets/state_machine_widget.cpp
  src/widgets/state_widget.cpp
  src/widgets/control_widget.cpp
  src/widgets/target_widget.cpp
  src/widgets/image_widget.cpp
)

set(SOURCE_FILES
  src/dual_manipulation_gui.cpp
  src/main.cpp
  ${GUI_MOC_FILES}
  ${WIDGETS_MOC_FILES}
  ${WIDGET_SOURCE_FILES}
)

add_executable(dual_manipulation_gui ${SOURCE_FILES})

target_link_libraries(dual_manipulation_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(dual_manipulation_gui dual_manipulation_shared_gencpp)