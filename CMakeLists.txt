cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_gui)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11 -o3" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x -o3" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#find_package(catkin REQUIRED COMPONENTS rviz roscpp ps3joy drc_shared)
find_package(rviz REQUIRED)
find_package(roscpp REQUIRED)
find_package(catkin REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

catkin_package(
CATKIN_DEPENDS 
)

set(catkin_LIBRARIES "${roscpp_LIBRARIES}"  "${rviz_LIBRARIES}" "${rosconsole_bridge_LIBRARIES}")

set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} /usr/include/OGRE)
link_directories(${catkin_LIBRARY_DIRS} ${ROS_LIB_FOLDER})

add_definitions(-DQT_NO_KEYWORDS)


qt4_wrap_cpp(GUI_MOC_FILES
  include/dual_manipulation_gui.h
)

qt4_wrap_cpp(WIDGETS_MOC_FILES
  include/widgets/render_widget.h
  include/widgets/camera_widget.h
  include/widgets/state_machine_widget.h
  include/widgets/state_widget.h
  include/widgets/control_widget.h
)

set(WIDGET_SOURCE_FILES
  src/widgets/render_widget.cpp
  src/widgets/camera_widget.cpp
  src/widgets/state_machine_widget.cpp
  src/widgets/state_widget.cpp
  src/widgets/control_widget.cpp
)

set(SOURCE_FILES
  src/dual_manipulation_gui.cpp
  src/main.cpp
  ${GUI_MOC_FILES}
  ${WIDGETS_MOC_FILES}
  ${WIDGET_SOURCE_FILES}
)

add_executable(dual_manipulation_gui ${SOURCE_FILES} ${QT_FORMS_HPP})

target_link_libraries(dual_manipulation_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS dual_manipulation_gui
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)