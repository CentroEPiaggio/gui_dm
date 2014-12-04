cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_gui)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
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
)

set(WIDGET_SOURCE_FILES
  src/widgets/render_3d_widget.cpp
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

add_executable(dual_manipulation_gui ${SOURCE_FILES})

target_link_libraries(dual_manipulation_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})