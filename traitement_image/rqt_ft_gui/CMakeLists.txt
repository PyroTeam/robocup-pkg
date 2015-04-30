cmake_minimum_required(VERSION 2.8.3)

project(rqt_ft_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_ft_gui_SRCS
  src/rqt_ft_gui/ft_gui.cpp
  src/rqt_ft_gui/ratio_layouted_frame.cpp
)

set(rqt_ft_gui_HDRS
  include/rqt_ft_gui/ft_gui.h
  include/rqt_ft_gui/ratio_layouted_frame.h
)

set(rqt_ft_gui_UIS
  src/rqt_ft_gui/ft_gui.ui
)

set(rqt_ft_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
	INCLUDE_DIRS ${rqt_ft_gui_INCLUDE_DIRECTORIES}
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge
)
catkin_python_setup()

qt4_wrap_cpp(rqt_ft_gui_MOCS ${rqt_ft_gui_HDRS})
qt4_wrap_ui(rqt_ft_gui_UIS_H ${rqt_ft_gui_UIS})

include_directories(${rqt_ft_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_ft_gui_SRCS} ${rqt_ft_gui_MOCS} ${rqt_ft_gui_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_ft_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
