cmake_minimum_required(VERSION 2.8.3)
project(rqt_liprec)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  OpenCV
  cv_bridge
  image_transport
  rqt_gui
  rqt_gui_cpp
  rqt_gui_py
)

#find_package(Boost REQUIRED system filesystem date_time thread iostreams)

find_package(OpenCV REQUIRED)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})


set(rqt_liprec_SRCS
	src/rqt_liprec/LipRec.cpp
)
set(rqt_liprec_HDRS
	include/rqt_liprec/LipRec.h
)

set(rqt_liprec_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR}
)

set(rqt_liprec_UIS
	src/rqt_liprec/liprec.ui
)

catkin_python_setup()
#generate_messages(
#  DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy rqt_gui rqt_gui_cpp rqt_gui_py message_runtime
  DEPENDS
    #Boost
)

qt4_wrap_cpp(rqt_liprec_MOCS ${rqt_liprec_HDRS})
qt4_wrap_ui(rqt_liprec_UIS_H ${rqt_liprec_UIS})

include_directories(${rqt_liprec_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})



add_library(swt src/rqt_liprec/Swt.cpp)
target_link_libraries(swt ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_library(dtw src/rqt_liprec/Dtw.cpp)
target_link_libraries(dtw ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_library(imageProcessing src/rqt_liprec/ImageProcessing.cpp)
target_link_libraries(imageProcessing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_library(keyPointsDeliverer src/rqt_liprec/KeyPointsDeliverer.cpp)
target_link_libraries(keyPointsDeliverer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} imageProcessing)

add_library(zernikeMoments src/rqt_liprec/ZernikeMoments.cpp)
target_link_libraries(zernikeMoments ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_library(${PROJECT_NAME} ${rqt_liprec_SRCS} ${rqt_liprec_MOCS} ${rqt_liprec_UIS_H})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} imageProcessing keyPointsDeliverer swt dtw zernikeMoments)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY resource
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/rqt_liprec
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_liprec
DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

