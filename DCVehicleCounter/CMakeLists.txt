# This is only for building on APPLE, not tested on any other platform
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(DCVC CXX)

FIND_PACKAGE(OpenCV REQUIRED core highgui imgproc video)

ADD_DEFINITIONS(-DMADEBYCMAKE)
SET( CMAKE_BUILD_TYPE "Custom" )
SET( exec_name "DCVC" )
SET(DCVC_files BackgroundProcessor.cpp BackgroundProcessor.h
    ErrorCodes.h
    ImageEnhancer.cpp ImageEnhancer.h
    ObjectSplitterVehicleCounter.cpp ObjectSplitterVehicleCounter.h
    Resizer.cpp Resizer.h
    VehicleTracking.cpp VehicleTracking.h
    VehicleMerger.cpp VehicleMerger.h
    ObjectSplitter.cpp
    ObjectSplitter.h
    EdgeDetector.cpp
    EdgeDetector.h
    Blinker.cpp Blinker.h
    SaveLoadConfig.cpp SaveLoadConfig.h
    main.cpp)

####################### Linux
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
MESSAGE("BUILDING ON LINUX, install dir .")

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
LINK_DIRECTORIES("/usr/local/lib")
ADD_EXECUTABLE(${exec_name} ${DCVC_files})
INSTALL(TARGETS ${exec_name} BUNDLE DESTINATION .
        RUNTIME DESTINATION .)
SET(APPS ${exec_name})
SET(CPACK_GENERATOR "TGZ")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
##############################

######################## MacOSX
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
MESSAGE("BUILDING ON APPLE")
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3 -mmacosx-version-min=10.9")
LINK_DIRECTORIES("/opt/local/lib")
ADD_EXECUTABLE(${exec_name} MACOSX_BUNDLE ${DCVC_files} )

SET(video_dest_dir ${exec_name}.app/Contents/MacOS)
SET(APPS "\${CMAKE_INSTALL_PREFIX}/${exec_name}.app")

SET_SOURCE_FILES_PROPERTIES(
  "Day_Medium_traffic.3gp"
  "config"
  PROPERTIES
  MACOSX_PACKAGE_LOCATION MacOS
  )

INSTALL(TARGETS DCVC BUNDLE DESTINATION ${CMAKE_INSTALL_PREFIX}
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})

SET(CPACK_BINARY_DRAGNDROP ON)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
###############################


TARGET_LINK_LIBRARIES(${exec_name} ${OpenCV_LIBS})

SET(DIRS ${OpenCV_LIBS} "/opt/local/lib" "/usr/local/lib" "/usr/lib")
INSTALL(CODE "
    include(BundleUtilities)
    fixup_bundle(\"${APPS}\" \"\" \"${DIRS}\")
")

INCLUDE(CPack)
