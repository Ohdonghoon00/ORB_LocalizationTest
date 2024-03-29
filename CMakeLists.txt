cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Compression
enable_language(CXX)
set(EXTRA_LIBS /opt/gurobi950/linux64/lib/libgurobi95.so
/opt/gurobi950/linux64/lib/libgurobi_c++.a
/opt/gurobi950/linux64/lib/libgurobi.so.9.5.0
)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
"/opt/gurobi950/linux64/include/"
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/gurobi_helper.cpp
src/Compression.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${EXTRA_LIBS}

)

message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Build examples

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum
# Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_euroc2
Examples/Monocular/mono_euroc2.cc)
target_link_libraries(mono_euroc2 ${PROJECT_NAME})

# Compression
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Compression)

add_executable(landmarkCompression
Compression/main_landmarkCompression.cc)
target_link_libraries(landmarkCompression ${PROJECT_NAME})

add_executable(landmarkIQPCompression
Compression/main_landmarkIQPCompression.cc)
target_link_libraries(landmarkIQPCompression ${PROJECT_NAME})

add_executable(keyframeILPCompression
Compression/main_keyframeILPCompression.cc)
target_link_libraries(keyframeILPCompression ${PROJECT_NAME})

add_executable(keyframeSimilarityCompression
Compression/main_keyframeSimilarityCompression.cc)
target_link_libraries(keyframeSimilarityCompression ${PROJECT_NAME})

add_executable(keyframeIQPCompression
Compression/main_keyframeIQPCompression.cc)
target_link_libraries(keyframeIQPCompression ${PROJECT_NAME})

add_executable(randomCompression
Compression/main_randomCompression.cc)
target_link_libraries(randomCompression ${PROJECT_NAME})

add_executable(readDbInfo
Compression/main_Db_info.cc)
target_link_libraries(readDbInfo ${PROJECT_NAME})

# Bundle adjusment
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Compression)

add_executable(ba
Compression/main_fullBa.cc)
target_link_libraries(ba ${PROJECT_NAME} Ceres::ceres)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})

if(USE_EXAMPLE_WEBCAM)
    find_package(PkgConfig REQUIRED)
    pkg_search_module(CURL REQUIRED libcurl)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    add_executable(mono_uvc Examples/mono_uvc.cpp)
    target_link_libraries(mono_uvc ${PROJECT_NAME})
    add_executable(mono_android_ipcam Examples/mono_android_ipcam.cpp)
    target_link_libraries(mono_android_ipcam ${PROJECT_NAME} ${CURL_LIBRARIES})
    add_executable(mono_android_ipcam_ar Examples/mono_android_ipcam_ar.cpp Examples/AR/ViewerAR.cc)
    target_include_directories(mono_android_ipcam_ar PUBLIC Examples/AR/)
    target_link_libraries(mono_android_ipcam_ar ${PROJECT_NAME} ${CURL_LIBRARIES})
endif()
