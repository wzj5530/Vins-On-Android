# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

#CMake版本信息
cmake_minimum_required(VERSION 3.4.1)


#配置加载native依赖
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/opencv)

include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/eigen3)
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/boost-1.53)
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/)

include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include/ceres)

include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/src/camera_model/include)
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/src/a2ir)

include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/src/vins_estimator/src/draw_result)

#静态方式加载
add_library( lib_opencv_calib3d STATIC IMPORTED )
set_target_properties(lib_opencv_calib3d PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_calib3d.a)

add_library( lib_opencv_core STATIC IMPORTED )
set_target_properties(lib_opencv_core PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_core.a)

add_library( lib_opencv_features2d STATIC IMPORTED )
set_target_properties(lib_opencv_features2d PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_features2d.a)

add_library( lib_opencv_flann STATIC IMPORTED )
set_target_properties(lib_opencv_flann PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_flann.a)

add_library( lib_opencv_highgui STATIC IMPORTED )
set_target_properties(lib_opencv_highgui PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_highgui.a)

add_library( lib_opencv_imgcodecs STATIC IMPORTED )
set_target_properties(lib_opencv_imgcodecs PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_imgcodecs.a)

add_library( lib_opencv_imgproc STATIC IMPORTED )
set_target_properties(lib_opencv_imgproc PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_imgproc.a)

add_library( lib_opencv_ml STATIC IMPORTED )
set_target_properties(lib_opencv_ml PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_ml.a)

add_library( lib_opencv_objdetect STATIC IMPORTED )
set_target_properties(lib_opencv_objdetect PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_objdetect.a)

add_library( lib_opencv_photo STATIC IMPORTED )
set_target_properties(lib_opencv_photo PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_photo.a)

add_library( lib_opencv_shape STATIC IMPORTED )
set_target_properties(lib_opencv_shape PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_shape.a)

add_library( lib_opencv_stitching STATIC IMPORTED )
set_target_properties(lib_opencv_stitching PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_stitching.a)

add_library( lib_opencv_superres STATIC IMPORTED )
set_target_properties(lib_opencv_superres PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_superres.a)

add_library( lib_opencv_ts STATIC IMPORTED )
set_target_properties(lib_opencv_ts PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_ts.a)

add_library( lib_opencv_video STATIC IMPORTED )
set_target_properties(lib_opencv_video PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_video.a)

add_library( lib_opencv_videoio STATIC IMPORTED )
set_target_properties(lib_opencv_videoio PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_videoio.a)

add_library( lib_opencv_videostab STATIC IMPORTED )
set_target_properties(lib_opencv_videostab PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_videostab.a)

#add_library( lib_tbb STATIC IMPORTED )
#set_target_properties(lib_tbb PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libtbb.a)

#add_library( lib_IlmImf STATIC IMPORTED )
#set_target_properties(lib_IlmImf PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libIlmImf.a)

#add_library( lib_libjasper STATIC IMPORTED )
#set_target_properties(lib_libjasper PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/liblibjasper.a)

#add_library( lib_libjpeg STATIC IMPORTED )
#set_target_properties(lib_libjpeg PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/liblibjpeg.a)

#add_library( lib_libpng STATIC IMPORTED )
#set_target_properties(lib_libpng PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/liblibpng.a)

#add_library( lib_libtiff STATIC IMPORTED )
#set_target_properties(lib_libtiff PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/liblibtiff.a)

#add_library( lib_libwebp STATIC IMPORTED )
#set_target_properties(lib_libwebp PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/liblibwebp.a)

add_library( opencv SHARED IMPORTED )
set_target_properties(opencv PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_java3.so)

#add_library(OpenCL SHARED IMPORTED)
#set_target_properties(OpenCL PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libOpenCL.so)

add_library(lib-ceres STATIC IMPORTED)
set_target_properties(lib-ceres PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libceres.a)

add_library(lib-boost-filesystem STATIC IMPORTED)
set_target_properties(lib-boost-filesystem PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_filesystem-gcc-mt-1_53.a)

add_library(lib-boost-program-options STATIC IMPORTED)
set_target_properties(lib-boost-program-options PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_program_options-gcc-mt-1_53.a)

add_library(lib-boost-system STATIC IMPORTED)
set_target_properties(lib-boost-system PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_system-gcc-mt-1_53.a)

#C++日志
find_library( log-lib log )
find_library( android-lib android )

#QSML
#add_library( libqsml SHARED IMPORTED )
#sig
#set_target_properties(libqsml PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libQSML-sequential-0.15.5.so)
#mul
#set_target_properties(libqsml PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libQSML-0.15.5.so)


# =================================================================
ADD_DEFINITIONS(-DCERES_USE_OPENMP -DNDEBUG )#CERES_USE_OPENMP/CERES_NO_THREADS
#ADD_DEFINITIONS(-D_OPENMP)
#ADD_DEFINITIONS(-DEIGEN_USE_BLAS)

set(vins_SRCS   src/main/cpp/native-lib.cpp
                src/main/cpp/src/vins_estimator/src/estimator_node.cpp
                src/main/cpp/src/vins_estimator/src/parameters.cpp
                src/main/cpp/src/vins_estimator/src/estimator.cpp
                src/main/cpp/src/vins_estimator/src/feature_manager.cpp
                src/main/cpp/src/vins_estimator/src/factor/pose_local_parameterization.cpp
                src/main/cpp/src/vins_estimator/src/factor/projection_factor.cpp
                src/main/cpp/src/vins_estimator/src/factor/marginalization_factor.cpp
                src/main/cpp/src/vins_estimator/src/utility/utility.cpp
                src/main/cpp/src/vins_estimator/src/utility/visualization.cpp
                src/main/cpp/src/vins_estimator/src/initial/solve_5pts.cpp
                src/main/cpp/src/vins_estimator/src/initial/initial_aligment.cpp
                src/main/cpp/src/vins_estimator/src/initial/initial_sfm.cpp
                src/main/cpp/src/vins_estimator/src/initial/initial_ex_rotation.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DBoW/BowVector.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DBoW/FBrief.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DBoW/FeatureVector.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DBoW/QueryResults.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DBoW/ScoringObject.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DUtils/Random.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DUtils/Timestamp.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/DVision/BRIEF.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/ThirdParty/VocabularyBinary.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/loop_closure.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/keyframe.cpp
                src/main/cpp/src/vins_estimator/src/loop-closure/keyframe_database.cpp
                src/main/cpp/src/vins_estimator/src/feature_tracker/feature_tracker.cpp

                src/main/cpp/src/camera_model/src/chessboard/Chessboard.cc
                src/main/cpp/src/camera_model/src/calib/CameraCalibration.cc
                src/main/cpp/src/camera_model/src/camera_models/Camera.cc
                src/main/cpp/src/camera_model/src/camera_models/CameraFactory.cc
                src/main/cpp/src/camera_model/src/camera_models/CostFunctionFactory.cc
                src/main/cpp/src/camera_model/src/camera_models/PinholeCamera.cc
                src/main/cpp/src/camera_model/src/camera_models/CataCamera.cc
                src/main/cpp/src/camera_model/src/camera_models/EquidistantCamera.cc
                src/main/cpp/src/camera_model/src/camera_models/ScaramuzzaCamera.cc
                src/main/cpp/src/camera_model/src/sparse_graph/Transform.cc
                src/main/cpp/src/camera_model/src/gpl/gpl.cc
                src/main/cpp/src/camera_model/src/gpl/EigenQuaternionParameterization.cc

                src/main/cpp/src/a2ir/log_util.cpp
                src/main/cpp/src/a2ir/phone_sensor.cpp
                src/main/cpp/src/a2ir/timer.cpp

                src/main/cpp/src/vins_estimator/src/draw_result/delaunay.cpp
                src/main/cpp/src/vins_estimator/src/draw_result/draw_result.cpp
                src/main/cpp/src/vins_estimator/src/draw_result/triangle.cpp)

set_property(SOURCE ${vins_SRCS}
               APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 -fomit-frame-pointer -ffast-math")

# =================================================================

add_library( native-lib SHARED ${vins_SRCS})
target_link_libraries( native-lib
                       opencv
                       lib_opencv_calib3d
                       lib_opencv_core
                       lib_opencv_features2d
                       lib_opencv_flann
                       lib_opencv_highgui
                       lib_opencv_imgcodecs
                       lib_opencv_imgproc
                       lib_opencv_ml
                       lib_opencv_objdetect
                       lib_opencv_photo
                       lib_opencv_shape
                       lib_opencv_stitching
                       lib_opencv_superres
                       lib_opencv_ts
                       lib_opencv_video
                       lib_opencv_videoio
                       lib_opencv_videostab

                       lib-ceres
                       lib-boost-filesystem
                       lib-boost-program-options
                       lib-boost-system

                       ${android-lib}
                       ${log-lib}

                       gomp
                       #libqsml
                       )