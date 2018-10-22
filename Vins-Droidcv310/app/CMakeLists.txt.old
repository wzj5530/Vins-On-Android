# Sets the minimum version of CMake required to build the native
# library. You should either keep the default value or only pass a
# value of 3.4.0 or lower.

cmake_minimum_required(VERSION 3.4.1)


# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds it for you.
# Gradle automatically packages shared libraries with your APK.

set(CMAKE_VERBOSE_MAKEFILE on)
set(libs "${CMAKE_SOURCE_DIR}/src/main/jniLibs")
include_directories(${CMAKE_SOURCE_DIR}/src/main/cpp/include)

#--------------------------------------------------- import ---------------------------------------------------#
add_library(libopencv_java3 SHARED IMPORTED )
set_target_properties(libopencv_java3 PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_java3.so")

add_library(libopencv_java SHARED IMPORTED )
set_target_properties(libopencv_java PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_java.so")

#add_library(libopencv_calib3d STATIC IMPORTED )
#set_target_properties(libopencv_calib3d PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_calib3d.a")
#
#add_library(libopencv_core STATIC IMPORTED )
#set_target_properties(libopencv_core PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_core.a")
#
#add_library(libopencv_dnn STATIC IMPORTED )
#set_target_properties(libopencv_core PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_dnn.a")
#
#add_library(libopencv_features2d STATIC IMPORTED )
#set_target_properties(libopencv_features2d PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_features2d.a")
#
#add_library(libopencv_flann STATIC IMPORTED )
#set_target_properties(libopencv_flann PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_flann.a")
#
#add_library(libopencv_highgui STATIC IMPORTED )
#set_target_properties(libopencv_highgui PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_highgui.a")
#
#add_library(libopencv_imgcodecs STATIC IMPORTED )
#set_target_properties(libopencv_imgcodecs PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_imgcodecs.a")
#
#add_library(libopencv_imgproc STATIC IMPORTED )
#set_target_properties(libopencv_imgproc PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_imgproc.a")
#
#add_library(libopencv_ml STATIC IMPORTED )
#set_target_properties(libopencv_ml PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_ml.a")
#
#add_library(libopencv_objdetect STATIC IMPORTED )
#set_target_properties(libopencv_objdetect PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_objdetect.a")
#
#add_library(libopencv_photo STATIC IMPORTED )
#set_target_properties(libopencv_photo PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_photo.a")
#
#add_library(libopencv_shape STATIC IMPORTED )
#set_target_properties(libopencv_shape PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_shape.a")
#
#add_library(libopencv_stitching STATIC IMPORTED )
#set_target_properties(libopencv_stitching PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_stitching.a")
#
#add_library(libopencv_superres STATIC IMPORTED )
#set_target_properties(libopencv_superres PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_superres.a")
#
#add_library(libopencv_video STATIC IMPORTED )
#set_target_properties(libopencv_video PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_video.a")
#
#add_library(libopencv_videoio STATIC IMPORTED )
#set_target_properties(libopencv_videoio PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_videoio.a")
#
#add_library(libopencv_videostab STATIC IMPORTED )
#set_target_properties(libopencv_videostab PROPERTIES
#    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_videostab.a")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fexceptions -frtti")

add_library( # Sets the name of the library.
             native-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/native-lib.cpp )

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log)

target_link_libraries(native-lib android log
    libopencv_java3 #used for java sdk
    libopencv_java #same to static libs
    #17 static libs in total
    #libopencv_calib3d
    #libopencv_core
    #libopencv_dnn
    #libopencv_features2d
    #libopencv_flann
    #libopencv_highgui
    #libopencv_imgcodecs
    #libopencv_imgproc
    #libopencv_ml
    #libopencv_objdetect
    #libopencv_photo
    #libopencv_shape
    #libopencv_stitching
    #libopencv_superres
    #libopencv_video
    #libopencv_videoio
    #libopencv_videostab
    ${log-lib}
    )

