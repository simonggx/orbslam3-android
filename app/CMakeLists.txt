cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -Wall  -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall   -O3")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

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

if(${ANDROID_ABI} STREQUAL "armeabi-v7a")
    include_directories(${ANDROID_SYSROOT}/usr/include/arm-linux-androideabi)
elseif(${ANDROID_ABI} STREQUAL "arm64-v8a")
    include_directories(${ANDROID_SYSROOT}/usr/include/aarch64-linux-android)
else()
    include_directories(${ANDROID_SYSROOT}/usr/include/arm-linux-androideabi)
endif()

set(OpenCV_DIR C:/android_project/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

#set(BOOST_ROOT C:/android_project/boost_1_49_0)
#set(Boost_INCLUDE_DIR C:/android_project/boost_1_49_0)
#set(Boost_LIBRARY_DIR C:/android_project/boost_1_49_0/stage/lib)
#find_package(Boost 1.49.0 COPONENTS system filesystem thread REQUIRED)
#if(NOT Boost_FOUND)
#    message(FATAL_ERROR "Boost > 1.49.0 not found.")
#endif()
#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#link_libraries(${Boost_LIBRARY_DIR})

#set(EIGEN3_INCLUDE_DIR D:/orb_slam2/app/src/main/cpp)
#find_package(Eigen3 3.1.0 REQUIRED)

SET( EIGEN3_INCLUDE_DIR C:/android_project/eigen-3.3.4 )
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${EIGEN3_INCLUDE_DIR}/cmake" )
FIND_PACKAGE(Eigen3 3.3.4 REQUIRED)
MESSAGE("Found Eigen ${EIGEN3_VERSION}")
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${EIGEN3_INCLUDE_DIR}
#Thirdparty/Boost/include/boost-1_49
${PROJECT_SOURCE_DIR}/Thirdparty/serialization
#/home/zhufan/Android/Sdk/ndk-bundle/sysroot/usr/include
    C:/android_project/boost_1_49_0
    C:/android_project/boost_1_49_0/boost

	C:/android_project/ceres-solver-1.12.0/include
    C:/android_project/ceres-solver-1.12.0/config
    C:/android_project/ceres-solver-1.12.0/internal
    C:/android_project/ceres-solver-1.12.0/internal/ceres
    C:/android_project/ceres-solver-1.12.0/internal/ceres/miniglog
    C:/android_project/OpenCV-android-sdk/sdk/native/jni/include
)

ADD_LIBRARY(g2o
       SHARED

        Thirdparty/g2o/g2o/types/types_sba.h
        Thirdparty/g2o/g2o/types/types_six_dof_expmap.h
        Thirdparty/g2o/g2o/types/types_sba.cpp
        Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp

        Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
        Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h
        Thirdparty/g2o/g2o/types/se3quat.h
        Thirdparty/g2o/g2o/types/se3_ops.h
        Thirdparty/g2o/g2o/types/se3_ops.hpp
        Thirdparty/g2o/g2o/core/base_edge.h
        Thirdparty/g2o/g2o/core/base_binary_edge.h
        Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
        Thirdparty/g2o/g2o/core/base_binary_edge.hpp
        Thirdparty/g2o/g2o/core/hyper_graph_action.h
        Thirdparty/g2o/g2o/core/base_multi_edge.h
        Thirdparty/g2o/g2o/core/hyper_graph.cpp
        Thirdparty/g2o/g2o/core/base_multi_edge.hpp
        Thirdparty/g2o/g2o/core/hyper_graph.h
        Thirdparty/g2o/g2o/core/base_unary_edge.h
        Thirdparty/g2o/g2o/core/linear_solver.h
        Thirdparty/g2o/g2o/core/base_unary_edge.hpp
        Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
        Thirdparty/g2o/g2o/core/base_vertex.h
        Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h
        Thirdparty/g2o/g2o/core/base_vertex.hpp
        Thirdparty/g2o/g2o/core/matrix_structure.cpp
        Thirdparty/g2o/g2o/core/batch_stats.cpp
        Thirdparty/g2o/g2o/core/matrix_structure.h
        Thirdparty/g2o/g2o/core/batch_stats.h
        Thirdparty/g2o/g2o/core/openmp_mutex.h
        Thirdparty/g2o/g2o/core/block_solver.h
        Thirdparty/g2o/g2o/core/block_solver.hpp
        Thirdparty/g2o/g2o/core/parameter.cpp
        Thirdparty/g2o/g2o/core/parameter.h
        Thirdparty/g2o/g2o/core/cache.cpp
        Thirdparty/g2o/g2o/core/cache.h
        Thirdparty/g2o/g2o/core/optimizable_graph.cpp
        Thirdparty/g2o/g2o/core/optimizable_graph.h
        Thirdparty/g2o/g2o/core/solver.cpp
        Thirdparty/g2o/g2o/core/solver.h
        Thirdparty/g2o/g2o/core/creators.h
        Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
        Thirdparty/g2o/g2o/core/estimate_propagator.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h
        Thirdparty/g2o/g2o/core/estimate_propagator.h
        Thirdparty/g2o/g2o/core/factory.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm_property.h
        Thirdparty/g2o/g2o/core/factory.h
        Thirdparty/g2o/g2o/core/sparse_block_matrix.h
        Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
        Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp
        Thirdparty/g2o/g2o/core/sparse_optimizer.h
        Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
        Thirdparty/g2o/g2o/core/hyper_dijkstra.h
        Thirdparty/g2o/g2o/core/parameter_container.cpp
        Thirdparty/g2o/g2o/core/parameter_container.h
        Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm.h
        Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h
        Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h
        Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
        Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h
        Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
        Thirdparty/g2o/g2o/core/jacobian_workspace.h
        Thirdparty/g2o/g2o/core/robust_kernel.cpp
        Thirdparty/g2o/g2o/core/robust_kernel.h
        Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
        Thirdparty/g2o/g2o/core/robust_kernel_factory.h
        Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
        Thirdparty/g2o/g2o/core/robust_kernel_impl.h
        Thirdparty/g2o/g2o/stuff/string_tools.h
        Thirdparty/g2o/g2o/stuff/color_macros.h
        Thirdparty/g2o/g2o/stuff/macros.h
        Thirdparty/g2o/g2o/stuff/timeutil.cpp
        Thirdparty/g2o/g2o/stuff/misc.h
        Thirdparty/g2o/g2o/stuff/timeutil.h
        Thirdparty/g2o/g2o/stuff/os_specific.c
        Thirdparty/g2o/g2o/stuff/os_specific.h
        Thirdparty/g2o/g2o/stuff/string_tools.cpp
        Thirdparty/g2o/g2o/stuff/property.cpp
        Thirdparty/g2o/g2o/stuff/property.h
        )

add_library(DBoW2
            SHARED

            Thirdparty/DBoW2/include/DBoW2/BowVector.h
            Thirdparty/DBoW2/include/DBoW2/FORB.h
            Thirdparty/DBoW2/include/DBoW2/FClass.h
            Thirdparty/DBoW2/include/DBoW2/FeatureVector.h
            Thirdparty/DBoW2/include/DBoW2/ScoringObject.h
            Thirdparty/DBoW2/include/DBoW2/TemplatedVocabulary.h
            Thirdparty/DBoW2/DLib/include/DUtils/Random.h
            Thirdparty/DBoW2/DLib/include/DUtils/Timestamp.h

            Thirdparty/DBoW2/src/BowVector.cpp
            Thirdparty/DBoW2/src/FORB.cpp
            Thirdparty/DBoW2/src/FeatureVector.cpp
            Thirdparty/DBoW2/src/ScoringObject.cpp
            Thirdparty/DBoW2/DLib/src/DUtils/Random.cpp
            Thirdparty/DBoW2/DLib/src/DUtils/Timestamp.cpp
            )
target_link_libraries(DBoW2 ${OpenCV_LIBS})

add_definitions(-D__ANDROID__)

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
        src/Atlas.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/OptimizableTypes.cpp
        src/MLPnPsolver.cpp
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Atlas.h
        include/Map.h
        include/MapDrawer.h
        include/Optimizer.h
        include/PnPsolver.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Initializer.h
        include/Viewer.h
        include/ImuTypes.h
        include/G2oTypes.h
        include/CameraModels/GeometricCamera.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/OptimizableTypes.h
        include/MLPnPsolver.h
        include/TwoViewReconstruction.h
        src/TwoViewReconstruction.cc
        include/SlamJni.h
        include/Log.h
        include/Utility.h
        src/SlamJni.cc
        src/Log.cc
        src/Utility.cc

)

#add_library(boost_serialization-gcc-mt-1_49 STATIC IMPORTED)
#set_target_properties(boost_serialization-gcc-mt-1_49
#  PROPERTIES IMPORTED_LOCATION
#  C:/Users/renye/Desktop/orb_slam2/app/src/jniLibs/armeabi-v7a/libboost_serialization-gcc-mt-1_49.a)

add_library(libceres STATIC IMPORTED)
set_target_properties(libceres
    PROPERTIES IMPORTED_LOCATION
    C:/android_project/ceres-solver-1.12.0/obj/local/arm64-v8a/libceres.a)

add_library(boost_system STATIC IMPORTED)
set_target_properties(boost_system
        PROPERTIES IMPORTED_LOCATION
        C:/android_project/boost_1_49_0/stage/lib/libboost_system.a)

add_library(boost_serialization STATIC IMPORTED)
set_target_properties(boost_serialization
        PROPERTIES IMPORTED_LOCATION
        C:/android_project/boost_1_49_0/stage/lib/libboost_serialization.a)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
#${EIGEN3_LIBS}
#${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
#${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
DBoW2
g2o
libceres
boost_system
boost_serialization
#boost_serialization-gcc-mt-1_49
${PROJECT_SOURCE_DIR}/Thirdparty/serialization/libserialization.a
)

