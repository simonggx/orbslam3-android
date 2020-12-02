cmake_minimum_required(VERSION 2.8)
project(DBoW2_Tools)

set(HDRS_DBOW2
  ../DBoW2/BowVector.h
  ../DBoW2/FORB.h 
  ../DBoW2/FClass.h       
  ../DBoW2/FeatureVector.h
  ../DBoW2/ScoringObject.h   
  ../DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  ../DBoW2/BowVector.cpp
  ../DBoW2/FORB.cpp      
  ../DBoW2/FeatureVector.cpp
  ../DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ../DUtils/Random.h
  ../DUtils/Timestamp.h)
set(SRCS_DUTILS
  ../DUtils/Random.cpp
  ../DUtils/Timestamp.cpp)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../DBoW2)
include_directories(../DUtils)
add_executable(DBoW2_Tools ${SRCS_DBOW2} ${SRCS_DUTILS} ${HDRS_DBOW2} ${HDRS_DUTILS} main.cc)
target_link_libraries(DBoW2_Tools ${OpenCV_LIBS})

