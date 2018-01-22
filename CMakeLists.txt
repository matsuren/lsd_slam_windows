######################################################################
# The project name
cmake_minimum_required (VERSION 3.1)
get_filename_component(ProjectId ${CMAKE_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
message(Project Id : ${ProjectId})
project (${ProjectId})


######################################################################
# CMake Options
set( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake )

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)
add_definitions("-std=c++11")
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/lsd_slam)

######################################################################
# Visual Studio Options
if(WIN32)
  # Displays warnings and disables compiler warning C4996 and C4819 ...
  add_definitions (/W3 /wd4996 /wd4819)
  # Multiprocessor compilation
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} /MP")
  # Enable handle addresses larger than 2 gigabytes
  if(NOT CMAKE_CL_64)
    SET(CMAKE_EXE_LINKER_FLAGS "/LARGEADDRESSAWARE ${CMAKE_EXE_LINKER_FLAGS}")
  endif(NOT CMAKE_CL_64)
endif(WIN32)

######################################################################
# Options
option (USE_STATIC "Use static library" OFF)

# SSE flags
add_definitions("-DENABLE_SSE")

# Disable Eigen error
add_definitions("-DEIGEN_DONT_ALIGN")

######################################################################
# OpenCV
if(USE_STATIC)
  set(OpenCV_STATIC ON)
else()
  set(OpenCV_STATIC OFF)
endif(USE_STATIC)
find_package( OpenCV REQUIRED)
list( APPEND PROJ_LIBS ${OpenCV_LIBS})

######################################################################
# Eigen
# find_package(Eigen3 REQUIRED)
# include_directories( ${Eigen3_INCLUDE_DIR})
set(Eigen3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/eigen3)
message(STATUS "Eigen include directory:${Eigen3_INCLUDE_DIR}")
include_directories(${Eigen3_INCLUDE_DIR})

######################################################################
# Sophus
set(Sophus_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus)
message(STATUS "Sophus include directory:${Sophus_INCLUDE_DIR}")
include_directories(${Sophus_INCLUDE_DIR})

######################################################################
# Pangolin
find_package(Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS})
list( APPEND PROJ_LIBS ${Pangolin_LIBRARIES})

######################################################################
# g2o
set(G2O_ROOT "" CACHE PATH "G2O install directory that has lib and include directory")
find_package(G2O REQUIRED)
set(G2O_COMPONENTS G2O_CORE_LIBRARY G2O_STUFF_LIBRARY G2O_TYPES_ICP G2O_TYPES_SBA G2O_TYPES_SLAM3D G2O_SOLVER_CSPARSE G2O_SOLVER_CSPARSE_EXTENSION)
foreach(LIB ${G2O_COMPONENTS})
  LIST(APPEND G2O_LIBS optimized ${${LIB}})
  LIST(APPEND G2O_LIBS debug ${${LIB}_DEBUG})
endforeach(LIB)
message(STATUS "G2O_LIBRARIES:${G2O_LIBS}")
LIST(APPEND PROJ_LIBS ${G2O_LIBS})
include_directories(${G2O_INCLUDE_DIR})
add_definitions(-DWINDOWS)
# use g2o bundle csparse g2o_ext_csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
LIST(APPEND PROJ_LIBS ${CSPARSE_LIBRARY})

######################################################################
# dirent
set(Sophus_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/dirent-1.20.1/include)

######################################################################
# Recurse into the subdirectories. 
add_subdirectory (lsd_slam)
# add_subdirectory (apps)
