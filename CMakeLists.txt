cmake_minimum_required(VERSION 3.1.0)
project(blank_coils_proj) ##EDIT THIS WITH YOUR PROJECT NAME

## S0: Some boilerplate
add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## S1: Finding prebuilt packages
find_package( OpenCV REQUIRED )
find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

## S2: Finding custom libraries
find_library(MID HCoilMiddlewareLib )
find_library(PSU  DxkdpLib )
find_library(LINACT LinActuatorLib )
find_library(TMETER LakeshoreF71Lib )
find_library(SERIAL CppLinuxSerial )

## S3: Set variables for library linkage. Use one of the below only
set(MiddlewareLibs ${MID} ${PSU} ${LINACT} ${TMETER} ${SERIAL} Threads::Threads)
set(PSULibs ${PSU} ${SERIAL})
set(LinActLibs ${LINACT} ${SERIAL})
set(TmeterLibs ${TMETER} ${SERIAL})
set(VisionLibs ${Pylon_LIBRARIES} ${OpenCV_LIBS})
set(EIGEN Eigen3::Eigen)
set(THREADS Threads::Threads)

## S4: Set include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/opt/pylon/include)


## Add your source files here to executables
add_executable(main "src/main.cpp")
add_executable(field3D "src/set3DField.cpp")
add_executable(fakeGradients "src/fakeGradients.cpp")

## Link the libraries frim S3 here
target_link_libraries( main ${MiddlewareLibs})
target_link_libraries(field3D ${MiddlewareLibs})
target_link_libraries(fakeGradients ${PSULibs})