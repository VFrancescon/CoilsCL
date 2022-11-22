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
find_library( ASTAR a-star)

## S3: Set variables for library linkage. Use one of the below only
set(MiddlewareLibs ${MID} ${PSU} ${LINACT} ${TMETER} ${SERIAL} Threads::Threads)
set(PSULibs ${PSU} ${SERIAL})
set(LinActLibs ${LINACT} ${SERIAL})
set(TmeterLibs ${TMETER} ${SERIAL})
set(VisionLibs ${Pylon_LIBRARIES} ${OpenCV_LIBS} ${ASTAR})
set(EIGEN Eigen3::Eigen)
set(THREADS Threads::Threads)

## S4: Set include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/opt/pylon/include)


## Add your source files here to executables
add_executable(main "src/main.cpp")
add_executable(field3D "src/set3DField.cpp")
add_executable(fakeGradients "src/fakeGradients.cpp")
add_executable(MIDsweep "src/sweep.cpp")
add_executable(PSUsweep "src/sweepPSU.cpp")
add_executable(CAMsweep "src/sweepNsave.cpp")
add_executable(precomp "src/precomputations.cpp")
add_executable(run_camera "src/run_camera")
add_executable(angles "src/anglededuction.cpp")
add_executable(LIVEangles "src/angledeductionLIVE.cpp")
add_executable(csvSweep "src/csvSweep.cpp")
add_executable(csvSweepM "src/csvSweepMULTI.cpp")
add_executable(controllerPrototype "src/ControllerPrototype.cpp")
add_executable( th_cal "src/thresholds_cal.cpp")
add_executable( demo_0111 "src/csv_Step_Introducer.cpp")
add_executable( PolarityTests "src/PolarityTests.cpp")


## Link the libraries frim S3 here
target_link_libraries( main ${MiddlewareLibs})
target_link_libraries(field3D ${MiddlewareLibs})
target_link_libraries(fakeGradients ${PSULibs})
target_link_libraries( MIDsweep ${MiddlewareLibs})
target_link_libraries( PSUsweep ${PSULibs})
target_link_libraries( CAMsweep ${MiddlewareLibs} ${VisionLibs})
target_link_libraries( precomp ${EIGEN})
target_link_libraries( run_camera ${EIGEN} ${VisionLibs})
target_link_libraries( angles ${EIGEN} ${VisionLibs})
target_link_libraries( LIVEangles ${EIGEN} ${VisionLibs})
target_link_libraries( csvSweep ${VisionLibs} ${MiddlewareLibs})
target_link_libraries( csvSweepM ${VisionLibs} ${MiddlewareLibs})
target_link_libraries( controllerPrototype ${EIGEN} ${VisionLibs} ${MiddlewareLibs})
target_link_libraries( th_cal ${VisionLibs})
target_link_libraries( demo_0111 ${VisionLibs} ${MiddlewareLibs} )
target_link_libraries( PolarityTests ${PSU} ${TMETER} ${SERIAL} Threads::Threads)
