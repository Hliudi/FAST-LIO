cmake_minimum_required(VERSION 3.5)
project(fast_lio)

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

if(YAMLCPP_STATIC_LIBRARY)
    set(YAMLCPP_STATIC libyaml-cpp.a)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

option(FASTLIO_ENABLE_PLOTTING "Enable matplotlib plotting" OFF)
if(FASTLIO_ENABLE_PLOTTING)
  find_package(PythonLibs REQUIRED)
  find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include)

if(FASTLIO_ENABLE_PLOTTING)
  include_directories(${PYTHON_INCLUDE_DIRS})
endif()
  
add_executable(fastlio_mapping src/main.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp nlohmann_json::nlohmann_json)

if(FASTLIO_ENABLE_PLOTTING)
  target_link_libraries(fastlio_mapping ${PYTHON_LIBRARIES})
  target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()