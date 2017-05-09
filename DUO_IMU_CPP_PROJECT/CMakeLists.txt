cmake_minimum_required(VERSION 2.8)
set(ProjectName DUO_IMU)
project(${ProjectName})

file(GLOB_RECURSE SRC "src/*.cpp"  "*.c")
file(GLOB_RECURSE HEADER "header/*.h")


find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  add_definitions(${EIGEN3_DEFINITIONS} )
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
header
${OpenCV_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/lib/DUO3D/include
)
set(
LIBS
${PROJECT_SOURCE_DIR}/lib/DUO3D/libs/DUOLib.lib
#${PROJECT_SOURCE_DIR}/lib/OpenCV/libs/opencv_world320d.lib
)

set(
DUO3D_DLL
${PROJECT_SOURCE_DIR}/lib/DUO3D/libs/DUOLib.dll
)


if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

add_executable(${ProjectName} ${SRC} ${HEADER})
target_link_libraries(${ProjectName} ${LIBS} ${OpenCV_LIBS} ${Eigen3_LIBS})

add_custom_command(
  TARGET ${ProjectName}
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${DUO3D_DLL}   "${PROJECT_BINARY_DIR}/Debug"
)
