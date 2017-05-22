cmake_minimum_required(VERSION 3.0)
project( MadgwickDllLib)

file(GLOB sources "src/*.cpp")
file(GLOB headers "header/*.h")

include_directories(header)

add_library(${PROJECT_NAME}  SHARED ${sources} )

find_package(Eigen3 REQUIRED NO_MODULE)


target_link_libraries(${PROJECT_NAME} Eigen3::Eigen )


install(TARGETS ${PROJECT_NAME} DESTINATION "${CMAKE_SOURCE_DIR}/../Assets/plugins")

#install(FILES ${headers} DESTINATION  "${CMAKE_SOURCE_DIR}/../Assets/plugins/include")

set(CMAKE_CXX_FLAGS /bigobj)

