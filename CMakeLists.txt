cmake_minimum_required(VERSION 2.8.3)
project(3d_model_conversion)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(src SRC_FILES)
add_executable(${PROJECT_NAME} ${SRC_FILES})

add_definitions(-std=c++11)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES})
