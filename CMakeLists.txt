cmake_minimum_required(VERSION 3.16)
project(bt_Controller)
include(cmake/CPM.cmake)
set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_BINARY_DIR}/_deps/behaviortree.cpp-src/include
        include
        )
#add_subdirectory(third_party/BehaviorTree.CPP)
CPMAddPackage("gh:BehaviorTree/BehaviorTree.CPP#master")

add_library(COM include/COM.h include/COM.cpp)
set_source_files_properties(COM.cpp PROPERTIES COMPILE_FLAGS "-include include/COM.h")
add_executable(bt_Controller main.cpp include/bt_controller/gen_waypoints.h include/pid.h src/pid.cpp include/bt_controller/position_controller.h include/bt_controller/orientation_controller.h)
target_link_libraries(${PROJECT_NAME}
        behaviortree_cpp
        COM
        )

