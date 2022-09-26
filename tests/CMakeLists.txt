ADD_EXECUTABLE(test_csv 
	               test_csv.cpp
	               ${CMAKE_SOURCE_DIR}/mechanics/utilities/parser.hpp)

ADD_EXECUTABLE(test_force 
	               test_force.cpp)
target_link_libraries(test_force PRIVATE mcts-mechanics)

target_include_directories(test_force PRIVATE "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3")

add_executable(test_level1 test_level1.cpp)

target_include_directories(test_level1 PRIVATE ${external_dirs})
target_link_libraries(test_level1 PRIVATE mcts-mechanics)