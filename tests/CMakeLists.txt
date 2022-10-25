# find_library(GLPKLIB glpk REQUIRED)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)
set(dart_libs dart dart-utils-urdf dart-gui dart-collision-bullet)

# ADD_EXECUTABLE(test_csv 
# 	               test_csv.cpp
# 	               ${CMAKE_SOURCE_DIR}/mechanics/utilities/parser.hpp)

ADD_EXECUTABLE(test_force 
	               test_force.cpp)
target_link_libraries(test_force PRIVATE mcts-mechanics)

# target_include_directories(test_force PRIVATE "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3")

# add_executable(test_level1 test_level1.cpp)

# target_include_directories(test_level1 PRIVATE ${external_dirs})
# target_link_libraries(test_level1 PRIVATE mcts-mechanics)

# add_executable(test_cmg test_level1_cmg.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)

# # target_include_directories(test_cmg PUBLIC ${example_include_dirs})
# # target_include_directories(test_cmg PRIVATE ${example_include_dirs})
# # target_link_libraries(test_cmg PRIVATE mcts-mechanics ${GLPKLIB} ${dart_libs} ${modus_libs})
# # target_link_libraries(test_cmg PRIVATE mcts-mechanics ${dart_libs} ${modus_libs})
# target_link_libraries(test_cmg PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_pushing cmg_pushing.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_pushing PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_cube_reorient90 cmg_cube_reorient90.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_cube_reorient90 PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_pick_card cmg_pick_card.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_pick_card PRIVATE mcts-mechanics ${dart_libs})

add_executable(test_hand_collision test_hand_collision.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(test_hand_collision PRIVATE mcts-mechanics ${dart_libs})