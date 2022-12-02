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
# test_csv.cpp
# ${CMAKE_SOURCE_DIR}/mechanics/utilities/parser.hpp)
ADD_EXECUTABLE(test_force test_force.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(test_force PRIVATE mcts-mechanics ${dart_libs})

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

add_executable(cmg_bookshelf cmg_bookshelf.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_bookshelf PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_peg cmg_peg.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_peg PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_wall cmg_wall.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_wall PRIVATE mcts-mechanics ${dart_libs})

add_executable(cmg_stairs cmg_stairs.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(cmg_stairs PRIVATE mcts-mechanics ${dart_libs})

add_executable(test_action_idx test_action_idx.cpp)
target_link_libraries(test_action_idx PRIVATE mcts-mechanics)

add_executable(test_time test_time.cpp)

add_executable(inhand_simplest inhand_simplest.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_simplest PRIVATE mcts-mechanics ${dart_libs})

add_executable(inhand_pen inhand_pen.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_pen PRIVATE mcts-mechanics ${dart_libs})