# find_library(GLPKLIB glpk REQUIRED)
find_library(YAML_LIBRARY yaml-cpp HINTS /usr/local/lib)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    # "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)
# set(dart_libs dart dart-utils-urdf dart-gui dart-collision-bullet)

# ADD_EXECUTABLE(test_csv
# test_csv.cpp
# ${CMAKE_SOURCE_DIR}/mechanics/utilities/parser.hpp)

# ADD_EXECUTABLE(test_force test_force.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(test_force PRIVATE mcts-mechanics ${dart_libs})

# target_include_directories(test_force PRIVATE "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3")

# add_executable(test_level1 test_level1.cpp)

# target_include_directories(test_level1 PRIVATE ${external_dirs})
# target_link_libraries(test_level1 PRIVATE mcts-mechanics)

# add_executable(test_cmg test_level1_cmg.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)

# # target_include_directories(test_cmg PUBLIC ${example_include_dirs})
# # target_include_directories(test_cmg PRIVATE ${example_include_dirs})
# # target_link_libraries(test_cmg PRIVATE mcts-mechanics ${GLPKLIB} ${dart_libs} ${modus_libs})
# # target_link_libraries(test_cmg PRIVATE mcts-mechanics ${dart_libs} ${modus_libs})
# target_link_libraries(test_cmg PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_pushing cmg_pushing.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_pushing PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_cube_reorient90 cmg_cube_reorient90.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_cube_reorient90 PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_pick_card cmg_pick_card.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_pick_card PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_bookshelf cmg_bookshelf.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_bookshelf PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_peg cmg_peg.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_peg PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_wall cmg_wall.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_wall PRIVATE mcts-mechanics ${dart_libs})

# add_executable(cmg_stairs cmg_stairs.cpp ../tasks/cmg_task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(cmg_stairs PRIVATE mcts-mechanics ${dart_libs})

# add_executable(test_action_idx test_action_idx.cpp)
# target_link_libraries(test_action_idx PRIVATE mcts-mechanics)

# add_executable(test_time test_time.cpp)
# add_executable(test_rewards test_rewards.cpp)
# add_test(NAME cxy_test COMMAND test_time test_action_idx)
# add_test(NAME cxy_test_time COMMAND test_time)
# add_test(NAME cxy_test_action COMMAND test_action_idx)
# add_test(NAME cxy_test_sample COMMAND test_discrete_sample)
# add_test(NAME cxy_test_reward COMMAND test_rewards)
# # in the build folder, run "ctest -R cxy_test" to see the output


# add_executable(test_discrete_sample test_discrete_sample.cpp)

add_executable(test_quasidynamic test_quasidynamic.cpp ../src/tasks/task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(test_quasidynamic PRIVATE mcts-mechanics ${dart_libs})

# add_executable(test_task_class test_task_class.cpp ../tasks/task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(test_task_class PRIVATE mcts-mechanics ${dart_libs})

# add_executable(test_task_inhand test_task_inhand.cpp ../tasks/task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
# target_link_libraries(test_task_inhand PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(test_exp_map test_exp_map.cpp 
    ../src/mechanics/manipulators/DartWholeHand.cpp
    ../src/mechanics/manipulators/wholehand/optimizer.cpp
    ../src/mechanics/manipulators/wholehand/sdf.cpp
    ../src/tasks/wholehand_task.cpp 
    ../src/tasks/wholehand_setup.cpp
    ../src/mechanics/manipulators/wholehand/exp_map.cpp)
target_link_libraries(test_exp_map PRIVATE mcts-mechanics ${dart_libs} dart-optimizer-nlopt ${YAML_LIBRARY} geometry-central)

set(m_files ../src/tasks/task.cpp 
../src/tasks/setup.cpp
../src/mechanics/manipulators/DartPointManipulator.cpp 
../src/mechanics/manipulators/DartDeltaManipulator.cpp
../src/mechanics/manipulators/DartDDHandScalable.cpp
../src/mechanics/manipulators/ddhand/two_link_kinematics.cpp)

add_executable(test_forward_integration test_forward_integration.cpp ${m_files} )

target_link_libraries(test_forward_integration PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(test_path_join test_path_join.cpp ${m_files})

target_link_libraries(test_path_join PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})