# find_library(GLPKLIB glpk REQUIRED)
find_library(YAML_LIBRARY yaml-cpp HINTS /usr/local/lib)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    # "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)

add_executable(test_quasidynamic test_quasidynamic.cpp ../src/tasks/task.cpp ../src/mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(test_quasidynamic PRIVATE mcts-mechanics ${dart_libs})

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