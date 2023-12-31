# find_library(GLPKLIB glpk REQUIRED)
find_library(YAML_LIBRARY yaml-cpp HINTS /usr/local/lib)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)
set(dart_libs dart dart-utils-urdf dart-gui dart-collision-bullet)

set(m_files ../src/tasks/task.cpp 
../src/tasks/setup.cpp
../src/mechanics/manipulators/DartPointManipulator.cpp 
../src/mechanics/manipulators/DartDeltaManipulator.cpp
../src/mechanics/manipulators/DartDDHandScalable.cpp
../src/mechanics/manipulators/ddhand/two_link_kinematics.cpp)

add_executable(hidex_planner hidex_planner.cpp ${m_files} )

target_link_libraries(hidex_planner PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(hidex_batch collect_data_in_batch.cpp ${m_files})

target_link_libraries(hidex_batch PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(hidex_trajectory_visualizer hidex_trajectory_visualizer.cpp ${m_files})

target_link_libraries(hidex_trajectory_visualizer PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(turn_trajectory_into_dataset turn_trajectory_into_dataset.cpp)

target_link_libraries(turn_trajectory_into_dataset PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})