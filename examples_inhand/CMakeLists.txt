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

add_executable(inhand_simplest inhand_simplest.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_simplest PRIVATE mcts-mechanics ${dart_libs})

add_executable(inhand_pen inhand_pen.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_pen PRIVATE mcts-mechanics ${dart_libs})

add_executable(inhand_3finger_palm inhand_3finger_palm.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_3finger_palm PRIVATE mcts-mechanics ${dart_libs})

add_executable(compare_2nd_level compare_2nd_level.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(compare_2nd_level PRIVATE mcts-mechanics ${dart_libs})

add_executable(inhand_all inhand_all.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_all PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(inhand_goal_finger inhand_goal_finger_locations.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_goal_finger PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(inhand_flange inhand_flange.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(inhand_flange PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})
