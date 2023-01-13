# find_library(GLPKLIB glpk REQUIRED)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)
set(dart_libs dart dart-utils-urdf dart-gui dart-collision-bullet)

add_executable(env_pick_card pick_card.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(env_pick_card PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(env_bookshelf bookshelf.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(env_bookshelf PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(env_reorient reorient.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(env_reorient PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(env_peg peg.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(env_peg PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})
