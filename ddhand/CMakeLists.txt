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

add_executable(ddhand_dex_insert ddhand_dex_insert.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_dex_insert PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(ddhand_flip_pinch ddhand_flip_pinch.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_flip_pinch PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(ddhand_peg_out_hole ddhand_peg_out_hole.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_peg_out_hole PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(ddhand_push_lever ddhand_push_lever.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_push_lever PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})


add_executable(ddhand_peg_in_hole_side ddhand_peg_in_hole_side.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_peg_in_hole_side PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})

add_executable(ddhand_peg_in_hole_up ddhand_peg_in_hole_up.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHandScalable.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_peg_in_hole_up PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})