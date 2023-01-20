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

# add_executable(delta_array delta_array.cpp ../tasks/inhand_task.cpp ../mechanics/manipulators/DartDeltaManipulator.cpp)
# target_link_libraries(delta_array PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})
add_executable(ddhand_simple ddhand_simple.cpp ../tasks/cmg_task.cpp ../mechanics/manipulators/DartDDHand.cpp ../mechanics/manipulators/ddhand/two_link_kinematics.cpp)
target_link_libraries(ddhand_simple PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})