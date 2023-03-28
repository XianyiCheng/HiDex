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

add_executable(action_integrator action_integrator.cpp ../tasks/task.cpp ../mechanics/manipulators/DartPointManipulator.cpp)
target_link_libraries(action_integrator PRIVATE mcts-mechanics ${dart_libs} ${YAML_LIBRARY})