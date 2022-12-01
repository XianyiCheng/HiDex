add_subdirectory(external)

find_library(GLPKLIB glpk REQUIRED)

set(include_dirs
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
    "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
)

set(modus_libs modus_common modus_modes)
set(dart_libs dart dart-utils-urdf dart-gui dart-collision-bullet)

add_library(mcts-mechanics
    mechanics.cpp
    contacts/contact_kinematics.cpp
    contacts/contact_constraints.cpp
    contacts/contact_mode_enumeration.cpp
    utilities/sample.cpp
    utilities/utilities.cpp
    utilities/io.cpp
    utilities/combinatorics.cpp
    utilities/miscellaneous.cpp
    worlds/DartWorld.cpp
    dart_utils/dart_utils.cpp
    )

target_include_directories(mcts-mechanics PUBLIC ${include_dirs})
target_link_libraries(mcts-mechanics PRIVATE ${GLPKLIB} ${dart_libs} ${modus_libs})
