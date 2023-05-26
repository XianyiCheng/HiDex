# find_library(GLPKLIB glpk REQUIRED)
find_library(YAML_LIBRARY yaml-cpp HINTS /usr/local/lib)

# set(include_dirs
#     "${PROJECT_SOURCE_DIR}"
#     "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
#     "${PROJECT_BINARY_DIR}/mechanics/external/modus/include"
#     # "${PROJECT_BINARY_DIR}/mechanics/external/dartsim"
# )

set(modus_libs modus_common modus_modes)


add_executable(box_surface_opt box_surface_opt.cpp 
    ../src/mechanics/manipulators/DartWholeHand.cpp
    ../src/mechanics/manipulators/wholehand/optimizer.cpp
    ../src/mechanics/manipulators/wholehand/sdf.cpp
    ../src/tasks/wholehand_task.cpp 
    ../src/tasks/wholehand_setup.cpp)
target_link_libraries(box_surface_opt PRIVATE mcts-mechanics ${dart_libs} dart-optimizer-nlopt ${YAML_LIBRARY})

add_executable(sample_configs sample_configs.cpp 
    ../src/mechanics/manipulators/DartWholeHand.cpp
    ../src/mechanics/manipulators/wholehand/optimizer.cpp
    ../src/mechanics/manipulators/wholehand/sdf.cpp
    ../src/tasks/wholehand_task.cpp 
    ../src/tasks/wholehand_setup.cpp)
target_link_libraries(sample_configs PRIVATE mcts-mechanics ${dart_libs} dart-optimizer-nlopt ${YAML_LIBRARY})

add_executable(test_ik test_ik.cpp 
    ../src/mechanics/manipulators/DartWholeHand.cpp
    ../src/mechanics/manipulators/wholehand/optimizer.cpp
    ../src/mechanics/manipulators/wholehand/sdf.cpp
    ../src/tasks/wholehand_task.cpp 
    ../src/tasks/wholehand_setup.cpp)
target_link_libraries(test_ik PRIVATE mcts-mechanics ${dart_libs} dart-optimizer-nlopt ${YAML_LIBRARY})

add_executable(test_sdf test_sdf.cpp 
    ../src/mechanics/manipulators/DartWholeHand.cpp
    ../src/mechanics/manipulators/wholehand/optimizer.cpp
    ../src/mechanics/manipulators/wholehand/sdf.cpp
    ../src/tasks/wholehand_task.cpp 
    ../src/tasks/wholehand_setup.cpp)
target_link_libraries(test_sdf PRIVATE mcts-mechanics ${dart_libs} dart-optimizer-nlopt ${YAML_LIBRARY})