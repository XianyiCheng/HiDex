# Add a library called mechanics
find_library(GLPKLIB glpk REQUIRED)

set(include_dirs 
    "${PROJECT_SOURCE_DIR}"
    "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3"
    )

add_library(mcts-mechanics 
    mechanics.cpp 
    contacts/contact_constraints.cpp 
    contacts/contact_kinematics.cpp
    utilities/sample.cpp
    utilities/utilities.cpp
    utilities/io.cpp)
    
target_include_directories(mcts-mechanics PUBLIC ${include_dirs})
target_link_libraries(mcts-mechanics PRIVATE ${GLPKLIB})
