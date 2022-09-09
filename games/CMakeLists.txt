# Author: petter.strandmark@gmail.com (Petter Strandmark)

MACRO (CREATE_EXAMPLE NAME)
	ADD_EXECUTABLE(${NAME}
	               ${NAME}.cpp
	               ${MCTS_HEADERS})
	MESSAGE("-- Adding game: " ${NAME})
ENDMACRO (CREATE_EXAMPLE)

CREATE_EXAMPLE(connect_four)

ADD_EXECUTABLE(finger_contact_planning
finger_contact.cpp
${MCTS_HEADERS})

target_link_libraries(finger_contact_planning PRIVATE mcts-mechanics)
target_include_directories(finger_contact_planning PRIVATE "${PROJECT_SOURCE_DIR}/mechanics/external/eigen3")