#include <iostream>

#include <mcts.h>

#include "finger_contact.h"

void main_program(){
    // create initial state

    MCTS::ComputeOptions options;
    options.max_iterations = 1000;
    options.verbose = true;
    // options.number_of_threads = 8;

    ObjectTrajectory object_traj("/home/xianyi/Research/MCTS/data/test_finger_contact", 1, 0.3, 0.8);
    FingerContactState state(object_traj);

    // MCTS until there is no move
    while(state.has_moves()){
        FingerContactState::Move move = FingerContactState::no_move;

        move = MCTS::compute_move(state, options);
        state.do_move(move);
    }

    // print the state result
    std::cout << "Reached Final State: " << state.get_result(0) << std::endl;
}

int main()
{
	// try {
	// 	main_program();
	// }
	// catch (std::runtime_error& error) {
	// 	std::cerr << "ERROR: " << error.what() << std::endl;
	// 	return 1;
	// }
    main_program();
}