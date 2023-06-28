#include "../src/tasks/setup.h"

#include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{
    std::string head_1 = "/home/xianyi/Research/MCTS/general_planner/";
    std::string head_2 = "/home/xianyi/Research/MCTS/general_planner";
    std::string tail_1 = "setup_template.yaml";
    std::string tail_2 = "/setup_template.yaml";

    std::cout << path_join(head_1, tail_1) << std::endl;
    std::cout << path_join(head_1, tail_2) << std::endl;
    std::cout << path_join(head_2, tail_1) << std::endl;
    std::cout << path_join(head_2, tail_2) << std::endl;
}