#include "../tasks/rewards.h"
#include <assert.h>
int main(){
    std::shared_ptr<RewardFunction> reward;
    std::vector<std::string> feature_names = {"hshs", "hhhh"};
    reward = std::make_shared<ExampleReward>(feature_names);

    assert(reward->get({1, 2, 3}) == 0.0);

    return 0;
}
