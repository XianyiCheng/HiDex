#include <vector>
#include <iostream>
#include "../mechanics/utilities/utilities.h"
#include "../mechanics/utilities/combinatorics.h"

std::vector<int> get_finger_locations(int N, int n, long int finger_location_index)
{

    // obtain finger location idxes from the single location idx
    // N: number of surface contact points
    // n: number of fingers
    long int action_idx =
        finger_location_index; // action_idx: index of the action

    std::vector<int> locations;

    // find out the number of active fingers
    int k = 0; // the number of active fingers
    long int sum = 0;
    for (k = 0; k <= n; k++)
    {
        // k: number of fingers on a surface contact point
        long int sum_i = combination(n, k) * permutation(N, k);
        if (sum + sum_i > action_idx)
        {
            break;
        }
        sum += sum_i;
    }

    // find out active finger indices
    std::vector<int> active_idxes;
    long int comb_idx = (action_idx - sum) / permutation(N, k);
    active_idxes = combination_set(n, k, comb_idx);
    // find out the locations of active finger indices
    long int loc_idx = (action_idx - sum) % permutation(N, k);
    std::vector<int> loc_idxes;
    loc_idxes = permutation_set(N, k, loc_idx);

    // create a vector of -1 with size n

    for (int i = 0; i < n; i++)
    {
        locations.push_back(-1);
    }

    // assign locations to active fingers
    for (int i = 0; i < active_idxes.size(); ++i)
    {
        locations[active_idxes[i]] = loc_idxes[i];
    }

    return locations;
}

int main(int argc, char** argv)
{
    if (argc != 3 && argc != 5){
        std::cout << "Usage: test_action_idx N n (min_idx=2*N) (max_idx=4*N)" << std::endl;
        return 0;
    }
    int N = atoi(argv[1]); 
    int n = atoi(argv[2]);
    int min_idx = 2*N;
    int max_idx = 4*N;
    if (argc == 5){
        min_idx = atoi(argv[3]);
        max_idx = atoi(argv[4]);
    }
    
    for (int i = min_idx; i < max_idx; i++){
        std::vector<int> locations = get_finger_locations(N, n, i);
        std::cout << "Finger index: " << i << ". Locations: ";
        for (int j = 0; j < locations.size(); j++){
            std::cout << locations[j] << " ";
        }
        std::cout << std::endl;
    }
}