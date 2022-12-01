#include <vector>
#include <iostream>
#include "../mechanics/utilities/utilities.h"
#include "../mechanics/utilities/combinatorics.h"

int main()
{

    std::vector<int> c_set = combination_set(55, 9, 1237);
    index_in_combination_set(55, 9, c_set);

    int N_min = 3;
    int N_max = 60;
    int k_max = 10;
    for (int iter = 0; iter < 100; ++ iter){
        int N = randi(N_max - N_min) + N_min;
        int k;
        if (N > k_max){
            k = randi(k_max);
        } else {
            k = randi(N);
        }

        int idx = randi(combination(N, k));

        std::cout << "N: " << N << ", k: " << k << ", idx: " << idx;

        std::vector<int> c_set = combination_set(N, k, idx);
        std::cout << ", c_idx: " << index_in_combination_set(N, k, c_set) << std::endl;
    }

    for (int iter = 0; iter < 100; ++ iter){
        int N = randi(N_max - N_min) + N_min;
        int k;
        if (N > k_max){
            k = randi(k_max);
        } else {
            k = randi(N);
        }

        int idx = randi(combination(N, k));

        std::cout << "N: " << N << ", k: " << k << ", idx: " << idx;

        std::vector<int> p_set = permutation_set(N, k, idx);
        std::cout << ", p_idx: " << index_in_permutation_set(N, k, p_set) << std::endl;
    }
}