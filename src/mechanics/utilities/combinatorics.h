#include <vector>

long int permutation(int N, int k);

long int combination(int N, int k);

std::vector<int> combination_set(int N, int k, long int idx);

std::vector<int> permutation_set(int N, int k, long int idx);

long int index_in_combination_set(int N, int k, const std::vector<int> & set);

long int index_in_permutation_set(int N, int k, const std::vector<int> & set);