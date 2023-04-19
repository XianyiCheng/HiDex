#include "combinatorics.h"

long int permutation(int N, int k) {
  long int result = 1;
  for (int i = 0; i < k; i++) {
    result *= (N - i);
  }
  return result;
}

long int combination(int N, int k) { return permutation(N, k) / permutation(k, k); }

std::vector<int> combination_set(int N, int k, long int idx) {
  // N: number of surface contact points
  // k: number of fingers on a surface contact point
  // idx: index of the combination set
  std::vector<int> set;

  int comb_idx = idx;
  int counter = 0;
  for (int i = 0; i < k; i++) {
    // i: the index of the combination set
    int sum = 0;
    int j;
    for (j = counter; j <= N - k + i; j++) {
      // j: the index of the sub combination set
      int comb_i = combination(N - 1 - j, k - 1 - i);
      if (sum + comb_i > comb_idx) {
        comb_idx = comb_idx - sum;
        break;
      }
      sum += comb_i;
    }
    set.push_back(j);
    counter = j + 1;
  }

  return set;
}

std::vector<int> permutation_set(int N, int k, long int idx) {
  // N: number of surface contact points
  // k: number of fingers on a surface contact point
  // idx: index of the combination set
  std::vector<int> set;
  std::vector<int> set_i;
  for (int i = 0; i < N; i++) {
    set_i.push_back(i);
  }

  long int permu_idx = idx;
  for (int i = 0; i < k; i++) {
    long int permu_i = permutation(N - i - 1, k - i - 1);
    int set_idx = permu_idx / permu_i;
    set.push_back(set_i[set_idx]);
    set_i.erase(set_i.begin() + set_idx);
    permu_idx = permu_idx % permu_i;
  }
  return set;
}

long int index_in_combination_set(int N, int k, const std::vector<int> &set) {

  long int idx = 0;

  int counter = 0;

  int set_idx = 0;

  for (int i = 0; i < k; i++) {
    // i: the index of the combination set
    
    int j;
    for (j = counter; j <= N - k + i; j++) {
      // j: the index of the sub combination set
      if (j == set[set_idx]) {
        set_idx += 1;
        break;
      }
      long int comb_i = combination(N - 1 - j, k - 1 - i);
      idx += comb_i;
    }
    counter = j + 1;
  }

  return idx;
}

long int index_in_permutation_set(int N, int k, const std::vector<int> &set) {
  // N: number of surface contact points
  // k: number of fingers on a surface contact point
  // idx: index of the combination set

  std::vector<int> set_i;
  for (int i = 0; i < N; i++) {
    set_i.push_back(i);
  }

  long int idx = 0; 
  for (int i = 0; i < k; i++) {
    long int permu_i = permutation(N - i - 1, k - i - 1);
    int set_idx = 0;
    for (auto j : set_i) {
      if (j == set[i]) {
        break;
      }
      set_idx += 1;
    }
    set_i.erase(set_i.begin() + set_idx);
    idx += set_idx * permu_i;
  }
  return idx;
}