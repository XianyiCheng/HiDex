#include <vector>
#include <iostream>

int permutation(int N, int k)
{
    int result = 1;
    for (int i = 0; i < k; i++)
    {
        result *= (N - i);
    }
    return result;
}

int combination(int N, int k)
{
    return permutation(N, k) / permutation(k, k);
}

int total_finger_locations(int N, int n)
{
    // N: number of surface contact points
    // n: number of fingers
    int sum = 0;
    for (int k = 0; k <= n; k++)
    {
        // k: number of fingers on a surface contact point
        int sum_i = combination(n, k) * permutation(N, k);
        std::cout << "sum_i: " << sum_i << std::endl;
        sum += sum_i;
    }

    return sum;
}

std::vector<int> combination_set(int N, int k, int idx)
{
    // N: number of surface contact points
    // k: number of fingers on a surface contact point
    // idx: index of the combination set
    std::vector<int> set;

    int comb_idx = idx;
    int counter = 0;
    for (int i = 0; i < k; i++)
    {
        // i: the index of the combination set
        int sum = 0;
        int j;
        for (j = counter; j <= N - k + i; j++)
        {
            // j: the index of the sub combination set
            int comb_i = combination(N - 1 - j, k - 1 - i);
            if (sum + comb_i > comb_idx)
            {
                comb_idx = comb_idx - sum;
                break;
            }
            sum += comb_i;
        }
        set.push_back(j);
        counter = j+1;
    }

    return set;
}

std::vector<int> permutation_set(int N, int k, int idx)
{
    // N: number of surface contact points
    // k: number of fingers on a surface contact point
    // idx: index of the combination set
    std::vector<int> set;
    std::vector<int> set_i;
    for (int i = 0; i < N; i++)
    {
        set_i.push_back(i);
    }

    int permu_idx = idx;
    for (int i = 0; i < k; i++)
    {
        int permu_i = permutation(N - i - 1, k - i - 1);
        int set_idx = permu_idx / permu_i;
        set.push_back(set_i[set_idx]);
        set_i.erase(set_i.begin() + set_idx);
        permu_idx = permu_idx % permu_i;
    }
    return set;
}

std::vector<int> get_locations(int N, int n, int action_idx)
{
    // N: number of surface contact points
    // n: number of fingers
    // action_idx: index of the action
    std::vector<int> locations;

    // find out the number of active fingers
    int k = 0;
    int sum = 0;
    for (k = 0; k <= n; k++)
    {
        // k: number of fingers on a surface contact point
        int sum_i = combination(n, k) * permutation(N, k);
        if (sum + sum_i > action_idx)
        {
            break;
        }
        sum += sum_i;
    }

    // find out active finger indices
    std::vector<int> active_idxes;
    int comb_idx = (action_idx - sum) / permutation(N, k);
    active_idxes = combination_set(n, k, comb_idx);
    // find out the locations of active finger indices
    int loc_idx = (action_idx - sum) % permutation(N, k);
    std::vector<int> loc_idxes;
    loc_idxes = permutation_set(N, k, loc_idx);

    // assign locations
    for (int i = 0; i < n; i++)
    {
        if (active_idxes.size() == 0)
        {
            locations.push_back(-1);
            continue;
        }
        if (i == active_idxes[0])
        {
            locations.push_back(loc_idxes[0]);
            active_idxes.erase(active_idxes.begin());
            loc_idxes.erase(loc_idxes.begin());
        }
        else
        {
            locations.push_back(-1);
        }
    }

    return locations;
}

int main()
{
    int N = 3;
    int k = 3;

    int n_comb = combination(N, k);
    for (int idx = 0; idx < n_comb; idx++)
    {
        std::vector<int> ff = combination_set(N,k, idx);
        std::cout << "idx: " << idx << " , locations: ";
        for (auto k : ff)
        {
            std::cout << k << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}