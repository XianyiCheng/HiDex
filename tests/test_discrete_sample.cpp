// discrete_distribution
#include <iostream>
#include <random>

int main()
{
  const int nrolls = 10000; // number of experiments
  const int nstars = 100;   // maximum number of stars to distribute

  std::vector<double> weights {2,2,1,1,2,2,1,1,2,2};

  std::default_random_engine generator;
  std::discrete_distribution<int> distribution {weights.begin(), weights.end()};

  int p[10]={};

  for (int i=0; i<nrolls; ++i) {
    int number = distribution(generator);
    ++p[number];
  }

  std::cout << "a discrete_distribution:" << std::endl;
  for (int i=0; i<10; ++i)
    std::cout << i << ": " << std::string(p[i]*nstars/nrolls,'*') << std::endl;

  return 0;
}