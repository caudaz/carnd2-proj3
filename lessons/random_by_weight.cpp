// discrete_distribution
#include <iostream>
#include <random>

int main()
{
  const int nrolls = 10000; // number of experiments
  const int nstars = 100;   // maximum number of stars to distribute

  std::default_random_engine generator;
  std::discrete_distribution<int> distribution {2,2,1,1,2,2,1,1,2,2};
  
  std::cout << distribution << std::endl;

  // start with an empty vector 0-9 where counting will be done   
  int p[10]={};

  for (int i=0; i<nrolls; ++i) {
	std::cout << "i " << i << std::endl;
    int number = distribution(generator);
	std::cout << "number " << number << std::endl;
    ++p[number];
	std::cout << "++p[number]" << p[number] << std::endl;	
  }

  std::cout << std::endl;
  for (int i=0; i<10; ++i)
    std::cout << i << ": p[i]: " << p[i] << std::endl;
  
  std::cout << "a discrete_distribution:" << std::endl;
  for (int i=0; i<10; ++i)
    std::cout << i << ": " << std::string(p[i]*nstars/nrolls,'*') << std::endl;

int x = 6;
std::cout << std::endl;
std::cout << std::string(x,'a') << std::endl;
x = 3;
std::cout << std::endl;
std::cout << std::string(x,'b') << std::endl;

  return 0;
}

