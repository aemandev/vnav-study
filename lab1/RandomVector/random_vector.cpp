#include "random_vector.h"
#include <cstdlib>
#include <vector>
// TODO: add any include you might require

RandomVector::RandomVector(int size, double max_val) { 
  // TODO: Write your code here

  for (auto i=1; i <= size; ++i){
      vect.push_back(rand()/double(RAND_MAX));
    }

}

void RandomVector::print(){
  // TODO: Write your code here
  for (auto &elem: vect){
    std::cout << elem << " ";
  }
  std::cout << std::endl;
}

double RandomVector::mean(){
  // TODO: Write your code here
  double sum = 0;
  for (auto &elem: vect){
    sum+= elem;
  }
  return sum/vect.size();
}

double RandomVector::max(){
  // TODO: Write your code here
    
  double max = vect[0];
  for (auto &elem: vect){
    if (elem>max){
      max = elem;
    }
  }
  return max;
}

double RandomVector::min(){
  //TODO:  Write your code here
  double min = vect[0];
  for (auto &elem: vect){
    if (elem<min){
      min = elem;
    }
  }
  return min;
}

void RandomVector::printHistogram(int bins){
  // TODO: Write your code here


}
