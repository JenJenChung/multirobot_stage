// Some functions migrated from rebhuhnc/libraries/Math/easymath.h
#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <vector>
#include <math.h>
#include <stdlib.h>

namespace easymath{
// Returns a random number between two values
double rand_interval(double low, double high){
  double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  return r*(high - low) + low;
}

// Normalise angles between +/-PI
double pi_2_pi(double x){
  x = fmod(x+M_PI,2.0*M_PI) ;
  if (x < 0.0)
    x += 2.0*M_PI ;
  return x - M_PI ;
}

// Sum elements in a vector
double sum(std::vector<double> v){
  double t = 0 ;
  for (size_t i = 0; i < v.size(); i++)
    t += v[i] ;
  return t ;
}
} // namespace easymath
#endif // UTILITIES_H_
