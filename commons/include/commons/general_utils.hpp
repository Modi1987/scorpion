#ifndef GENERAL_UTILS_HPP_
#define GENERAL_UTILS_HPP_

#include <iostream>

class ExponentialMovingAverage {
private:
  double alfa{0.1};
  double filtered{0.0};

public:
  explicit ExponentialMovingAverage(double alfa = 0.1) {
    if (alfa <= 0.0 || alfa >= 1.0) {
      std::cerr << "ExponentialMovingAverage warning: alfa must be between 0 "
                   "and 1 (exclusive). "
                << "Passed value: " << alfa << ". Using default alfa = 0.1.\n";
      this->alfa = 0.1;
    } else {
      this->alfa = alfa;
    }
  }

  double update(double measurement) {
    filtered = (1.0 - alfa) * filtered + alfa * measurement;
    return filtered;
  }

  void reset(double value = 0.0) { filtered = value; }
  double getFiltered() const { return filtered; }
};

#endif // GENERAL_UTILS_HPP_
