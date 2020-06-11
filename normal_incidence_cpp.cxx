#include <iostream>
#include <cmath>
#include "calculateNormalIncidence.hxx"
#include <vector>

int main() {
  std::vector<double> result;
  result.reserve(6);

  for (int j = 0; j < 1000; j++) {
    result = calculateNormalIncidence(0.3, 0.31, 0.31, 0.1, j / 5000.0, j / 5000.0, 0, 0);

    for (int i = 0; i < 6; i++) {
      std::cout << result[i] << std::endl;
    }
  }
}
