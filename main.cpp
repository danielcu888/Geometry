#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "Vector.h"

int main(int argc, char *argv[])
{
  // Tests for angle of single vector
  const double theta_start = -180.0;
  const double theta_end = 180.0;
  const double theta_delta = 0.01;

  geometry::Vector v0(1.0, 0.0);
  for (double theta = theta_start+theta_delta; theta < theta_end; theta += theta_delta)
  {
    const geometry::Vector v = geometry::rotate(v0, theta);
    const double ang = v.angle();
    if (fabs(ang - theta) > 1e-13)
    {
      std::cout << "expected: " << theta << ", obtained: " << ang << std::endl;
    }
  }

  // Tests for angle between two vectors
  geometry::Vector v1(1.0, 0.0);
  v1.rotate(178.0);
  geometry::Vector v2(1.0, 0.0);
  v2.rotate(-178.0);

  std::cout << geometry::angular_difference(v1, v2) << std::endl;

  return EXIT_SUCCESS;
}
