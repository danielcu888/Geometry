#ifndef GUARD_VECTOR_H_
#define GUARD_VECTOR_H_

#include <limits>
#include <stdexcept>
#include <cassert>
#include <cmath>

#include <boost/math/constants/constants.hpp>

namespace geometry
{

class Vector
{
public:
  Vector() = default;
  Vector(double x, double y)
    : m_x(x)
    , m_y(y)
  {}

  void rotate(double theta)
  {
    if (theta < -180.0 || theta > 180.0)
    {
      throw std::out_of_range("-180 < theta < 180");
    }

    theta *= (boost::math::double_constants::pi/180.0);
    m_x = cos(theta) * m_x - sin(theta) * m_y;
    m_y = sin(theta) * m_x + cos(theta) * m_y;
  }

  // Angles range between -180.0 < theta < 180.0
  double angle() const
  {
    // Cope with null vector
    const double norm = m_x*m_x + m_y*m_y;
    if (norm < std::numeric_limits<double>::epsilon())
    {
      // null vector, throw
      throw std::logic_error("Null Vector!!!");
    }

    if (fabs(m_x) < std::numeric_limits<double>::epsilon())
    {
      // Cope with zero m_x
      // Angle is either 90.0 or -90.0
      return m_y > 0.0 ? 90.0 : -90.0;
    }
    else if(fabs(m_y) < std::numeric_limits<double>::epsilon())
    {
      // Cope with zero m_y
      // Angle is either 0 or +/-180.0 (choose + sign arbitrarily, makes no difference).
      return m_x > 0.0 ? 0.0 : 180.0;
    }
    else
    {
      // General case

      double theta = 180.0/boost::math::double_constants::pi * atan(m_y/m_x);
      // This will return an angle between -90.0 and 90.0.
      // i.e., in the 1st and 4th sectors.
      assert(theta < 90.0 && theta > -90.0);

      // There is an ambiguity between the the two pairs of sectors.
      // Deal with 1st
      if (theta > 0.0 && theta < 90.0)
      {
        if (m_y < 0.0)
        {
          // we are actually in the 3rd sector.
          theta -= 180.0;
        }
      }
      // Deal with 4th sector ambiguity
      else if (theta < 0.0 && theta > -90.0)
      {
        if (m_y > 0.0)
        {
          // we are actually in the 2nd sector.
          theta += 180.0;
        }
      }

      return theta;
    }
  }

  double x() const
  {
    return m_x;
  }

  double y() const
  {
    return m_y;
  }

private:

  double m_x;
  double m_y;

}; // ! class Vector

double angular_difference(const Vector& v1, const Vector& v2)
{
  const double angle1 = v1.angle();
  const double angle2 = v2.angle();

  double ret = angle2 - angle1;
  if (ret > 180.0)
  {
    ret = 360.0 - ret;
  }
  else if (ret < -180.0)
  {
    ret += 360.0;
  }

  return ret;
}

Vector rotate(const Vector& v, double theta)
{
  if (theta < -180.0 || theta > 180.0)
  {
    throw std::out_of_range("-180 < theta < 180");
  }

  theta *= (boost::math::double_constants::pi/180.0);

  Vector ret( cos(theta) * v.x() - sin(theta) * v.y()
            , sin(theta) * v.x() + cos(theta) * v.y()
            );

  return ret;
}

} // ! namespace geometry

#endif
