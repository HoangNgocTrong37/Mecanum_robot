#pragma once
#include <cmath>
#include <stdexcept>

namespace rf2o {
struct PlanarTwist { double x, y, yaw; };

// SE(2) logarithm: translation is expressed in the previous base frame.
// This also works for a rotating, translated laser after converting poses to base_link.
inline PlanarTwist planarTwist(double dx, double dy, double yaw, double dt)
{
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(yaw) ||
      !std::isfinite(dt) || dt <= 0.0)
    throw std::invalid_argument("Invalid planar pose increment / scan interval");
  const double a = std::abs(yaw) < 1e-8 ? 1.0 : std::sin(yaw)/yaw;
  const double b = std::abs(yaw) < 1e-8 ? yaw/2.0 : (1.0-std::cos(yaw))/yaw;
  const double divisor = (a*a+b*b)*dt;
  return {(a*dx+b*dy)/divisor, (-b*dx+a*dy)/divisor, yaw/dt};
}
}
