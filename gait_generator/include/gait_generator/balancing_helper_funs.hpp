#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <chrono>

struct Point {
  double x, y;
};

static Point polygon_centroid(const std::vector<Point> &poly) {
  Point c{0.0, 0.0};
  double A = 0.0;
  int n = (int)poly.size();
  for (int i = 0; i < n; ++i) {
    int j = (i+1)%n;
    double crossp = poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    c.x += (poly[i].x + poly[j].x) * crossp;
    c.y += (poly[i].y + poly[j].y) * crossp;
    A += crossp;
  }
  A *= 0.5;
  if (std::abs(A) < 1e-12) return poly.empty() ? Point{0,0} : poly[0]; // degenerate
  c.x /= (6.0 * A);
  c.y /= (6.0 * A);
  return c;
}



