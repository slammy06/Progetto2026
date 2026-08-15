#ifndef PROJECT_MAIN_HPP
#define PROJECT_MAIN_HPP
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace project {
template <typename T1>

struct point {
  T1 x;
  T1 y;
  T1 norm() {
    return std::sqrt(x * x + y * y);
    T1 operator+(point<T1> const& p) { return point<T1>{x + p.x, y + p.y}; };
    T1 operator-(point<T1> const& p) { return point<T1>{x - p.x, y - p.y}; };
    T1 operator+=(point<T1> const& p) {
      x += p.x;
      y += p.y;
      return *this;
    }
  }
};

struct body {
  point<float> pos{0.0, 0.0};
  point<float> vel{0.0, 0.0};
  point<float> acc{0.0, 0.0};
  float mass{0.0};
  float radius{0.0};
};

class system {
  std::vector<body> bodies;
  std::vector<float> energy;
  int n_bodies{0};

 public:
  system(int n, std::vector<float> const& masses,
         std::vector<point<float>> const& pos,
         std::vector<point<float>> const& vel)
      : n_bodies(n) {
    assert(masses.size() == n);
    for (int i = 0; i < n; i++) {
      bodies.push_back(
          body{pos[i], vel[i], point<float>{0.0, 0.0}, masses[i], 1.0});
    }
  };
  auto get_bodies() { return bodies; };
  auto get_accelerations() {  // returns the current accelerations of the bodies
    std::vector<point<float>> acc;
    for (int i = 0; i < bodies.size(); ++i) {
      acc.push_back(bodies[i].acc);
    }
    return acc;
  }
  void compute_acceleration();
};

void vel_verlet(system& sys, float dt,
                int t);  // defines one step for a system type object

};  // namespace project

#endif