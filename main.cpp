#include "main.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
const float G = 6.67e-11;
const float epsilon = 1e-12;
using namespace project;
inline void project::system::compute_acceleration() {
  for (long unsigned int i = 0; i < n_bodies; ++i) {
    for (long unsigned int j = i + 1; j < n_bodies; ++j) {
      point<float> r{
          bodies[j].pos.x - bodies[i].pos.x,
          bodies[j].pos.y - bodies[i].pos.y};  // distance between body i and j
      float r_norm = r.norm();
      bodies[i].acc += point<float>{
          G * bodies[j].mass * r.x /
              std::sqrt(std::pow(r_norm * r_norm + epsilon * epsilon, 3.0f)),
          G * bodies[j].mass * r.y /
              std::sqrt(std::pow(r_norm * r_norm + epsilon * epsilon, 3.0f))};
    }
  }
};
inline void project::vel_verlet(system& sys, float dt, int t) {
  for (long unsigned int l = 0; l < sys.get_bodies().size(); ++l) {
    // update the position of body i
    sys.get_bodies()[l].pos.x += sys.get_bodies()[l].vel.x * dt +
                                 0.5 * sys.get_bodies()[l].acc.x * dt * dt;
    sys.get_bodies()[l].pos.y += sys.get_bodies()[l].vel.y * dt +
                                 0.5 * sys.get_bodies()[l].acc.y * dt * dt;
    std::vector<point<float>> acc = sys.get_accelerations();

    sys.compute_acceleration();

    sys.get_bodies()[l].vel.x +=
        0.5 * (acc[l].x + sys.get_bodies()[l].acc.x) * dt;
    sys.get_bodies()[l].vel.y +=
        0.5 * (acc[l].y + sys.get_bodies()[l].acc.y) * dt;
  }
};

inline std::vector<point<float>> generate_points(int n, float min, float max) {
  std::vector<point<float>> points;

  std::default_random_engine eng;
  std::uniform_real_distribution<float> uniform{min, max};
  std::generate_n(std::back_inserter(points), n,
                  [&]() { return uniform(eng); });
  return points;
}
