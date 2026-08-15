#include "main.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
const float G = 6.67e-11;
const float epsilon = 1e-12;
namespace project {
inline body::compute_acceleration() {
  for (int i = 0; i < n_bodies; ++i) {
    for (int j = i + 1; j < n_bodies; ++j) {
      point<float> r{
          bodies[j].pos.x - bodies[i].pos.x,
          bodies[j].pos.y - bodies[i].pos.y};  // distance between body i and j
      float r_norm = r.norm();
      bodies[i].acc += point<float>{
          G * bodies[j].mass * r.x /
          std::sqrt(std::pow(r_norm * r_norm + epsilon * epsilon, 3.0))};
    }
  }
};
inline void vel_verlet(system& sys, float dt, int t) {
  for (int i = 0; i < sys.get_bodies().size(); ++i) {
    // update the position of body i
    sys.get_bodies()[i].pos.x += sys.get_bodies()[i].vel.x * dt +
                                 0.5 * sys.get_bodies()[i].acc.x * dt * dt;
    sys.get_bodies()[i].pos.y += sys.get_bodies()[i].vel.y * dt +
                                 0.5 * sys.get_bodies()[i].acc.y * dt * dt;
    std::vector<point<float>> acc = sys.get_accelerations();

    sys.compute_acceleration();

    sys.get_bodies()[i].vel.x +=
        0.5 * (acc[i].x + sys.get_bodies()[i].acc.x) * dt;
    sys.get_bodies()[i].vel.y +=
        0.5 * (acc[i].y + sys.get_bodies()[i].acc.y) * dt;
  }
};
}  // namespace project