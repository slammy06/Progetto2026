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
inline project::check project::collision_check(system const& sys){
    check planet;
    auto const& bodies = sys.get_bodies();
     for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
     project::point<float> r{
          std::abs(bodies[i].pos.x - bodies[j].pos.x),
          std::abs(bodies[i].pos.y - bodies[j].pos.y)};
          float r_norm = r.norm();
          float k_min = bodies[i].radius + bodies[j].radius;
          if(r_norm <= k_min){
            planet.i = i;
            planet.j = j;
            planet.crash = true;
            return planet;
          } else {
            planet.crash = false;
            return planet;
          }
    }
}
}
inline void project::collided (system& syst, int i, int j){
float mass_tot{0.};
float radius_tot{0.};
mass_tot = syst.get_bodies()[i].mass +  syst.get_bodies()[j].mass;
radius_tot = syst.get_bodies()[i].radius +  syst.get_bodies()[j].radius;
syst.get_bodies()[i].mass = mass_tot;
syst.get_bodies()[i].radius = radius_tot;
syst.get_bodies()[i].acc = {0., 0.};
syst.get_bodies()[i].pos = syst.get_bodies()[i].pos * syst.get_bodies()[i].mass  + syst.get_bodies()[j].pos * syst.get_bodies()[j].mass/ mass_tot;
syst.get_bodies()[i].vel = syst.get_bodies()[i].vel * syst.get_bodies()[i].mass  + syst.get_bodies()[j].vel * syst.get_bodies()[j].mass/ mass_tot;
syst.get_bodies().erase(syst.get_bodies().begin() + j);
syst.get_energy().erase(syst.get_energy().begin() + j);
}

inline void project::vel_verlet(system& sys, float dt, int t) {
    std::vector<body> bodies = sys.get_bodies();
    for(dt = 0; dt<t; dt++){
      check crashed = collision_check(sys);
      if(crashed.crash == true){
        collided(sys, crashed.i, crashed.j);
        bodies.erase(bodies.begin() + crashed.j);
        bodies[crashed.i] = sys.get_bodies()[crashed.i]; 
      }
  for (long unsigned int l = 0; l < bodies.size(); ++l) {
    // update the position of body i
    bodies[l].pos += bodies[l].vel * dt +
                                 bodies[l].acc * dt * dt * 0.5;
    std::vector<point<float>> acc = sys.get_accelerations();

    sys.compute_acceleration();

    bodies[l].vel +=
        (acc[l] + bodies[l].acc) * dt * 0.5;
        sys.get_bodies()[l] = bodies[l];
  }
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
