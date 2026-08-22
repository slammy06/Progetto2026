#include "main.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
const float G = 100;
const float epsilon = 5;
using namespace project;

// Constructors

System::System(int n, std::vector<float> const& masses,
               std::vector<point<float>> const& pos,
               std::vector<point<float>> const& vel,
               std::vector<float> const& rads)
    : n_bodies(n) {
  for (int i = 0; i < n; i++) {
    bodies.push_back(
        Body{pos[i], vel[i], point<float>{0.0, 0.0}, masses[i], rads[i]});
  }
}
// Methods

inline void System::compute_acceleration() {  // calculates and sets
                                              // new accelerations
  for (long unsigned int i = 0; i < n_bodies; ++i) {
    for (long unsigned int j = 0; j < n_bodies; ++j) {
      if (i == j) continue;
      point<float> r = bodies[j].pos - bodies[i].pos;
      float r_norm = r.norm();
      bodies[i].acc += point<float>{
          G * bodies[j].mass * r.x /
              std::sqrt(std::pow(r_norm * r_norm + epsilon * epsilon, 3.0f)),
          G * bodies[j].mass * r.y /
              std::sqrt(std::pow(r_norm * r_norm + epsilon * epsilon, 3.0f))};
    }
  }
}

void System::kineticEnergy() {
  float K = 0.f;

  for (int i = 0; i < bodies.size(); ++i) {
    float v2 = std::pow(bodies[i].vel.norm(), 2);
    K += 0.5 * bodies[i].mass * v2;
  }
  kinetic.push_back(K);
}

void System::potentialEnergy() {
  float U = 0.f;
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
      point<float> r = bodies[j].pos - bodies[i].pos;
      float norm_r = r.norm();
      U -= G * bodies[i].mass * bodies[j].mass /
           (sqrt(norm_r * norm_r + epsilon * epsilon));
    }
  }
  potential.push_back(U);
}

void System::totalEnergy() {
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    totEnergy.push_back(kinetic[i] + potential[i]);
  }
};

inline auto System::get_accelerations() {  // returns the current accelerations
                                           // of the bodies
  std::vector<point<float>> acc;

  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    acc.push_back(bodies[i].acc);
  }
  return acc;
}

// Functions

inline void vel_verlet(System& sys, float dt) {
  std::vector<Body> bodies = sys.get_bodies();
  check crashed = collision_check(sys);
  if (crashed.crash == true) {
    collided(sys, crashed.i, crashed.j);
    bodies.erase(bodies.begin() + crashed.j);
    bodies[crashed.i] = sys.get_bodies()[crashed.i];
  }

  // updating the positions
  for (long unsigned int l = 0; l < bodies.size(); ++l) {
    bodies[l].pos += bodies[l].vel * dt + bodies[l].acc * dt * dt * 0.5;
  }
  std::vector<point<float>> acc =
      sys.get_accelerations();  // saving previous accelerations
  sys.compute_acceleration();   // updating accelerations
  // updating velocities
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    bodies[i].vel += (acc[i] + bodies[i].acc) * dt * 0.5;
    sys.get_bodies()[i] = bodies[i];
  }
}
inline std::vector<point<float>> generate_points(int n, float min, float max) {
  std::vector<point<float>> points;

  std::default_random_engine eng;
  std::uniform_real_distribution<float> uniform{min, max};
  std::generate_n(std::back_inserter(points), n,
                  [&]() { return uniform(eng); });
  return points;
}

inline project::check project::collision_check(System const& sys) {
  check planet;
  auto const& bodies = sys.get_bodies();
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
      project::point<float> r{std::abs(bodies[i].pos.x - bodies[j].pos.x),
                              std::abs(bodies[i].pos.y - bodies[j].pos.y)};
      float r_norm = r.norm();
      float k_min = bodies[i].radius + bodies[j].radius;
      if (r_norm <= k_min) {
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
inline void collided(System& syst, int i, int j) {
  float mass_tot{0.};
  float radius_tot{0.};
  mass_tot = syst.get_bodies()[i].mass + syst.get_bodies()[j].mass;
  radius_tot = syst.get_bodies()[i].radius + syst.get_bodies()[j].radius;
  syst.get_bodies()[i].mass = mass_tot;
  syst.get_bodies()[i].radius = radius_tot;
  syst.get_bodies()[i].acc = {0., 0.};
  syst.get_bodies()[i].pos =
      syst.get_bodies()[i].pos * syst.get_bodies()[i].mass +
      syst.get_bodies()[j].pos * syst.get_bodies()[j].mass / mass_tot;
  syst.get_bodies()[i].vel =
      syst.get_bodies()[i].vel * syst.get_bodies()[i].mass +
      syst.get_bodies()[j].vel * syst.get_bodies()[j].mass / mass_tot;
  syst.get_bodies().erase(syst.get_bodies().begin() + j);
  syst.get_totEnergy().erase(syst.get_totEnergy().begin() + j);
}
