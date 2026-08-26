#include "main.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
const float G = 100;

using namespace project;

// Constructors

System::System(long unsigned int n, std::vector<float> const& masses,
               std::vector<point<float>> const& pos,
               std::vector<point<float>> const& vel,
               std::vector<float> const& rads)
    : n_bodies(n) {
  for (long unsigned int i = 0; i < n; i++) {
    bodies.push_back(
        Body{pos[i], vel[i], point<float>{0.0, 0.0}, masses[i], rads[i]});
  }
}

System::System(std::vector<Body> const& in_bodies) {
  n_bodies = in_bodies.size();
  for (long unsigned int i = 0; i < in_bodies.size(); i++) {
    bodies.push_back(in_bodies[i]);
  }
}
// Methods

inline void System::compute_acceleration() {  // calculates and sets
                                              // new accelerations
  for (long unsigned int i = 0; i < n_bodies; ++i) {
    bodies[i].acc = {0.0f, 0.0f};
    for (long unsigned int j = 0; j < n_bodies; ++j) {
      if (i == j) continue;
      point<float> r = bodies[j].pos - bodies[i].pos;
      float r_norm = r.norm();
      bodies[i].acc += point<float>{
          G * bodies[j].mass * r.x / std::sqrt(std::pow(r_norm * r_norm, 3.0f)),
          G * bodies[j].mass * r.y /
              std::sqrt(std::pow(r_norm * r_norm, 3.0f))};
    }
  }
}

void System::kineticEnergy() {
  float K = 0.f;

  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    float v2 = bodies[i].vel.norm() * bodies[i].vel.norm();
    K += 0.5f * bodies[i].mass * v2;
  }
  kinetic.push_back(K);
}

void System::potentialEnergy() {
  float U = 0.f;
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
      point<float> r = bodies[j].pos - bodies[i].pos;
      float norm_r = r.norm();
      U -= G * bodies[i].mass * bodies[j].mass / norm_r;
    }
  }
  potential.push_back(U);
}

void System::totalEnergy() {
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    totEnergy.push_back(kinetic[i] + potential[i]);
  }
}

void System::linearMomentum() {
  point<float> P{0., 0.};
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    P += bodies[i].vel * bodies[i].mass;
  }
  lin_momentum.push_back(P);
}

void System::angularMomentum() {
  float L{0.f};
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    L += bodies[i].mass * (bodies[i].pos.x * bodies[i].vel.y -
                           bodies[i].pos.y * bodies[i].vel.x);
  }
  ang_momentum.push_back(L);
}

inline auto System::get_accelerations() {  // returns the current accelerations
                                           // of the bodies
  std::vector<point<float>> acc;

  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    acc.push_back(bodies[i].acc);
  }
  return acc;
}

// Functions

void project::vel_verlet(System& sys, float dt) {
  // checking for collisions before updating
  collided(sys, project::collision_check(sys));
  // saving previous accelerations
  std::vector<point<float>> acc = sys.get_accelerations();
  // updating the positions
  for (long unsigned int i = 0; i < sys.get_bodies().size(); ++i) {
    sys.get_bodies()[i].pos +=
        sys.get_bodies()[i].vel * dt + sys.get_bodies()[i].acc * dt * dt * 0.5f;
  }

  sys.compute_acceleration();  // updating accelerations
  // updating velocities
  for (long unsigned int i = 0; i < sys.get_bodies().size(); ++i) {
    sys.get_bodies()[i].vel += (acc[i] + sys.get_bodies()[i].acc) * dt * 0.5f;
  }
}

/*inline std::vector<point<float>> project::generate_points(int n, float min,
                                                          float max) {
  std::vector<point<float>> points;

  std::default_random_engine eng;
  std::uniform_real_distribution<float> uniform{min, max};
  std::generate_n(std::back_inserter(points), n,
                  [&]() { return uniform(eng); });
  return points;
}*/

std::vector<project::check> project::collision_check(
    project::System const& sys) {
  auto const& bodies = sys.get_bodies();
  std::vector<check> planets(bodies.size(), check{0, 0, false});
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
      project::point<float> r{std::abs(bodies[i].pos.x - bodies[j].pos.x),
                              std::abs(bodies[i].pos.y - bodies[j].pos.y)};
      float r_norm = r.norm();
      float k_min = bodies[i].radius + bodies[j].radius;
      if (r_norm <= k_min) {
        planets[i].i = static_cast<int>(i);
        planets[j].j = static_cast<int>(j);
        planets[i].crash = true;

      } else {
        planets[i].crash = false;
      }
    }
  }
  return planets;
}
inline void project::collided(System& syst, std::vector<check> const& planets) {
  (void)planets;
  for (long unsigned int i = 0; i < syst.get_bodies().size(); ++i) {
    for (long unsigned int j = i + 1; j < syst.get_bodies().size(); ++j) {
      const point<float> distance = syst.get_bodies()[j].pos -
                                    syst.get_bodies()[i].pos;
      if (distance.norm() <= syst.get_bodies()[i].radius +
                                 syst.get_bodies()[j].radius) {
        const float first_mass = syst.get_bodies()[i].mass;
        const float second_mass = syst.get_bodies()[j].mass;
        const float mass_tot = first_mass + second_mass;
        const float radius_tot = syst.get_bodies()[i].radius +
                                 syst.get_bodies()[j].radius;
        syst.get_bodies()[i].mass = mass_tot;
        syst.get_bodies()[i].radius = radius_tot;
        syst.get_bodies()[i].acc = {0., 0.};
        syst.get_bodies()[i].pos =
            (syst.get_bodies()[i].pos * first_mass +
             syst.get_bodies()[j].pos * second_mass) /
            mass_tot;
        syst.get_bodies()[i].vel =
            (syst.get_bodies()[i].vel * first_mass +
             syst.get_bodies()[j].vel * second_mass) /
            mass_tot;
        syst.get_bodies().erase(
            syst.get_bodies().begin() +
            static_cast<std::vector<Body>::difference_type>(j));
        syst.update_body_count();
        return;
      }
    }
  }
}
void project::step(project::System& sys, float dt) {
  project::vel_verlet(sys, dt);
  sys.kineticEnergy();
  sys.potentialEnergy();
  sys.linearMomentum();
  sys.angularMomentum();
};


void System::update_body_count() { n_bodies = bodies.size(); }
