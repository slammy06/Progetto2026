#include "main.hpp"

#include <cassert>
#include <cmath>
#include <vector>
const double G = 6.6743e-11;

using namespace project;

// Constructors

System::System(std::size_t n, std::vector<double> const& masses,
               std::vector<point<double>> const& pos,
               std::vector<point<double>> const& vel,
               std::vector<double> const& rads)
    : n_bodies(n) {
  for (long unsigned int i = 0; i < n; i++) {
    bodies.push_back(
        Body{pos[i], vel[i], point<double>{0.0, 0.0}, masses[i], rads[i]});
  }
  check_invariant();
}

System::System(std::vector<Body> const& in_bodies) {
  n_bodies = in_bodies.size();
  for (long unsigned int i = 0; i < in_bodies.size(); i++) {
    bodies.push_back(in_bodies[i]);
  }
  check_invariant();
}

void System::check_invariant() const {
  assert(n_bodies == bodies.size());
  for (const Body& body : bodies) {
    assert(std::isfinite(body.mass) && body.mass >= 0.0);
    assert(std::isfinite(body.radius) && body.radius >= 0.0);
  }
}
// Methods

void System::compute_acceleration() {  // calculates and sets
                                       // new accelerations
  for (long unsigned int i = 0; i < n_bodies; ++i) {
    bodies[i].acc = {0.0f, 0.0f};
    for (long unsigned int j = 0; j < n_bodies; ++j) {
      if (i == j) continue;
      point<double> r = bodies[j].pos - bodies[i].pos;
      double r_norm = r.norm();
      bodies[i].acc += point<double>{
          G * bodies[j].mass * r.x / std::sqrt(std::pow(r_norm * r_norm, 3.0f)),
          G * bodies[j].mass * r.y /
              std::sqrt(std::pow(r_norm * r_norm, 3.0f))};
    }
  }
  check_invariant();
}

void System::kineticEnergy() {
  double K = 0.f;

  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    double v2 = bodies[i].vel.norm() * bodies[i].vel.norm();
    K += 0.5f * bodies[i].mass * v2;
  }
  kinetic.push_back(K);
}

void System::potentialEnergy() {
  double U = 0.f;
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    for (long unsigned int j = i + 1; j < bodies.size(); ++j) {
      point<double> r = bodies[j].pos - bodies[i].pos;
      double norm_r = r.norm();
      U -= G * bodies[i].mass * bodies[j].mass / norm_r;
    }
  }
  potential.push_back(U);
}

void System::totalEnergy() {
  totEnergy.push_back(kinetic.back() + potential.back());
}

void System::linearMomentum() {
  point<double> P{0., 0.};
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    P += bodies[i].vel * bodies[i].mass;
  }
  lin_momentum.push_back(P);
}

void System::angularMomentum() {
  double L{0.f};
  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    L += bodies[i].mass * (bodies[i].pos.x * bodies[i].vel.y -
                           bodies[i].pos.y * bodies[i].vel.x);
  }
  ang_momentum.push_back(L);
}

auto System::get_accelerations() const {  // returns the current accelerations
                                          // of the bodies
  std::vector<point<double>> acc;

  for (long unsigned int i = 0; i < bodies.size(); ++i) {
    acc.push_back(bodies[i].acc);
  }
  return acc;
}

// Functions

void project::vel_verlet(System& sys, float dt) {
  // checking for collisions before updating
  collided(sys);
  // saving previous accelerations
  std::vector<point<double>> acc = sys.get_accelerations();
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

inline void project::collided(System& syst) {
  for (long unsigned int i = 0; i < syst.get_bodies().size(); ++i) {
    for (long unsigned int j = i + 1; j < syst.get_bodies().size(); ++j) {
      const point<double> distance =
          syst.get_bodies()[j].pos - syst.get_bodies()[i].pos;
      if (distance.norm() <=
          syst.get_bodies()[i].radius + syst.get_bodies()[j].radius) {
        const double first_mass = syst.get_bodies()[i].mass;
        const double second_mass = syst.get_bodies()[j].mass;
        const double mass_tot = first_mass + second_mass;
        const double radius_tot =
            syst.get_bodies()[i].radius + syst.get_bodies()[j].radius;
        syst.get_bodies()[i].mass = mass_tot;
        syst.get_bodies()[i].radius = radius_tot;
        syst.get_bodies()[i].acc = {0., 0.};
        syst.get_bodies()[i].pos = (syst.get_bodies()[i].pos * first_mass +
                                    syst.get_bodies()[j].pos * second_mass) /
                                   mass_tot;
        syst.get_bodies()[i].vel = (syst.get_bodies()[i].vel * first_mass +
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
  sys.linearMomentum();
  sys.angularMomentum();
  sys.kineticEnergy();
  sys.potentialEnergy();
  sys.totalEnergy();
};

void System::update_body_count() { n_bodies = bodies.size(); }
