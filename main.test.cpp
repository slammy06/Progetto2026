#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "main.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include "doctest.h"

TEST_CASE("1 Planet - Static") {
  std::vector<project::Body> bodies{
      project::Body{0., 0., 0., 0., 0., 0., 5., 4.}};
  int dt = 0;
  project::System sys(bodies);
  while (dt < 3000) {
    project::step(sys, 0.1f);



    dt++;
  }
      CHECK(sys.get_bodies()[0].acc.x == doctest::Approx(0));
    CHECK(sys.get_bodies()[0].acc.y == doctest::Approx(0));
    CHECK(sys.get_bodies()[0].mass == doctest::Approx(5));
    CHECK(sys.get_bodies()[0].vel.x == doctest::Approx(0));
    CHECK(sys.get_bodies()[0].vel.y == doctest::Approx(0));
    CHECK(sys.get_bodies()[0].pos.x == doctest::Approx(0));
    CHECK(sys.get_bodies()[0].pos.y == doctest::Approx(0));
    CHECK(sys.get_lin_momentum()[0].x == doctest::Approx(0));
    CHECK(sys.get_lin_momentum()[0].y == doctest::Approx(0));
    CHECK(sys.get_ang_momentum()[0] == doctest::Approx(0));
}
TEST_CASE("1 Planet - Linear") {
  std::vector<project::Body> bodies{
      project::Body{0., 10., 1., 0., 0., 0., 5., 4.}};
  int dt = 0;
  project::System sys(bodies);
  while (dt < 30000) {
    project::step(sys, 0.1f);

    dt++;
  }
  CHECK(sys.get_bodies()[0].acc.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].acc.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].mass == doctest::Approx(5));
  CHECK(sys.get_bodies()[0].vel.x == doctest::Approx(1));
  CHECK(sys.get_bodies()[0].vel.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].pos.x == doctest::Approx(3000).epsilon(0.001f));
  CHECK(sys.get_bodies()[0].pos.y == doctest::Approx(10));
  CHECK(sys.get_lin_momentum()[0].x == doctest::Approx(5));
  CHECK(sys.get_lin_momentum()[0].y == doctest::Approx(0));
  CHECK(sys.get_ang_momentum()[0] == doctest::Approx(-50));
}

TEST_CASE("Figure 8 configuration") {
  std::vector<double> masses{1.0, 1.0, 1.0};
  std::vector<double> radii{10.0, 10.0, 10.0};
  std::vector<project::point<double>> positions{
      {441.7997384, 514.5852518},
      {558.2002616, 485.4147482},
      {500.0, 500.0}};
  std::vector<project::point<double>> velocities{
      {0.601866369, 0.558181757},
      {0.601866369, 0.558181757},
      {-1.203732738, -1.116363514}};

  project::System sys(3, masses, positions, velocities, radii);
  sys.compute_acceleration();

  CHECK(sys.get_bodies().size() == 3);
 

  for (int step = 0; step < 20; ++step) {
    project::step(sys, 0.1f);
  }

  CHECK(sys.get_bodies().size() == 3);
  for (const project::Body& body : sys.get_bodies()) {
    CHECK(std::isfinite(body.pos.x));
    CHECK(std::isfinite(body.pos.y));
    CHECK(std::isfinite(body.vel.x));
    CHECK(std::isfinite(body.vel.y));
  }
  CHECK(sys.get_lin_momentum().back().x == doctest::Approx(0.0).epsilon(0.01));
  CHECK(sys.get_lin_momentum().back().y == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("2 Planets - static collision"){
  std::vector<project::Body> bodies{
    project::Body{0., -1., 0., 0., 0., 0., std::sqrt(1.e11), .1}, 
    project::Body{0., 1., 0., 0., 0., 0., std::sqrt(1.e11), .1}};
  int dt = 0;
 
  project::System sys(bodies);
sys.kineticEnergy();
sys.potentialEnergy();
sys.totalEnergy();

   CHECK(sys.get_totEnergy()[0] == doctest::Approx(-3.33715));

  while (dt < 5000) {
    project::step(sys, 0.1f);

    dt++;
  }
 CHECK(sys.get_totEnergy()[static_cast<std::size_t>(dt)] == doctest::Approx(0.));
   CHECK(sys.get_bodies()[0].pos.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].pos.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].acc.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].acc.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].mass == doctest::Approx(2*std::sqrt(1.e11)));
  CHECK(sys.get_bodies()[0].vel.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].vel.y == doctest::Approx(0));
}

TEST_CASE("2 Planets - frontal collision") {
  std::vector<project::Body> bodies{
    project::Body{0., -1., 0., 1., 0., 0., std::sqrt(1.e11), .1},
    project::Body{0.,  1., 0., -1., 0., 0., std::sqrt(1.e11), .1}
  };

  int dt = 0;
  project::System sys(bodies);

  sys.kineticEnergy();
  sys.potentialEnergy();
  sys.totalEnergy();

  while (dt < 5000) {
    project::step(sys, 0.1f);
    dt++;
  }

  CHECK(sys.get_bodies().size() == 1);
  CHECK(sys.get_bodies()[0].pos.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].pos.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].vel.x == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].vel.y == doctest::Approx(0));
  CHECK(sys.get_bodies()[0].mass ==
        doctest::Approx(2 * std::sqrt(1.e11)));
}


TEST_CASE("2 Planets - different masses collision") {
  std::vector<project::Body> bodies{
    project::Body{0., -1., 0., 0., 0., 0., std::sqrt(1.e11), .1},
    project::Body{0.,  1., 0., 0., 0., 0., 2 * std::sqrt(1.e11), .1}
  };

  int dt = 0;
  project::System sys(bodies);

  sys.kineticEnergy();
  sys.potentialEnergy();
  sys.totalEnergy();

  while (dt < 5000) {
    project::step(sys, 0.1f);
    dt++;
  }

  CHECK(sys.get_bodies().size() == 1);
  CHECK(sys.get_bodies()[0].mass ==
        doctest::Approx(3 * std::sqrt(1.e11)));
}
TEST_CASE("2 Planets - immediate collision") {
  std::vector<project::Body> bodies{
    project::Body{0., 0., 0., 0., 0., 0., std::sqrt(1.e11), .1},
    project::Body{0., 0.15, 0., 0., 0., 0., std::sqrt(1.e11), .1}
  };


  project::System sys(bodies);

  project::step(sys, 0.1f);

  CHECK(sys.get_bodies().size() == 1);
  CHECK(sys.get_bodies()[0].mass ==
        doctest::Approx(2 * std::sqrt(1.e11)));
}
TEST_CASE("3 Planets - two collide") {
  std::vector<project::Body> bodies{
    project::Body{0., -1., 0., 0., 0., 0., std::sqrt(1.e11), .1},
    project::Body{0.,  1., 0., 0., 0., 0., std::sqrt(1.e11), .1},
    project::Body{10., 10., 0., 0., 0., 0., std::sqrt(1.e11), .1}
  };

  int dt = 0;

  project::System sys(bodies);

  while (dt < 5000) {
    project::step(sys, 0.1f);
    dt++;
  }

  CHECK(sys.get_bodies().size() == 2);
}


