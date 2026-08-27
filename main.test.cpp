#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "main.cpp"

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

    dt++;
  }
}
TEST_CASE("1 Planet - Linear") {
  std::vector<project::Body> bodies{
      project::Body{0., 10., 1., 0., 0., 0., 5., 4.},
      project::Body{0., 0., 0., 0., 0., 0., 0., 0.}};
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
  std::vector<double> masses{1.0f, 1.0f, 1.0f};
  std::vector<double> radii{10.0f, 10.0f, 10.0f};
  std::vector<project::point<double>> positions{
      {441.7997384f, 514.5852518f},
      {558.2002616f, 485.4147482f},
      {500.0f, 500.0f}};
  std::vector<project::point<double>> velocities{
      {0.601866369f, 0.558181757f},
      {0.601866369f, 0.558181757f},
      {-1.203732738f, -1.116363514f}};

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
  CHECK(sys.get_lin_momentum().back().x == doctest::Approx(0.0f).epsilon(0.01f));
  CHECK(sys.get_lin_momentum().back().y == doctest::Approx(0.0f).epsilon(0.01f));
}
