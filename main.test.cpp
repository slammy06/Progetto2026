#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "main.hpp"
#include "doctest.h"
#include <fstream>
#include <iostream>
#include <string>

TEST_CASE("1 Planet - Static"){
    std::vector<project::Body> bodies{
        project::Body {0., 0., 0., 0., 0., 0., 5., 4.}
    };
    int dt;
    project::System sys(bodies);
    while( dt< 3000){
    project::step(sys, 0.1);
    
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
TEST_CASE("1 Planet - Linear"){
    std::vector<project::Body> bodies{
        project::Body {0., 10., 1., 0., 0., 0., 5., 4.},
        project::Body {0., 0., 0., 0., 0., 0., 0., 0.}
    };
    int dt;
    project::System sys(bodies);
    while( dt< 30000){
    project::step(sys, 0.1);
    



    dt++;
}
   CHECK(sys.get_bodies()[0].acc.x == doctest::Approx(0));
   CHECK(sys.get_bodies()[0].acc.y == doctest::Approx(0));
   CHECK(sys.get_bodies()[0].mass == doctest::Approx(5));
   CHECK(sys.get_bodies()[0].vel.x == doctest::Approx(1));
   CHECK(sys.get_bodies()[0].vel.y == doctest::Approx(0));
   CHECK(sys.get_bodies()[0].pos.x == doctest::Approx(3000));
   CHECK(sys.get_bodies()[0].pos.y == doctest::Approx(10));
    CHECK(sys.get_lin_momentum()[0].x == doctest::Approx(5));
    CHECK(sys.get_lin_momentum()[0].y == doctest::Approx(0));
   CHECK(sys.get_ang_momentum()[0] == doctest::Approx(1));
}

