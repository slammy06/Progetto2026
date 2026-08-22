#include <fstream>
#include <iostream>
#include <string>

#include "graphics.cpp"
#include "main.cpp"

const float P_MAX = 800;
const float V_MAX = 1000;
const float V_MIN = -1000;
const float dt = 0.01f;

int main() {
  // reading data from config.txt
  int n, t;
  std::vector<float> masses;
  std::vector<point<float>> positions;
  std::vector<point<float>> velocities;
  std::vector<float> rads;

  std::ifstream config;
  config.open("config.txt", std::ios::in);
  if (config.is_open()) {
    config >> n >> t;
    for (int i = 0; i < n; ++i) {
      config >> masses[i];
    }
    for (int i = 0; i < n; ++i) {
      config >> rads[i];
    }
    for (int i = 0; i < n; ++i) {
    }
  }

  config.close();
  // Initializing simulation and system with data from config.txt

  System sys(n, masses, positions, velocities, rads);
  Sim sim;
  float accumulator = 0.0f;
  // Simulation Loop

  while (sim.running()) {
    while (accumulator <= t) {
      // Update
      sys.kineticEnergy();
      sys.potentialEnergy();
      sim.update();
      vel_verlet(sys, dt);
      sim.initBodies(sys);
      // Render
      sim.render();
    }
    sys.totalEnergy();
  }
}