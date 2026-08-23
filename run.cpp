#include <fstream>
#include <iostream>
#include <string>

#include "graphics.cpp"
#include "main.cpp"

const float P_MAX = 800;
const float V_MAX = 1000;
const float V_MIN = -1000;
const float dt = 0.01f;

int main(int argc, char* argv[]) {
  if ( argc < 2) {
    std::cerr << "Specificare il file di configurazione.\n";
    return 1;
  }

  // reading data from config.txt
  int N, t;
 std::vector<float> masses;
  std::vector<point<float>> positions;
  std::vector<point<float>> velocities;
  std::vector<float> rads;

  std::ifstream config(argv[1]);
  if (config.is_open()) {
    config >> N >> t;
    masses.resize(N);
    positions.resize(N);
    velocities.resize(N);
    rads.resize(N);
    for (int i = 0; i < N; ++i) {
      config >> masses[i];
    }
    for (int i = 0; i < N; ++i) {
      config >> rads[i];
    }
    for (int i = 0; i < N; ++i) {
      config >> positions[i].x;
      config >> positions[i].y;
    }
        for (int i = 0; i < N; ++i) {
      config >> velocities[i].x;
      config >> velocities[i].y;
    }
  } else{
    std::cerr << "Impossibile aprire il file: " << argv[1] << "\n";
    return 1;
  } 

  config.close();
  // Initializing simulation and system with data from config.txt

  System sys(N, masses, positions, velocities, rads);
  Sim sim;
  float accumulator = 0.0f;
  // Simulation Loop

  while (sim.running()) {
    while (accumulator <= t) {
      // Update
      project::step(sys, dt);
      sim.update();
      sim.initBodies(sys);
      // Render
      sim.render();
    }
    sys.totalEnergy();
  }

}