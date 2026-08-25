#include <fstream>
#include <iostream>
#include <string>

#include "graphics.hpp"
#include "main.hpp"

const float P_MAX = 800;
const float V_MAX = 1000;
const float V_MIN = -1000;
const float dt = 0.01f;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Specificare il file di configurazione.\n";
    return 1;
  }
  // reading data from config.txt
  int N = 0;
  int t = 0;
  std::vector<float> masses;
  std::vector<project::point<float>> positions;
  std::vector<project::point<float>> velocities;
  std::vector<float> rads;

  std::ifstream config(argv[1]);
  if (config.is_open()) {
    if (!(config >> N >> t) || N <= 0 || t < 0) {
      std::cerr << "Configurazione non valida: attesi N > 0 e t >= 0.\n";
      return 1;
    }
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
  } else {
    std::cerr << "Impossibile aprire il file: " << argv[1] << "\n";
    return 1;
  }

  config.close();
  // Initializing simulation and system with data from config.txt

  project::System sys(N, masses, positions, velocities, rads);
  sys.compute_acceleration();
  project::Sim sim;
  sim.initBodies(sys);
while(sim.running()){
  // Simulation Loop: t è la durata della simulazione in secondi.
  for (float elapsed = 0.0f; elapsed < static_cast<float>(t);
       elapsed += dt) {
    sim.update();
    if (!sim.running()) break;

    project::step(sys, dt);
    sim.initBodies(sys);
    sim.render();
  }
  sim.display_chart(sys.get_lin_momentum(),sys.get_ang_momentum());
}
}
