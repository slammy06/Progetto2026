#include <fstream>
#include <iostream>
#include <string>

#include "graphics.hpp"
#include "main.hpp"

const float dt = 0.10f;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Specificare il file di configurazione.\n";
    return 1;
  }
  // reading data from config.txt
  int N = 0;
  int t = 0;
  std::vector<double> masses;
  std::vector<project::point<double>> positions;
  std::vector<project::point<double>> velocities;
  std::vector<double> rads;

  std::ifstream config(argv[1]);
  if (config.is_open()) {
    if (!(config >> N >> t) || N <= 0 || t < 0) {
      std::cerr << "Configurazione non valida: attesi N > 0 e t >= 0.\n";
      return 1;
    }
    const std::size_t bodyCount = static_cast<std::size_t>(N);
    masses.resize(bodyCount);
    positions.resize(bodyCount);
    velocities.resize(bodyCount);
    rads.resize(bodyCount);
    for (std::size_t i = 0; i < bodyCount; ++i) {
      config >> masses[i];
    }
    for (std::size_t i = 0; i < bodyCount; ++i) {
      config >> rads[i];
    }
    for (std::size_t i = 0; i < bodyCount; ++i) {
      config >> positions[i].x;
      config >> positions[i].y;
    }
    for (std::size_t i = 0; i < bodyCount; ++i) {
      config >> velocities[i].x;
      config >> velocities[i].y;
    }
  } else {
    std::cerr << "Impossibile aprire il file: " << argv[1] << "\n";
    return 1;
  }

  config.close();
  // Initializing simulation and system with data from config.txt

  project::System sys(static_cast<std::size_t>(N), masses, positions,
                      velocities, rads);
  sys.compute_acceleration();
  project::Sim sim;
  sim.initBodies(sys);

  for (int step = 0; step < t; ++step) {
    sim.update();
    if (!sim.running()) break;
   
    project::step(sys, dt);
    sim.initBodies(sys);  
    sim.render();
    if(step % 20 == 0){
          sim.display_chart(sys.get_lin_momentum(), sys.get_ang_momentum(),
              sys.get_totEnergy());
    }

      
  }

    while (sim.running() || sim.graphing()) {
    sim.pollEvents();
  }
}
