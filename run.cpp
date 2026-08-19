#include <fstream>
#include <iostream>
#include <string>

#include "main.cpp"
#include "main.hpp"
const float pmax = 100000;
const float pmin = 0;
const float vmax = 1000;
const float vmin = -1000;

int main() {
  // lettura dati dal file config.txt
  int n, t;
  std::vector<float> masses;
  std::fstream config;
  config.open("config.txt", std::ios::in);
  if (config.is_open()) {
    config >> n >> t;
    for (int i = 0; i < n; ++i) {
      config >> masses[i];
    }
  }

  config.close();
  //generazione delle restanti condizioni iniziali e inizializzazione del sistema
  std::vector<point<float>> positions = generate_points(n, pmin, pmax);
  std::vector<point<float>> velocities = generate_points(n, vmin, vmax);

  project::system s(n, masses, positions, velocities);

  // 
}