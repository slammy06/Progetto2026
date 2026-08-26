#ifndef PROJECT_MAIN_HPP
#define PROJECT_MAIN_HPP

#include <cmath>
#include <vector>

namespace project {
template <typename T1>

struct point {
  T1 x;
  T1 y;
  T1 norm() const { return std::sqrt(x * x + y * y); }

  point operator+(const point<T1>& p) { return point<T1>{x + p.x, y + p.y}; }
  point operator-(const point<T1>& p) { return point<T1>{x - p.x, y - p.y}; }
  point& operator+=(const point<T1>& p) {
    x += p.x;
    y += p.y;
    return *this;
  }
  point operator*(const float& a) { return point<T1>{x * a, y * a}; }
  point operator/(const float& a) { return point<T1>{x / a, y / a}; }
};

struct Body {
  point<float> pos{0.0f, 0.0f};
  point<float> vel{0.0f, 0.0f};
  point<float> acc{0.0f, 0.0f};
  float mass{0.0f};
  float radius{0.0f};
};

class System {
  std::vector<Body> bodies;
  std::vector<float> kinetic;
  std::vector<float> potential;
  std::vector<float> totEnergy;
  std::vector<point<float>> lin_momentum;
  std::vector<float> ang_momentum;
  long unsigned int n_bodies{0};

 public:
  // Constructors

  System(long unsigned int n, std::vector<float> const& masses,
         std::vector<point<float>> const& pos,
         std::vector<point<float>> const& vel, std::vector<float> const& rads);
  System(std::vector<Body> const& in_bodies);
  // Accessors

  auto& get_bodies() { return bodies; }

  auto const& get_bodies() const { return bodies; }

  auto& get_totEnergy() { return totEnergy; }

  auto get_accelerations();

  auto get_lin_momentum() { return lin_momentum; }
  auto get_ang_momentum() { return ang_momentum; }
  // Methods

  void compute_acceleration();
  void kineticEnergy();
  void potentialEnergy();
  void totalEnergy();
  void linearMomentum();
  void angularMomentum();
  void update_body_count();
};
// Functions
void vel_verlet(System& sys,
                float dt);  // Defines one step for a system type object
std::vector<point<float>>
generate_points();  // Generates a vector of random points

void collided(
    System& syst);  // Definies when a collision is detected and its behaviour

void step(System& sys, float dt);

std::vector<point<float>> generate_points(int n, float min, float max);

};  // namespace project

#endif