#ifndef PROJECT_MAIN_HPP
#define PROJECT_MAIN_HPP

#include <vector>
#include <cmath>
namespace project {
template <typename T1>

struct point {
  T1 x;
  T1 y;
  T1 norm() { return std::sqrt(x * x + y * y); }

  point operator+(const point<T1>& p) { return point<T1>{x + p.x, y + p.y}; };
  point operator-(const point<T1>& p) { return point<T1>{x - p.x, y - p.y}; };
  point& operator+=(const point<T1>& p) {
    x += p.x;
    y += p.y;
    return *this;
  }
  point operator*(const float& a) { return point<T1>{x * a, y* a}; };
  point operator/(const float& a) { return point<T1>{x / a, y / a}; };
};

struct check {
  int i;
  int j;
  bool crash;
};

struct Body {
  point<float> pos{0.0, 0.0};
  point<float> vel{0.0, 0.0};
  point<float> acc{0.0, 0.0};
  float mass{0.0};
  float radius{0.0};
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

  System(int n, std::vector<float> const& masses,
         std::vector<point<float>> const& pos,
         std::vector<point<float>> const& vel, std::vector<float> const& rads);
  System(std::vector<Body> in_bodies);
  // Accessors

  auto& get_bodies() { return bodies; }

  auto const& get_bodies() const { return bodies; }

  auto& get_totEnergy() { return totEnergy; }

  auto get_accelerations();

  auto get_lin_momentum() {return lin_momentum; }
  auto get_ang_momentum() {return ang_momentum; }

  // Methods

  void compute_acceleration();
  void kineticEnergy();
  void potentialEnergy();
  void totalEnergy();
  void linearMomentum();
  void angularMomentum();
};
// Functions

std::vector<point<float>> generate_points(
    int n);  // Generates a vector of random points

check collision_check(System const& sys, int i);  // Checks the system for collisions
void collided(System& syst, int i, int j);          // Definies the behaviour of colliding bodies
void vel_verlet(System& sys, float dt);  // Defines one step for a system type object
void step(System& sys, float dt);
};  // namespace project

#endif