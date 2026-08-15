#include <iostream>
#include <vector>
namespace project {
template <typename T1>
struct point {
  T1 x;
  T1 y;
 };

template <typename T1>
  point<T1> operator+(point<T1> const& a, point<T1> const& b){
return {a.x + b.x, a.y + b.y};
}
template <typename T1>
point<T1> operator-(point<T1> const& a, point<T1> const& b){
return {a.x - b.x, a.y - b.y};
}
template <typename T1>
T1 operator*(point<T1> const& a, point<T1> const& b){
return a.x * b.x + a.y * b.y;
}
template <typename T1>
point<T1> operator*(point<T1> const& a, float const& b){
return {a.x * b,  a.y * b};
}
template <typename T1>
point<T1> operator*(point<T1> const& a, double const& b){
return {a.x * b,  a.y * b};
}
template <typename T1>
point<T1> operator*(point<T1> const& a, int const& b){
return {a.x * b,  a.y * b};
}
template <typename T1>
point<T1> operator/(point<T1> const& a, double const& b){
return {a.x * b,  a.y * b};
}
 



class body {
  point<double> pos{0.0, 0.0};
  point<double> vel{0.0, 0.0};
  point<double> acc{0.0, 0.0};
  double mass{0.0};
  double radius{0.0};

 public:
  body(point<double> const& p, point<double> const& v, point<double> const& a,
       double m, double r) : pos(p), vel(v), acc(a), mass(m), radius(r) {};

  point<double> get_pos() const {
       return pos;
  }
  point<double> get_vel() const {
       return vel;
  }
  point<double> get_acc() const {
       return acc;
  }
  double get_mass() const {
       return mass;
  }

};

double distance(point<double> const& a,point<double> const& b){
       return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
};

class system{
       std::vector<body> bodies;
       float step{0.0};
       int time{0};
       bool is_default{false};
       bool is_random{false};

       void add(body const& b){
              bodies.push_back(b);
       }

       public: 
       system(float s, int t, float m, float r, int n): step(s), time(t) {
        for(int i = 0; i<n; i++){
        }
       }
       void evolve(){
switch (is_default){
    case true:

}
       }

};


}  // namespace project