#include <iostream>
#include <vector>
namespace project {
template <typename T1>

struct point {
  T1 x;
  T1 y;
 };

class body {
  point<float> pos{0.0, 0.0};
  point<float> vel{0.0, 0.0};
  point<float> acc{0.0, 0.0};
  float mass{0.0};
  float radius{0.0};

 public:
  body(point<float> const& p, point<float> const& v, point<float> const& a,
       float m, float r) : pos(p), vel(v), acc(a), mass(m), radius(r) {};
};


class system{
       std::vector<body> bodies;
       float step{0.0};
       int time{0};
       bool is_default{false};
       bool is_random{false};
       
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

}


}  // namespace project