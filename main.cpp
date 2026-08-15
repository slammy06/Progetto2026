#ifndef PROJECT_MAIN_HPP
#define PROJECT_MAIN_HPP
#include "main.hpp"
#include <cmath>
#include <vector>
#include <iostream>

//IDEA DI COME FARE IL VELOCITY VERLET, PRIMA DOBBIAMO FARE DEI VECTOR PER OGNI COMPONENTE CON DEI PUSH DAL SISTEMA DI BODIES
std::vector<project::body> bodies;
std::vector<project::point<double>> pos;
std::vector<project::point<double>> acc;
std::vector<project::point<double>> vel;
std::vector<double> mass;
int N; //vooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
int tmax; //cnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn
void BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB(){
    for(int i=0; i<N; i++){
        pos.push_back(bodies[i].get_pos());
        acc.push_back(bodies[i].get_acc());
        vel.push_back(bodies[i].get_vel());
        mass.push_back(bodies[i].get_mass());
}
    project::point<double> sum_acc{0., 0.};
    project::point<double> point0{0., 0.};
    auto time{0.};
    float dt{0.};
    double E{0.};
    auto soft = std::pow(10, -12);
    const double G = 6.6743 * std::pow(10, -11);
    
    while(time<tmax){

    double K{0.};
    double U{0.};

        for(int i=0; i<N; i++){
        pos[i] = pos[i] + vel[i] * dt + acc[i] * dt * dt * 0.5;
        for(int j=0; j<N; j++){
            sum_acc = (pos[i] - pos[j]) * mass[j] * G / std::pow((pos[j] - pos[i]) *(pos[j] - pos[i]) + soft * soft, 3.0f/2.0f);
          if (i<j){
             U = G * mass[i] * mass[j] / project::distance(pos[j], pos[i]);
          } 

        }
        acc[i] = point0 - sum_acc;
        vel[i] = vel[i] + (bodies[i].get_acc() + acc[i]) * dt * 0.5 ;  

        K = bodies[i].get_vel() * bodies[i].get_vel() * bodies[i].get_mass()  * 0.5;
        
        }
        
        E = K - U;

        dt++;
    }


}

#endif