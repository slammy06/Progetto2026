#ifndef PROJECT_MAIN_HPP
#define PROJECT_MAIN_HPP
#include "main.hpp"

//IDEA DI COME FARE IL VELOCITY VERLET, PRIMA DOBBIAMO FARE DEI VECTOR PER OGNI COMPONENTE CON DEI PUSH DAL SISTEMA DI BODIES

{
    auto pos = body.pos
    auto time = 0;
    auto acc = body.acc
    auto vel = body.vel
    auto dt = 0;

    while(time<tmax){
        for(i=0; i<system.max, i++){
        pos[i] = pos[i] + vel[i] * dt + acc[i] * dt * dt
        for(j=0; j<system.max,, j++){
            sum_acc = G * mass[j] * (pos[i] - pos[j])/ ((pos[j]-pos[i])^(2) + e^(2))^(3/2);


        }
        acc[i] = - sum_acc;
        vel[i] = vel[i] + 0.5 * (body.acc[i] + acc[i]) * dt;  


        }
      


        dt++;
    }


}

#endif