#define VERBOSE
#define DEBUG_SHOW_LAST_COLLISION
#define DEBUG_SHOW_CG
#define CATCH_OBJECT_OVERFLOW
#define DEBUG_SHOW_LAST_COLLISION
#define SIMULATION_STEPS 32
#include "physics.h"
#include <stdio.h>

int main(int argc, char **argv) {
    simulation_t simulation = {
        .tick_rate = 45,
        .gravity = {0, 0.098},
        .air_resistance = 0
    };

    simulation_add_mobj(&simulation, (mobj_t){
        .position = {0, 200},
        .velocity = {10, 0},
        .angular_velocity = 0.005,
        .collider = make_collider(4,
            -40.0, -40.0,
            -40.0, 40.0,
            40.0, 40.0,
            40.0, -40.0
        ),
        .material = {
            .bounciness = 0,
            .friction_static = 0,
            .friction_kinetic = 0
        },
        .mass = 1
    });
    collider_t original = make_collider(3,
        100.0, 0.0,
        0.0, 100.0,
        100.0, 100.0
    );
    collider_t rotated = rotate(original, M_PI/4);
    simulation_add_sobj(&simulation, (sobj_t){
        .position = {150, 200},
        .collider = original,
        .material = {
            .bounciness = 0,
            .friction_static = 0,
            .friction_kinetic = 0
        }
    });
    simulation_add_sobj(&simulation, (sobj_t){
        .position = {300, 100},
        .collider = rotated,
        .material = {
            .bounciness = 0.8,
            .friction_static = 0,
            .friction_kinetic = 0
        }
    });

    return begin_loop(&simulation, 640, 480, 0);
}
