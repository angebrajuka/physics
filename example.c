#define VERBOSE
#define CATCH_OBJECT_OVERFLOW
#include "physics.h"
#include <stdio.h>

int main(int argc, char **argv) {
    SDL_version version;
    SDL_GetVersion(&version);
    printf("running SDL version %d.%d.%d\n", version.major, version.minor, version.patch);

    simulation_t simulation = {
        .tick_rate=60,
        .gravity={0, 0.098},
        .air_resistance=0
    };

    simulation_add_mobj(&simulation, (mobj_t){
        .position={0, 0},
        .velocity={1, 0},
        .angular_velocity=0.01,
        .collider=make_collider(3,
            0.0, 0.0,
            0.0, 100.0,
            100.0, 100.0
        ),
        .material={0.7, 0, 0}
    });
    collider_t original = make_collider(3,
        0.0, 0.0,
        0.0, 100.0,
        100.0, 100.0
    );
    collider_t rotated = rotate(original, M_PI/4);
    // simulation_add_sobj(&simulation, (sobj_t){
    //     .position={10, 100},
    //     .collider=original,//make_collider(4,  0.0, 0.0, 0.0, 10.0, 600.0, 10.0, 600.0, 0.0),
    //     .material={1, 0, 0}
    // });
    simulation_add_sobj(&simulation, (sobj_t){
        .position={300, 100},
        .collider=rotated,
        .material={1, 0, 0}
    });

    begin_loop(&simulation, 640, 480, 0);
    return 0;
}