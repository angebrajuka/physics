#define VERBOSE
#define CATCH_OBJECT_OVERFLOW
#include "physics.h"
#include <stdio.h>

int main(int argc, char **argv) {
    SDL_version version;
    SDL_GetVersion(&version);
    printf("running SDL version %d.%d.%d\n", version.major, version.minor, version.patch);

    simulation simulation = {
        .tick_rate=60,
        .gravity={0, 0.098},
        .air_resistance=0
    };

    simulation_add_m_obj(&simulation, (m_obj){
        .pos={0, 0},
        .vel={1, 0},
        .collider=make_collider(3, (double[]){0, 0, 0, 100, 100, 100}),
        .material={0.7, 0, 0}
    });
    simulation_add_s_obj(&simulation, (s_obj){
        .pos={0, 470},
        .collider=make_collider(4, (double[]){0, 0, 0, 10, 600, 10, 600, 0}),
        .material={1, 0, 0}
    });

    begin_loop(&simulation, 640, 480, 0);
    return 0;
}