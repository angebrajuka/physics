#pragma once

#include <stdbool.h>
#include <SDL2/SDL.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>

#ifdef DEBUG
#define VERBOSE
#define DEBUG_SHOW_LAST_COLLISION
#define DEBUG_SHOW_CG
#endif


#ifdef VERBOSE
#include <stdio.h>
#endif

#ifndef MAX_MOBJ_COUNT
#define MAX_MOBJ_COUNT 32
#endif

#ifndef MAX_SOBJ_COUNT
#define MAX_SOBJ_COUNT 128
#endif

#ifndef MAX_COLLIDER_VERTICES
#define MAX_COLLIDER_VERTICES 16
#endif

#ifndef SIMULATION_STEPS
#define SIMULATION_STEPS 32
#endif

#if defined(USE_BLOCKMAP) || defined(BLOCKMAP_SIZE) || defined(BLOCKMAP_COUNT)
# ifndef BLOCKMAP_SIZE
# define BLOCKMAP_SIZE 128
# endif
# ifndef BLOCKMAP_COUNT
# define BLOCKMAP_COUNT 128
# endif
#endif

#define sqr(x) x*x

typedef struct vector_s {
    double x, y;
} vector_t;

const vector_t zero_vector = {.x=0, .y=0};

vector_t vector_add(vector_t vec1, vector_t vec2) {
    vector_t result;
    result.x = vec1.x+vec2.x;
    result.y = vec1.y+vec2.y;
    return result;
}

vector_t vector_multiply(vector_t vec, double scale) {
    vector_t result;
    result.x = vec.x * scale;
    result.y = vec.y * scale;
    return result;
}

double vector_magnitude(vector_t vec) {
    return sqrt(sqr(vec.x) + sqr(vec.y));
}

vector_t vector_normalize(vector_t vec) {
    return vector_multiply(vec, 1.0/vector_magnitude(vec));
}

// VERTICES ARE COUNTER CLOCKWISE
typedef struct collider_s {
    vector_t vertices[MAX_COLLIDER_VERTICES];
    int vertex_count;
} collider_t;

// ensure minimum 3 vertices
// ensure varargs have a decimal point, (double) cast, or suffix to explicitly tell the compiler they are doubles
collider_t make_collider(int vertex_count, ...) {
    collider_t collider;
    collider.vertex_count = vertex_count;
    va_list args;
    va_start(args, vertex_count);
    int i;
    for(i=0; i < vertex_count; ++i) {
        collider.vertices[i] = (vector_t){va_arg(args, double), va_arg(args, double)};
    }
    va_end(args);
    return collider;
}

collider_t rotate(collider_t original, double angle) {
    collider_t result = {.vertex_count=original.vertex_count};
    int i;
    if(angle ==0) {
        for(i=0; i<original.vertex_count; ++i) {
            result.vertices[i] = original.vertices[i];
        }
        return result;
    }

    angle = 2*M_PI-angle;
    for(i=0; i<original.vertex_count; ++i) {
        result.vertices[i].x = original.vertices[i].x * cos(angle) - original.vertices[i].y * sin(angle);
        result.vertices[i].y = original.vertices[i].x * sin(angle) + original.vertices[i].y * cos(angle);
    }

    return result;
}

typedef struct line_s {
    vector_t start, end;
} line_t;

bool lines_collide(line_t l1, line_t l2, vector_t *collision_point) {
    double uA = ((l2.end.x-l2.start.x)*(l1.start.y-l2.start.y) - (l2.end.y-l2.start.y)*(l1.start.x-l2.start.x))
              / ((l2.end.y-l2.start.y)*(l1.end.x-l1.start.x) - (l2.end.x-l2.start.x)*(l1.end.y-l1.start.y));
    double uB = ((l1.end.x-l1.start.x)*(l1.start.y-l2.start.y) - (l1.end.y-l1.start.y)*(l1.start.x-l2.start.x))
              / ((l2.end.y-l2.start.y)*(l1.end.x-l1.start.x) - (l2.end.x-l2.start.x)*(l1.end.y-l1.start.y));

    if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
        collision_point->x = l1.start.x + (uA * (l1.end.x-l1.start.x));
        collision_point->y = l1.start.y + (uA * (l1.end.y-l1.start.y));
        return true;
    }
    return false;
}

// only use collision_point if function returns true 
bool collides(collider_t c1, vector_t position1, collider_t c2, vector_t position2, vector_t *collision_point, line_t *collision_line) {
    int i, j;
    line_t line_i, line_j;
    for(i=0; i<c1.vertex_count; ++i) {
        line_i.start = vector_add(position1, c1.vertices[i]);
        line_i.end = vector_add(position1, c1.vertices[i == c1.vertex_count-1 ? 0 : i+1]);
        for(j=0; j<c2.vertex_count; ++j) {
            line_j.start = vector_add(position2, c2.vertices[j]);
            line_j.end = vector_add(position2, c2.vertices[j == c2.vertex_count-1 ? 0 : j+1]);
            if(lines_collide(line_i, line_j, collision_point)) {
                *collision_line = line_j;
                return true;
            }
        }
    }
    return false;
}

typedef struct material_s {
    double bounciness;
    double friction_tatic;
    double friction_kinetic;
} material_t;

typedef struct sobj_s {
    vector_t position;
    collider_t collider;
    material_t material;
} sobj_t;

typedef struct mobj_s {
    vector_t position, velocity;
    double angular_velocity;
    collider_t collider;
    material_t material;
    double mass;
} mobj_t;

void mobj_apply_force(mobj_t *mobj, vector_t force, vector_t position) {
    mobj->velocity = vector_add(mobj->velocity, vector_multiply(force, 1.0/mobj->mass));
}

typedef struct simulation_s {
    int tick_rate;
    sobj_t sobjs[MAX_SOBJ_COUNT];
    int sobj_count;
    mobj_t mobjs[MAX_MOBJ_COUNT];
    int mobj_count;
    vector_t gravity;
    double air_resistance;
} simulation_t;

const simulation_t default_simulation = {
    .tick_rate=60,
    .sobj_count=0,
    .mobj_count=0,
    .gravity={0, 0.098},
    .air_resistance=0
};

void simulation_add_mobj(simulation_t *simulation, mobj_t mobj) {
#ifdef CATCH_OBJECT_OVERFLOW
    if(simulation->mobj_count == MAX_MOBJ_COUNT) {
        return;
    }
#endif
    simulation->mobjs[simulation->mobj_count++] = mobj;
}

void simulation_add_sobj(simulation_t *simulation, sobj_t sobj) {
#ifdef CATCH_OBJECT_OVERFLOW
    if(simulation->sobj_count == MAX_SOBJ_COUNT) {
        return;
    }
#endif
    simulation->sobjs[simulation->sobj_count++] = sobj;
}

#ifdef DEBUG_SHOW_LAST_COLLISION
line_t collision_line;
#endif

void tick(simulation_t *simulation) {
    int i, j, step;
    mobj_t *mobj, *mobj_other;
    sobj_t *sobj_other;
    vector_t old_position, collision_point;
    #ifndef DEBUG_SHOW_LAST_COLLISION
    line_t collision_line;
    #endif
    double impact_speed;
    collider_t old_collider;
    bool collided;
    for(step=0; step < SIMULATION_STEPS; ++step) for(i=0; i<simulation->mobj_count; ++i) {
        mobj = &simulation->mobjs[i];
        mobj->velocity = vector_add(mobj->velocity, vector_multiply(simulation->gravity, 1.0/SIMULATION_STEPS));
        collided = false;
        old_position = mobj->position;
        old_collider = mobj->collider;
        mobj->position = vector_add(mobj->position, vector_multiply(mobj->velocity, 1.0/SIMULATION_STEPS));
        mobj->collider = rotate(mobj->collider, mobj->angular_velocity/SIMULATION_STEPS);
        for(j=0; j<simulation->mobj_count; ++j) {
            if(i==j) continue;
            mobj_other = &simulation->mobjs[j];
            if(collides(mobj->collider, mobj->position, mobj_other->collider, mobj_other->position, &collision_point, &collision_line)) {
                collided = true;
                mobj->position = old_position;
                mobj->collider = old_collider;
                // TODO two mobjs colliding
                break;
            }
        }
        for(j=0; j<simulation->sobj_count; ++j) {
            sobj_other = &simulation->sobjs[j];
            if(collides(mobj->collider, mobj->position, sobj_other->collider, sobj_other->position, &collision_point, &collision_line)) {
                collided = true;
                mobj->position = old_position;
                mobj->collider = old_collider;
                impact_speed = vector_magnitude(mobj->velocity);
                mobj->velocity = zero_vector; // TODO add normal force instead of zero to fix slide
                mobj_apply_force(
                    mobj,
                    vector_multiply(
                        vector_normalize((vector_t) {
                            .x=collision_line.start.y-collision_line.end.y, 
                            .y=collision_line.end.x-collision_line.start.x
                        }),
                        impact_speed * mobj->mass * mobj->material.bounciness * sobj_other->material.bounciness
                    ),
                    collision_point
                );

                break;
            }
        }
    }
}

void render_obj(SDL_Renderer *renderer, vector_t position, collider_t c) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    int i;
    for(i=0; i<c.vertex_count-1; ++i) {
        SDL_RenderDrawLine(renderer, c.vertices[i].x+position.x, c.vertices[i].y+position.y, c.vertices[i+1].x+position.x, c.vertices[i+1].y+position.y);
    }
    SDL_RenderDrawLine(renderer, c.vertices[c.vertex_count-1].x+position.x, c.vertices[c.vertex_count-1].y+position.y, c.vertices[0].x+position.x, c.vertices[0].y+position.y);
#ifdef DEBUG_SHOW_CG
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_Rect rect = {(int)position.x-1, (int)position.y-1, 4, 4};
    SDL_RenderFillRect(renderer, &rect);
#endif
}

void render(simulation_t *simulation, SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    int i;
    for(i=0; i<simulation->mobj_count; ++i) {
        render_obj(renderer, simulation->mobjs[i].position, simulation->mobjs[i].collider);
    }
    for(i=0; i<simulation->sobj_count; ++i) {
        render_obj(renderer, simulation->sobjs[i].position, simulation->sobjs[i].collider);
    }

#ifdef DEBUG_SHOW_LAST_COLLISION
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawLine(renderer, collision_line.start.x, collision_line.start.y, collision_line.end.x, collision_line.end.y);
    SDL_Rect rect = {(int)collision_line.start.x-1, (int)collision_line.start.y-1, 4, 4};
    SDL_RenderFillRect(renderer, &rect);
#endif
}

int begin_loop(simulation_t *simulation, int windowWidth, int windowHeight, Uint32 flags) {
    if(SDL_Init(SDL_INIT_VIDEO) < 0) {
#ifdef VERBOSE
        printf("SDL2 failed to initialize video\n");
        printf("%s\n", SDL_GetError());
#endif
        return -1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "SDL2 Window",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        windowWidth, windowHeight,
        flags
    );
    if(!window) {
#ifdef VERBOSE
        printf("SDL2 failed to create window\n");
        printf("%s\n", SDL_GetError());
#endif
        return -1;
    }

    SDL_Surface *window_surface = SDL_GetWindowSurface(window);

    if(!window_surface)
    {
#ifdef VERBOSE
        printf("SDL2 failed to get the surface from the window\n");
        printf("%s\n", SDL_GetError());
#endif
        return -1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, 0, 0);
    if(!renderer) {
#ifdef VERBOSE
        printf("SDL2 failed to create a renderer\n");
        printf("%s\n", SDL_GetError());
#endif
        return -1;
    }

    bool running = true;
    clock_t start, stop;
    while(running) {
        start = clock();

        SDL_Event e;
        while(SDL_PollEvent(&e) > 0) {
            switch(e.type) {
                case SDL_QUIT:
                    running = false;
                    break;
            }
        }

        tick(simulation);
        render(simulation, renderer);

        SDL_RenderPresent(renderer);

        stop = clock();
        int delay_time = 1000/simulation->tick_rate - (stop-start);
        if(delay_time > 0) {
            SDL_Delay(delay_time);
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}