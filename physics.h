#pragma once

#include <stdbool.h>
#include <SDL2/SDL.h>
#include <time.h>

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

#ifdef USE_BLOCKMAP
# ifndef BLOCKMAP_SIZE
# define BLOCKMAP_SIZE 128
# endif
# ifndef BLOCKMAP_COUNT
# define BLOCKMAP_COUNT 128
# endif
#endif

typedef struct vector {
    double x, y;
} vector;

vector vector_add(vector vec1, vector vec2) {
    vector result;
    result.x = vec1.x+vec2.x;
    result.y = vec1.y+vec2.y;
    return result;
}

vector vector_multiply(vector vec, double scale) {
    vector result;
    result.x = vec.x * scale;
    result.y = vec.y * scale;
    return result;
}

// VERTICES ARE COUNTER CLOCKWISE
typedef struct collider {
    vector vertices[MAX_COLLIDER_VERTICES];
    int vertex_count;
} collider;

// ensure minimum 3 vertices
collider make_collider(int vertex_count, double *vertex_positions) {
    collider c;
    c.vertex_count = vertex_count;
    int i;
    vector pos;
    for(i=0; i < vertex_count; ++i) {
        pos.x = vertex_positions[2*i];
        pos.y = vertex_positions[2*i+1];
        c.vertices[i] = pos;
    }
    return c;
}

typedef struct line {
    vector start, end;
} line;

bool lines_collide(line l1, line l2, vector *collision_point) {
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
bool collides(collider c1, vector pos1, collider c2, vector pos2, vector *collision_point) {
    int i, j;
    line line_i, line_j;
    for(i=0; i<c1.vertex_count; ++i) {
        line_i.start = vector_add(pos1, c1.vertices[i]);
        line_i.end = vector_add(pos1, c1.vertices[i == c1.vertex_count-1 ? 0 : i+1]);
        for(j=0; j<c2.vertex_count; ++j) {
            line_j.start = vector_add(pos2, c2.vertices[j]);
            line_j.end = vector_add(pos2, c2.vertices[j == c2.vertex_count-1 ? 0 : j+1]);
            if(lines_collide(line_i, line_j, collision_point)) {
                return true;
            }
        }
    }
    return false;
}

typedef struct material {
    double bounciness;
    double friction_static;
    double friction_kinetic;
} material;

typedef struct s_obj {
    vector pos;
    collider collider;
    material material;
} s_obj;

typedef struct m_obj {
    vector pos, vel;
    collider collider;
    material material;
} m_obj;

typedef struct simulation {
    int tick_rate;
    s_obj s_objs[MAX_SOBJ_COUNT];
    int s_obj_count;
    m_obj m_objs[MAX_MOBJ_COUNT];
    int m_obj_count;
    vector gravity;
    double air_resistance;
} simulation;

simulation default_simulation = {
    .tick_rate=60,
    .s_obj_count=0,
    .m_obj_count=0,
    .gravity={0, 0.098},
    .air_resistance=0
};

void simulation_add_m_obj(simulation *sim, m_obj obj) {
#ifdef CATCH_OBJECT_OVERFLOW
    if(sim->m_obj_count == MAX_MOBJ_COUNT) {
        return;
    }
#endif
    sim->m_objs[sim->m_obj_count++] = obj;
}

void simulation_add_s_obj(simulation *sim, s_obj obj) {
#ifdef CATCH_OBJECT_OVERFLOW
    if(sim->s_obj_count == MAX_SOBJ_COUNT) {
        return;
    }
#endif
    sim->s_objs[sim->s_obj_count++] = obj;
}

void tick(simulation *sim) {
    int i, j, step;
    double steps = 100;
    m_obj *m_obj_i, *m_obj_j;
    s_obj *s_obj_j;
    vector collision_point;
    for(i=0; i<sim->m_obj_count; ++i) {
        m_obj_i = &sim->m_objs[i];
        m_obj_i->vel = vector_add(m_obj_i->vel, sim->gravity);
        for(step=0; step < steps; ++step) {
            m_obj_i->pos = vector_add(m_obj_i->pos, vector_multiply(m_obj_i->vel, 1.0/steps));
            for(j=0; j<sim->m_obj_count; ++j) {
                if(i==j) continue;
                m_obj_j = &sim->m_objs[j];
                if(collides(m_obj_i->collider, m_obj_i->pos, m_obj_j->collider, m_obj_j->pos, &collision_point)) {
                    goto collided;
                }
            }
            for(j=0; j<sim->s_obj_count; ++j) {
                s_obj_j = &sim->s_objs[j];
                if(collides(m_obj_i->collider, m_obj_i->pos, s_obj_j->collider, s_obj_j->pos, &collision_point)) {
                    m_obj_i->vel.y *= -(m_obj_i->material.bounciness * s_obj_j->material.bounciness);
                    goto collided;
                }
            }
            continue;
            collided:
            break;
        }
    }
}

void render_obj(SDL_Renderer *renderer, vector pos, collider c) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    int i;
    for(i=0; i<c.vertex_count-1; ++i) {
        SDL_RenderDrawLine(renderer, c.vertices[i].x+pos.x, c.vertices[i].y+pos.y, c.vertices[i+1].x+pos.x, c.vertices[i+1].y+pos.y);
    }
    SDL_RenderDrawLine(renderer, c.vertices[c.vertex_count-1].x+pos.x, c.vertices[c.vertex_count-1].y+pos.y, c.vertices[0].x+pos.x, c.vertices[0].y+pos.y);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_Rect rect = {(int)pos.x-1, (int)pos.y-1, 4, 4};
    SDL_RenderFillRect(renderer, &rect);
}

void render(simulation *sim, SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);

    int i;
    for(i=0; i<sim->m_obj_count; ++i) {
        render_obj(renderer, sim->m_objs[i].pos, sim->m_objs[i].collider);
    }
    for(i=0; i<sim->s_obj_count; ++i) {
        render_obj(renderer, sim->s_objs[i].pos, sim->s_objs[i].collider);
    }
}

int begin_loop(simulation *sim, int windowWidth, int windowHeight, Uint32 flags) {
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

        tick(sim);
        render(sim, renderer);

        SDL_RenderPresent(renderer);

        stop = clock();
        int delay_time = 1000/sim->tick_rate - (stop-start);
        if(delay_time > 0) {
            SDL_Delay(delay_time);
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}