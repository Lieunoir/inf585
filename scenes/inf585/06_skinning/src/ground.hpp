#pragma once

#include "vcl/vcl.hpp"


namespace vcl
{
    struct ground_struct {
        int size = 100;
        int character_position;
        mesh shape;
        buffer<vec3> positions;
        mesh_drawable visual;
    };

    void initiate_ground(ground_struct &ground);

    void evolve_ground(float dt, vec2 &character_position, vec2 character_direction, ground_struct &ground, float speed);

    void add_ground_offset(ground_struct &ground, buffer<vec3> &mesh);
    void add_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh);
}
