#pragma once

#include "vcl/vcl.hpp"


namespace vcl
{
    struct ground_struct {
        int size = 100;
        int character_position_1;
        int character_position_2;
        int character_position_3;
        int character_position_4;
        mesh shape;
        buffer<vec3> positions;
        mesh_drawable visual;
    };

    void initiate_ground(ground_struct &ground);

    void evolve_ground(float dt, vec2 &character_position, vec2 &character_direction, ground_struct &ground);

    float add_ground_offset(ground_struct &ground, buffer<vec3> &mesh, int i1, int i2);
    float add_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i1, int i2);
    vec3 get_localized_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i, float off);
    void add_localized_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i, float off);
    void IK(buffer<affine_rt> &bones, std::vector<int> boneIndices, std::vector<float> bone_lengths, int iPos, vec3 target);
    void add_legs_IK(ground_struct &ground, buffer<affine_rt> &bones, float off);
}
