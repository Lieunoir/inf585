#include "ground.hpp"

namespace vcl
{
    void initiate_ground(ground_struct &ground) {
        int bestIndex = 0;
        float bestOffset = norm(ground.positions[0]);
        for(int i = 1; i < ground.shape.position.size(); i++) {
            if(norm(ground.positions[i]) < bestOffset) {
                bestIndex = i;
                bestOffset = norm(ground.positions[i]);
            }
        }
        ground.character_position = bestIndex;
    }

    void evolve_ground(float dt, vec2 &character_position, vec2 &character_direction, ground_struct &ground) {
        character_position += character_direction*dt;
        size_t const N = ground.size;
        for(size_t k=0; k<N*N; ++k)
        {
            vec3 const& p0 = ground.positions[k];
            vec3& p        = ground.shape.position[k];
            float const dz = 0.3f*noise_perlin({0.2f*(p0.x-character_position.x), p0.y, 0.2f*(p0.z-character_position.y)}, 10);
            p = p0 + vec3(0, dz, 0);
        }
        ground.visual.update_position(ground.shape.position);
        ground.shape.compute_normal();
        ground.visual.update_normal(ground.shape.normal);
    }

    float get_offset(ground_struct &ground) {
        return ground.shape.position[ground.character_position].y;
    }

    void add_ground_offset(ground_struct &ground, buffer<vec3> &mesh) {
        for(int i = 0; i < mesh.size(); i++) {
            mesh[i].y += get_offset(ground);
        }
    }

    void add_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh) {
        for(int i = 0; i < mesh.size(); i++) {
            mesh[i].translate.y += get_offset(ground);
        }
    }
}
