#include "ground.hpp"

namespace vcl
{
    void initiate_ground(ground_struct &ground) {
        int bestIndex = 0;
        float bestOffset = norm(ground.positions[0]);
        for(int i = 1; i < ground.shape.position.size(); i++) {
            if(ground.positions[i].x < 0.f && ground.positions[i].z < 0.f && norm(ground.positions[i]) < bestOffset) {
                bestIndex = i;
                bestOffset = norm(ground.positions[i]);
            }
        }
        ground.character_position_1 = bestIndex;

        bestIndex = 0;
        bestOffset = norm(ground.positions[0]);
        for(int i = 1; i < ground.shape.position.size(); i++) {
            if(ground.positions[i].x > 0.f && ground.positions[i].z < 0.f && norm(ground.positions[i]) < bestOffset) {
                bestIndex = i;
                bestOffset = norm(ground.positions[i]);
            }
        }
        ground.character_position_2 = bestIndex;

        bestIndex = 0;
        bestOffset = norm(ground.positions[0]);
        for(int i = 1; i < ground.shape.position.size(); i++) {
            if(ground.positions[i].x > 0.f && ground.positions[i].z > 0.f && norm(ground.positions[i]) < bestOffset) {
                bestIndex = i;
                bestOffset = norm(ground.positions[i]);
            }
        }
        ground.character_position_3 = bestIndex;

        bestIndex = 0;
        bestOffset = norm(ground.positions[0]);
        for(int i = 1; i < ground.shape.position.size(); i++) {
            if(ground.positions[i].x < 0.f && ground.positions[i].z > 0.f && norm(ground.positions[i]) < bestOffset) {
                bestIndex = i;
                bestOffset = norm(ground.positions[i]);
            }
        }
        ground.character_position_4 = bestIndex;
    }

    void evolve_ground(float dt, vec2 &character_position, vec2 character_direction, ground_struct &ground, float speed) {
        //character_position += character_direction*dt;
        character_position += character_direction*dt*speed;

        size_t const N = ground.size;
        for(size_t k=0; k<N*N; ++k)
        {
            vec3 const& p0 = ground.positions[k];
            vec3& p        = ground.shape.position[k];
            float const dz = 0.4f*noise_perlin({0.4f*(p0.x-character_position.x), p0.y, 0.4f*(p0.z-character_position.y)}, 10);
            p = p0 + vec3(0, dz, 0);
        }
        ground.visual.update_position(ground.shape.position);
        ground.shape.compute_normal();
        ground.visual.update_normal(ground.shape.normal);
    }

    float get_offset(ground_struct &ground, float x, float z) {
        vec3 p1 = ground.shape.position[ground.character_position_1];
        vec3 p2 = ground.shape.position[ground.character_position_2];
        vec3 p3 = ground.shape.position[ground.character_position_3];
        vec3 p4 = ground.shape.position[ground.character_position_4];
        float z1 = p1.y;
        float z2 = p2.y;
        float z3 = p3.y;
        float z4 = p4.y;
        float z11 = (- z1 * (p2.x - x) + z2 * (p1.x - x)) / (p2.x - p1.x);
        float z12 = (- z4 * (p3.x - x) + z3 * (p4.x - x)) / (p3.x - p4.x);
        return (- z11 * (p3.z - z) + z12 * (p1.z - z)) / (p3.z - p1.z) ;
    }

    float add_ground_offset(ground_struct &ground, buffer<vec3> &mesh, int i1, int i2) {
        float x1 = mesh[i1].x;
        float z1 = mesh[i1].z;
        float x2 = mesh[i2].x;
        float z2 = mesh[i2].z;
        float offset1 = get_offset(ground, x1, z1);
        float offset2 = get_offset(ground, x2, z2);
        float offset = std::min(offset1, offset2);
        for(int i = 0; i < mesh.size(); i++) {
            mesh[i].y += offset;
        }
        return offset;
    }

    float add_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i1, int i2) {
        float x1 = mesh[i1].translate.x;
        float z1 = mesh[i1].translate.z;
        float x2 = mesh[i2].translate.x;
        float z2 = mesh[i2].translate.z;
        float offset1 = get_offset(ground, x1, z1);
        float offset2 = get_offset(ground, x2, z2);
        float offset = std::min(offset1, offset2);
        for(int i = 0; i < mesh.size(); i++) {
            mesh[i].translate.y += offset;
        }
        return offset;
    }

    vec3 get_localized_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i, float off) {
        float x = mesh[i].translate.x;
        float z = mesh[i].translate.z;
        return mesh[i].translate + vec3(0.f, get_offset(ground, x, z) - off, 0.f);
    }

    void add_localized_ground_offset(ground_struct &ground, buffer<affine_rt> &mesh, int i, float off) {
        float x = mesh[i].translate.x;
        float z = mesh[i].translate.z;
        mesh[i].translate = mesh[i].translate + vec3(0.f, get_offset(ground, x, z) - off, 0.f);
    }

    void IK(buffer<affine_rt> &bones, std::vector<int> boneIndices, std::vector<float> bone_lengths, int iPos, vec3 target) {
        /*
        vec3 pos = bones[iPos].translate;
        //takes wayyyyyy too much time to converge
        for(int k = 0; k < 500; k++) {
            for(int i = boneIndices.size()-1; i >= 0; i--) {
                vec3 axis = cross(normalize(target-bones[boneIndices[i]].translate), normalize(pos-bones[boneIndices[i]].translate));
                if(norm(axis) > 0.00001f) {
                    float sin = norm(axis);
                    normalize(axis);
                    for(int j = i; j < boneIndices.size(); j++) {
                        bones[boneIndices[j]].rotate = rotation(axis, -sin * 1.) * bones[boneIndices[j]].rotate;
                    }
                }
                for(int j = i; j < boneIndices.size(); j++) {
                    bones[boneIndices[j]+1].translate = bones[boneIndices[j]] * vec3(0.f, 0.f, -bone_lengths[j]);
                }
                pos = bones[iPos].translate;
            }
        }
        bones[iPos].translate = target;
        */
        vec3 init = bones[boneIndices[0]].translate;
        buffer<affine_rt> temp;
        for(int i = 0; i < boneIndices.size(); i++) {
            temp.push_back(bones[boneIndices[i]]);
        }
        temp.push_back(bones[iPos]);
        for(int k = 0; k < 10; k++) {
            bones[iPos].translate = target;
            for(int i = boneIndices.size()-1; i >= 0; i--) {
                bones[boneIndices[i]].translate = bone_lengths[i] * normalize(bones[boneIndices[i]].translate - bones[boneIndices[i]+1].translate) + bones[boneIndices[i]+1].translate;
            }

            bones[boneIndices[0]].translate = init;
            for(int i = 0; i < boneIndices.size(); i++) {
                bones[boneIndices[i]+1].translate = bone_lengths[i] * normalize(bones[boneIndices[i]+1].translate - bones[boneIndices[i]].translate) + bones[boneIndices[i]].translate;
            }
        }
        for(int i = 0; i < boneIndices.size(); i++) {
            vec3 orig = normalize(temp[i+1].translate - temp[i].translate);
            //vec3 orig = vec3(0.f, 0.f, -1.f);
            vec3 current = normalize(bones[boneIndices[i]+1].translate - bones[boneIndices[i]].translate);
            vec3 axis = cross(current, orig);
            if(norm(axis) > 0.00001f) {
                bones[boneIndices[i]].rotate = rotation(axis/norm(axis), -asin(norm(axis))) * bones[boneIndices[i]].rotate;
            }
        }
        //bones[iPos].translate = target;
    }

    void add_leg_IK(ground_struct &ground, buffer<affine_rt> &bones, float off, int iFoot1, int iFoot2, std::vector<int> boneIndices) {
        vec3 oldFoot = normalize(bones[iFoot2].translate - bones[iFoot1].translate);
        std::vector<float> bone_lengths;
        for(int i = 0; i < boneIndices.size(); i++) {
            bone_lengths.push_back(norm(bones[boneIndices[i]+1].translate - bones[boneIndices[i]].translate));
        }
        float foot_length = norm(bones[iFoot2].translate - bones[iFoot1].translate);

        vec3 target = get_localized_ground_offset(ground, bones, iFoot1, off);
        IK(bones, boneIndices, bone_lengths, iFoot1, target);

        add_localized_ground_offset(ground, bones, iFoot2, off);
        bones[iFoot2].translate = bones[iFoot1].translate + foot_length * normalize(bones[iFoot2].translate - bones[iFoot1].translate);
        vec3 newLeftFoot = normalize(bones[iFoot2].translate - bones[iFoot1].translate);
        vec3 axis = cross(newLeftFoot, oldFoot);
        if(norm(axis) > 0.00001f)
            bones[iFoot1].rotate = rotation(axis/norm(axis), -asin(norm(axis))) * bones[iFoot1].rotate;
    }

    void add_legs_IK(ground_struct &ground, buffer<affine_rt> &bones, float off) {
        add_leg_IK(ground, bones, off, 5, 6, {3, 4});
        add_leg_IK(ground, bones, off, 9, 10, {7, 8});
    }
}
