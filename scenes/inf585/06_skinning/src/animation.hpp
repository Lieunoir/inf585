#pragma once

#include "vcl/vcl.hpp"
#include "skeleton.hpp"

namespace vcl
{
    enum Marine_Animations {
        IDLE,
        WALK,
        RUN,
    };

    struct marine_animation_structure {
        bool transition = false;
        float transitionStart;
        Marine_Animations current_animation = IDLE;
        Marine_Animations next_animation = IDLE;
        skeleton_animation_structure idle_animation;
        skeleton_animation_structure walk_animation;
        skeleton_animation_structure run_animation;
    };

    buffer<affine_rt> mix(buffer<affine_rt> const &source, skeleton_animation_structure const &target, float alpha);

    buffer<affine_rt> to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index);

    buffer<affine_rt> transition(marine_animation_structure &marine_animation, skeleton_animation_structure &skeleton_data, timer_interval &timer);

    void change_animation(marine_animation_structure &marine_animation, timer_interval &timer);
}
