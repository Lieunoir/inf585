#include "animation.hpp"

namespace vcl
{
    buffer<affine_rt> mix(buffer<affine_rt> const &source, skeleton_animation_structure const &target, float alpha) {
        size_t const N_joint = source.size();
        buffer<affine_rt> skeleton_current;
        skeleton_current.resize(N_joint);

        for(size_t kj=0; kj<N_joint; ++kj)
        {

            affine_rt const T0 = source[kj];
            affine_rt const T1 = target.animation_geometry_local[0][kj];

            affine_rt T;
            T.translate = alpha * T0.translate + (1-alpha) * T1.translate;
            T.rotate = rotation::lerp(T0.rotate, T1.rotate, alpha);

            skeleton_current[kj] = T;
        }

        return skeleton_current;
    }


    buffer<affine_rt> to_global(buffer<affine_rt> const& local, buffer<int> const& parent_index)
    {
        assert_vcl(parent_index.size()==local.size(), "Incoherent size of skeleton data");
        size_t const N = parent_index.size();
        buffer<affine_rt> global;
        global.resize(N);
        global[0] = local[0];

        for (size_t k = 1; k < N; ++k)
            global[k] = global[parent_index[k]] * local[k];

        return global;
    }

    buffer<affine_rt> transition(marine_animation_structure &marine_animation, skeleton_animation_structure &skeleton_data, timer_interval &timer) {
        skeleton_animation_structure target;
        switch(marine_animation.next_animation) {
            case WALK:
                  target = marine_animation.walk_animation;
                  break;
            case RUN:
                  target = marine_animation.run_animation;
                  break;
            case IDLE:
                  target = marine_animation.idle_animation;
                  break;
        }

        if(20.f * timer.t >  timer.t_max) {
            marine_animation.transition = false;
            skeleton_data = target;
            marine_animation.current_animation = marine_animation.next_animation;

            timer.t_min = skeleton_data.animation_time[0];
            timer.t_max = skeleton_data.animation_time[skeleton_data.animation_time.size()-1];
            timer.t = skeleton_data.animation_time[0];
            return skeleton_data.evaluate_global(0.f);
        }
        return to_global(mix(skeleton_data.evaluate_local(marine_animation.transitionStart), target, timer.t * 20.f / timer.t_max), skeleton_data.parent_index);

    }

    void change_animation(marine_animation_structure &marine_animation, timer_interval &timer) {
        marine_animation.transitionStart = timer.t;
        marine_animation.transition = true;

        timer.t_min = 0.f;
        timer.t_max = 2.f;
        timer.t = 0.f;
    }
}
