#include "skinning.hpp"

namespace vcl
{
	void normalize_weights(buffer<buffer<float>>& weights)
	{
		size_t const N = weights.size();
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			for(float w : weights[k]) s += w;
			assert_vcl_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}


	// Linear Blend Skinning
	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,  // position to deform
		buffer<vec3>& normal_skinned,    // normal to deform
		buffer<affine_rt> const& skeleton_current,    // rigid transforms for skeleton joints in current pose
		buffer<affine_rt> const& skeleton_rest_pose,  // rigid transforms of skeleton joints in rest pose
		buffer<vec3> const& position_rest_pose,       // vertex positions of the mesh in rest pose
		buffer<vec3> const& normal_rest_pose,         // normal coordinates of the mesh in rest pose
		rig_structure const& rig)                     // information of the skinning weights (joints and weights associated to a vertex)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		// Sanity check on sizes of buffers
		assert_vcl_no_msg(position_skinned.size()==N_vertex);
		assert_vcl_no_msg(normal_skinned.size()==N_vertex);
		assert_vcl_no_msg(normal_rest_pose.size()==N_vertex);
		assert_vcl_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_vcl_no_msg(rig.joint.size()==N_vertex);
		assert_vcl_no_msg(rig.weight.size()==N_vertex);

		// To do
		//   Compute the Linear Blend Skinning ...
        for(int i = 0; i < N_vertex; i++) {
            vec3 res(0.f, 0.f, 0.f);
            vec3 resn(0.f, 0.f, 0.f);
            quaternion resq1(0.0f, 0.0f, 0.0f, 0.0f);
            quaternion resq2(0.0f, 0.0f, 0.0f, 0.0f);
            for(int j = 0; j < rig.joint[i].size(); j++) {
/*
                quaternion temp1 = skeleton_current[rig.joint[i][j]].rotate.quat();
                quaternion temp2(skeleton_current[rig.joint[i][j]].translate.x,
                                skeleton_current[rig.joint[i][j]].translate.y,
                                skeleton_current[rig.joint[i][j]].translate.z,
                                0.0f);
                temp2 = 0.5f * temp2 * temp1;

                quaternion temp3 = skeleton_rest_pose[rig.joint[i][j]].rotate.quat();
                quaternion temp4(skeleton_rest_pose[rig.joint[i][j]].translate.x,
                                skeleton_rest_pose[rig.joint[i][j]].translate.y,
                                skeleton_rest_pose[rig.joint[i][j]].translate.z,
                                0.0f);
                temp4 = 0.5f * temp4 * temp3;

                temp3 = inverse(temp3);
                temp4 = - 1.0f * temp3 * temp4 * temp3;

                resq1 = resq1 + rig.weight[i][j] * temp1 * temp3;
                resq2 = resq2 + rig.weight[i][j] * (temp1 * temp4 + temp2 * temp3);
*/
                res = res + rig.weight[i][j]
                    * (skeleton_current[rig.joint[i][j]].translate + skeleton_current[rig.joint[i][j]].rotate
                            * inverse(skeleton_rest_pose[rig.joint[i][j]].rotate)
                            * ( - skeleton_rest_pose[rig.joint[i][j]].translate + position_rest_pose[i]));
                resn = resn + rig.weight[i][j]
                    * (skeleton_current[rig.joint[i][j]].translate + skeleton_current[rig.joint[i][j]].rotate
                            * inverse(skeleton_rest_pose[rig.joint[i][j]].rotate)
                            * ( - skeleton_rest_pose[rig.joint[i][j]].translate + normal_rest_pose[i]));
                //resq += resq + rig.weight[i][j] * skeleton_current[rig.joint[i][j]];
            }
            /*
            quaternion q0 = normalize(resq1);
            quaternion q1 = 2.f * resq2 * conjugate(q0);
            vec3 translation(q1.x, q1.y, q1.z);
            mat3 rot = rotation::quaternion_to_matrix(q0);
            position_skinned[i] = translation + rot * position_rest_pose[i];
            normal_skinned[i] = translation + rot * normal_rest_pose[i];
            */
            position_skinned[i] = res;
            normal_skinned[i] = resn;
        }
	}

}
