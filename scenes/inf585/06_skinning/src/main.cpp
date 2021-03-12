/**
	Objectives:
	- Complete rigid transform interpolation in the function evaluate_local function in file skeleton.cpp
	- Complete the Linear Blend Skinning computation in the function skinning.cpp
*/

#include "vcl/vcl.hpp"
#include <iostream>

#include "skeleton.hpp"
#include "skeleton_drawable.hpp"
#include "skinning.hpp"
#include "skinning_loader.hpp"
#include "animation.hpp"
#include "ground.hpp"


using namespace vcl;

struct gui_parameters {
	bool display_frame = true;
	bool surface_skinned = true;
	bool wireframe_skinned = false;
	bool surface_rest_pose = false;
	bool wireframe_rest_pose = false;

	bool skeleton_current_bone = true;
	bool skeleton_current_frame = false;
	bool skeleton_current_sphere = false;

	bool skeleton_rest_pose_bone = false;
	bool skeleton_rest_pose_frame = false;
	bool skeleton_rest_pose_sphere = false;
};

struct user_interaction_parameters {
	vec2 mouse_prev;
	timer_fps fps_record;
	gui_parameters gui;
	mesh_drawable global_frame;
	bool cursor_on_gui;
};
user_interaction_parameters user;

struct scene_environment
{
	camera_around_center camera;
	mat4 projection;
	vec3 light;
};
scene_environment scene;

struct visual_shapes_parameters
{
	mesh_drawable surface_skinned;
	mesh_drawable surface_rest_pose;
	skeleton_drawable skeleton_current;
	skeleton_drawable skeleton_rest_pose;
};
visual_shapes_parameters visual_data;

struct skinning_current_data
{
	buffer<vec3> position_rest_pose;
	buffer<vec3> position_skinned;
	buffer<vec3> normal_rest_pose;
	buffer<vec3> normal_skinned;

	buffer<affine_rt> skeleton_current;
	buffer<affine_rt> skeleton_rest_pose;
};

skeleton_animation_structure skeleton_data;
rig_structure rig;
skinning_current_data skinning_data;

marine_animation_structure marine_animation;

timer_interval timer;
timer_basic floor_timer;
float previous_time;
timer_basic rotate_timer;


ground_struct ground;

// vec2 character_direction;
vec2 character_position;
float start_angle = 0.0;
float speed = 0.0;

float target_angle = 0.0;
float current_angle = 0.0;

void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void window_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

void initialize_data();
void display_scene();
void display_interface();
void compute_deformation();
void update_new_content(mesh const& shape, GLuint texture_id);

int main(int, char* argv[])
{

	std::cout << "Run " << argv[0] << std::endl;

	int const width = 1280, height = 1024;
	GLFWwindow* window = create_window(width, height);
	window_size_callback(window, width, height);
	std::cout << opengl_info_display() << std::endl;;

	imgui_init(window);
	glfwSetCursorPosCallback(window, mouse_move_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);
    glfwSetKeyCallback(window, key_callback);

	std::cout<<"Initialize data ..."<<std::endl;
	initialize_data();

	std::cout<<"Start animation loop ..."<<std::endl;
	user.fps_record.start();
	timer.start();
    floor_timer.start();
	glEnable(GL_DEPTH_TEST);
	while (!glfwWindowShouldClose(window))
	{
		scene.light = scene.camera.position();
		user.fps_record.update();
		timer.update();
        floor_timer.update();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);
		imgui_create_frame();
		if(user.fps_record.event) {
			std::string const title = "VCL Display - "+str(user.fps_record.fps)+" fps";
			glfwSetWindowTitle(window, title.c_str());
		}

		ImGui::Begin("GUI",NULL,ImGuiWindowFlags_AlwaysAutoResize);
		user.cursor_on_gui = ImGui::IsAnyWindowFocused();

		if(user.gui.display_frame)
			draw(user.global_frame, scene);

		display_interface();
        evolve_ground(floor_timer.t-previous_time, character_position, vec2(std::sin(start_angle), std::cos(start_angle)), ground, speed);
        previous_time = floor_timer.t;
		compute_deformation();
		display_scene();

		ImGui::End();
		imgui_render_frame(window);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	imgui_cleanup();
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void initialize_data()
{
	GLuint const shader_mesh = opengl_create_shader_program(opengl_shader_preset("mesh_vertex"), opengl_shader_preset("mesh_fragment"));
	GLuint const shader_uniform_color = opengl_create_shader_program(opengl_shader_preset("single_color_vertex"), opengl_shader_preset("single_color_fragment"));
	GLuint const texture_white = opengl_texture_to_gpu(image_raw{1,1,image_color_type::rgba,{255,255,255,255}});
	mesh_drawable::default_shader = shader_mesh;
	mesh_drawable::default_texture = texture_white;
	curve_drawable::default_shader = shader_uniform_color;
	segments_drawable::default_shader = shader_uniform_color;

	user.global_frame = mesh_drawable(mesh_primitive_frame());
	user.gui.display_frame = false;
	scene.camera.distance_to_center = 2.5f;

    ground.shape = mesh_primitive_grid({-10, 0, -10}, {10, 0, -10}, {10, 0, 10}, {-10, 0, 10}, ground.size, ground.size);
    ground.positions = ground.shape.position;
    ground.visual = mesh_drawable(ground.shape);
    ground.visual.shading.color  = {1.f, 1.f, 1.f};
    initiate_ground(ground);

	mesh shape;
	load_skinning_anim("assets/marine/anim_run/", marine_animation.run_animation);
	load_skinning_anim("assets/marine/anim_walk/", marine_animation.walk_animation);
	load_skinning_anim("assets/marine/anim_idle/", marine_animation.idle_animation);
    load_skinning_anim("assets/marine/anim_idle/", skeleton_data);
    GLuint texture_id;
	load_skinning_data("assets/marine/", skeleton_data, rig, shape, texture_id);
    marine_animation.run_animation.parent_index = skeleton_data.parent_index;
    marine_animation.walk_animation.parent_index = skeleton_data.parent_index;
    marine_animation.idle_animation.parent_index = skeleton_data.parent_index;
    marine_animation.run_animation.rest_pose_local = skeleton_data.rest_pose_local;
    marine_animation.walk_animation.rest_pose_local = skeleton_data.rest_pose_local;
    marine_animation.idle_animation.rest_pose_local = skeleton_data.rest_pose_local;

    normalize_weights(rig.weight);
    float const scaling = 0.005f;
    for(auto& p: shape.position) p *= scaling;
    skeleton_data.scale(scaling);
    marine_animation.walk_animation.scale(scaling);
    marine_animation.idle_animation.scale(scaling);
    marine_animation.run_animation.scale(scaling);
	update_new_content(shape, texture_id);
	rotate_timer.start();
}


void compute_deformation()
{
    if(marine_animation.transition) {
        skinning_data.skeleton_current = transition(marine_animation, skeleton_data, timer);
    } else {
        float const t = timer.t;
        skinning_data.skeleton_current = skeleton_data.evaluate_global(t);
    }

	rotate_timer.update();
    // Compute skinning deformation
	current_angle = start_angle;
	for(int i = 0; i < skinning_data.skeleton_current.size(); i++)
	{

		if(rotate_timer.t > 0.5f) // finish the rotation
		{
			start_angle = target_angle;
			skinning_data.skeleton_current[i] = rotation({0,1,0},start_angle)*skinning_data.skeleton_current[i];
		}
		else { //rotating
			float alpha = rotate_timer.t / 0.5;
			current_angle = alpha * target_angle + (1 - alpha) * start_angle;
			skinning_data.skeleton_current[i] = rotation({0,1,0},current_angle)*skinning_data.skeleton_current[i];

			//skinning_data.skeleton_current[i] = rotation::lerp(rotation({0,1,0},start_angle), rotation({0,1,0},target_angle), (rotate_timer.t / 0.5))*skinning_data.skeleton_current[i];
		}
		// skinning_data.skeleton_current[i].translate = {-1,0,0};
	}

	


    skinning_LBS_compute(skinning_data.position_skinned, skinning_data.normal_skinned,
        skinning_data.skeleton_current, skinning_data.skeleton_rest_pose,
        skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
        rig);

    add_ground_offset(ground, skinning_data.position_skinned);
    add_ground_offset(ground, skinning_data.skeleton_current);
    //TODO compute character facing rotation
    //TODO add IK

    visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index);
    visual_data.surface_skinned.update_position(skinning_data.position_skinned);
    visual_data.surface_skinned.update_normal(skinning_data.normal_skinned);
}

void display_scene()
{
	if(user.gui.surface_skinned)
		draw(visual_data.surface_skinned, scene);
	if (user.gui.wireframe_skinned)
		draw_wireframe(visual_data.surface_skinned, scene, {0.5f, 0.5f, 0.5f});

	draw(visual_data.skeleton_current, scene);

	if(user.gui.surface_rest_pose)
		draw(visual_data.surface_rest_pose, scene);
	if (user.gui.wireframe_rest_pose)
		draw_wireframe(visual_data.surface_rest_pose, scene, {0.5f, 0.5f, 0.5f});

    draw(ground.visual, scene);
	draw(visual_data.skeleton_rest_pose, scene);

}

void update_new_content(mesh const& shape, GLuint texture_id)
{
	visual_data.surface_skinned.clear();
	visual_data.surface_skinned = mesh_drawable(shape);
	visual_data.surface_skinned.texture = texture_id;

	visual_data.surface_rest_pose.clear();
	visual_data.surface_rest_pose = mesh_drawable(shape);
	visual_data.surface_rest_pose.texture = texture_id;

	skinning_data.position_rest_pose = shape.position;
	skinning_data.position_skinned = skinning_data.position_rest_pose;
	skinning_data.normal_rest_pose = shape.normal;
	skinning_data.normal_skinned = skinning_data.normal_rest_pose;

	skinning_data.skeleton_current = skeleton_data.rest_pose_global();
	skinning_data.skeleton_rest_pose = skinning_data.skeleton_current;

	visual_data.skeleton_current.clear();
	visual_data.skeleton_current = skeleton_drawable(skinning_data.skeleton_current, skeleton_data.parent_index);

	visual_data.skeleton_rest_pose.clear();
	visual_data.skeleton_rest_pose = skeleton_drawable(skinning_data.skeleton_rest_pose, skeleton_data.parent_index);

	timer.t_min = skeleton_data.animation_time[0];
	timer.t_max = skeleton_data.animation_time[skeleton_data.animation_time.size()-1];
	timer.t = skeleton_data.animation_time[0];
}

void display_interface()
{
	ImGui::Checkbox("Display frame", &user.gui.display_frame);
	ImGui::Spacing(); ImGui::Spacing();

	ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max, "%.2f s");
	ImGui::SliderFloat("Time Scale", &timer.scale, 0.05f, 2.0f, "%.2f s");

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Deformed ");
	ImGui::Text("Surface: ");ImGui::SameLine();
	ImGui::Checkbox("Plain", &user.gui.surface_skinned); ImGui::SameLine();
	ImGui::Checkbox("Wireframe", &user.gui.wireframe_skinned);

	ImGui::Text("Skeleton: "); ImGui::SameLine();
	ImGui::Checkbox("Bones", &user.gui.skeleton_current_bone); ImGui::SameLine();
	ImGui::Checkbox("Frame", &user.gui.skeleton_current_frame); ImGui::SameLine();
	ImGui::Checkbox("Sphere", &user.gui.skeleton_current_sphere);

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Rest Pose");
	ImGui::Text("Surface: ");ImGui::SameLine();
	ImGui::Checkbox("Plain##Rest", &user.gui.surface_rest_pose); ImGui::SameLine();
	ImGui::Checkbox("Wireframe##Rest", &user.gui.wireframe_rest_pose);

	ImGui::Text("Skeleton: "); ImGui::SameLine();
	ImGui::Checkbox("Bones##Rest", &user.gui.skeleton_rest_pose_bone); ImGui::SameLine();
	ImGui::Checkbox("Frame##Rest", &user.gui.skeleton_rest_pose_frame); ImGui::SameLine();
	ImGui::Checkbox("Sphere##Rest", &user.gui.skeleton_rest_pose_sphere);

	ImGui::Spacing(); ImGui::Spacing();


	visual_data.skeleton_current.display_segments = user.gui.skeleton_current_bone;
	visual_data.skeleton_current.display_joint_frame = user.gui.skeleton_current_frame;
	visual_data.skeleton_current.display_joint_sphere = user.gui.skeleton_current_sphere;
	visual_data.skeleton_rest_pose.display_segments = user.gui.skeleton_rest_pose_bone;
	visual_data.skeleton_rest_pose.display_joint_frame = user.gui.skeleton_rest_pose_frame;
	visual_data.skeleton_rest_pose.display_joint_sphere = user.gui.skeleton_rest_pose_sphere;

	mesh new_shape;
}

void window_size_callback(GLFWwindow* , int width, int height)
{
	glViewport(0, 0, width, height);
	float const aspect = width / static_cast<float>(height);
	scene.projection = projection_perspective(50.0f*pi/180.0f, aspect, 0.1f, 100.0f);
}


void mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
	vec2 const  p1 = glfw_get_mouse_cursor(window, xpos, ypos);
	vec2 const& p0 = user.mouse_prev;

	glfw_state state = glfw_current_state(window);

	auto& camera = scene.camera;
	if(!user.cursor_on_gui){
		if(state.mouse_click_left && !state.key_ctrl)
			scene.camera.manipulator_rotate_trackball(p0, p1);
		if(state.mouse_click_left && state.key_ctrl)
			camera.manipulator_translate_in_plane(p1-p0);
		if(state.mouse_click_right)
			camera.manipulator_scale_distance_to_center( (p1-p0).y );
	}

	user.mouse_prev = p1;
}

void opengl_uniform(GLuint shader, scene_environment const& current_scene)
{
	opengl_uniform(shader, "projection", current_scene.projection);
	opengl_uniform(shader, "view", scene.camera.matrix_view());
	opengl_uniform(shader, "light", scene.light, false);
}

bool shift = false;
bool up = false;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

    if(action == GLFW_PRESS) {
		if(key == GLFW_KEY_LEFT_SHIFT)
		{
			shift = true;
			if(marine_animation.current_animation == WALK)
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = RUN;
				speed = 1.0 * 2.f;
			}
		}
        if(key == GLFW_KEY_UP) {
			up = true;
			if(marine_animation.current_animation == IDLE && shift == false ) {
				change_animation(marine_animation, timer);
				marine_animation.next_animation = WALK;
				speed = 1.0;
			} 
			if(marine_animation.current_animation == IDLE && shift == true)
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = RUN;
				speed = 1.0 * 2.f;
			}

		}
		if(key == GLFW_KEY_DOWN)
		{
			start_angle = current_angle;
			rotate_timer.t = 0;
			target_angle = float(start_angle + M_PI);
			if(up == true) {
				if(marine_animation.current_animation == IDLE && shift == false) {
					change_animation(marine_animation, timer);
					marine_animation.next_animation = WALK;
					speed = 1.0;
				} 
				if(marine_animation.current_animation == IDLE && shift == true)
				{
					change_animation(marine_animation, timer);
					marine_animation.next_animation = RUN;
					speed = 1.0 * 2.f;
				}
			}
			else {
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}
		if(key == GLFW_KEY_LEFT)
		{
			start_angle = current_angle;
			rotate_timer.t = 0;
			target_angle = float(start_angle + M_PI/2);
			if(up == true) {
				if(marine_animation.current_animation == IDLE && shift == false) {
					change_animation(marine_animation, timer);
					marine_animation.next_animation = WALK;
					speed = 1.0;
				} 
				if(marine_animation.current_animation == IDLE && shift == true)
				{
					change_animation(marine_animation, timer);
					marine_animation.next_animation = RUN;
					speed = 1.0 * 2.f;
				}
			}
			else {
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}
		if(key == GLFW_KEY_RIGHT)
		{
			start_angle = current_angle;
			rotate_timer.t = 0;
			target_angle = float(start_angle - M_PI/2);
			if(up == true) {
				if(marine_animation.current_animation == IDLE && shift == false) {
					change_animation(marine_animation, timer);
					marine_animation.next_animation = WALK;
					speed = 1.0;
				} 
				if(marine_animation.current_animation == IDLE && shift == true)
				{
					change_animation(marine_animation, timer);
					marine_animation.next_animation = RUN;
					speed = 1.0 * 2.f;
				}
			}
			else {
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}

	}
	if(action == GLFW_RELEASE)
	{
		if(key ==  GLFW_KEY_LEFT_SHIFT) 
		{
			shift = false;
			if(marine_animation.current_animation == RUN)
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = WALK;
				speed = 1.0;
			}
		}
		if(key == GLFW_KEY_UP)
		{
			up = false;
			change_animation(marine_animation, timer);
			marine_animation.next_animation = IDLE;
			speed = 0.0;
		}
		if(key == GLFW_KEY_DOWN)
		{
			if( up == true && marine_animation.current_animation == WALK) {
				marine_animation.next_animation = WALK;
				speed = 1.0;
			}
			else if(up == true && marine_animation.current_animation == RUN)
			{
				marine_animation.next_animation = RUN;
				speed = 1.0;
			}
			else
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}
		if(key == GLFW_KEY_LEFT)
		{
			if( up == true && marine_animation.current_animation == WALK) {
				marine_animation.next_animation = WALK;
				speed = 1.0;
			}
			else if(up == true && marine_animation.current_animation == RUN)
			{
				marine_animation.next_animation = RUN;
				speed = 1.0;
			}
			else
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}
		if(key == GLFW_KEY_RIGHT)
		{
			if( up == true && marine_animation.current_animation == WALK) {
				marine_animation.next_animation = WALK;
				speed = 1.0;
			}
			else if(up == true && marine_animation.current_animation == RUN)
			{
				marine_animation.next_animation = RUN;
				speed = 1.0;
			}
			else
			{
				change_animation(marine_animation, timer);
				marine_animation.next_animation = IDLE;
				speed = 0.0;
			}
		}
	}
}
