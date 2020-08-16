
#include "bench_hierarchy.hpp"


#ifdef SCENE_BENCH_HIERARCHY


using namespace vcl;



void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    // Loads the terrain texture
    //GLuint plank_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/oak-wood.png") );

    // Scaling parameter that adapts the distance between bench parts:
    const float scaling                 = 1.0f;

    // Bench specs:
    const float bench_leg_height        = scaling * 1.0f;
    const float bench_leg_width         = bench_leg_height * 0.08f;
    const float bench_plank_width       = bench_leg_height * 0.75f;
    const float bench_plank_length      = bench_leg_height * 3.5;
    const float bench_plank_height      = bench_leg_height * 0.05f;
    const float distance_between_planks = bench_leg_height * 0.1f;
    const float distance_between_back_post_planks = bench_plank_width * 0.4f;
    const float angle_of_back_post      = 4 * PI / 9;

    mesh_drawable bench_leg_back = mesh_primitive_parallelepiped(
    {0, 0, 0},
    {bench_leg_width, 0, 0},
    {0, bench_leg_width, 0},
    {-2*bench_leg_width, 2*bench_leg_width, -bench_leg_height});
    bench_leg_back.uniform.color = {0.1, 0.1, 0.1};


    mesh_drawable bench_leg_front = mesh_primitive_parallelepiped(
    {0, 0, 0},
    {bench_leg_width, 0, 0},
    {0, bench_leg_width, 0},
    {-2*bench_leg_width, -2*bench_leg_width, -bench_leg_height});
    bench_leg_front.uniform.color = {0.1, 0.1, 0.1};


    mesh_drawable bench_plank = mesh_primitive_parallelepiped(
    {0, 0, 0},
    {bench_plank_length, 0, 0},
    {0, bench_plank_width, 0},
    {0, 0, bench_plank_height});
    bench_plank.uniform.color = {0.58, 0.33, 0.16};

    mesh_drawable bench_plank_joint = mesh_primitive_parallelepiped(
    {0, 0, 0},
    {bench_leg_width, 0, 0},
    {0, 2 * bench_plank_width + distance_between_planks, 0},
    {0, 0, bench_plank_height * 0.9f});
    bench_plank_joint.uniform.color = {0.1, 0.1, 0.1};

    mesh_drawable bench_plank_joint_2 = mesh_primitive_parallelepiped(
    {0, 0, 0},
    {bench_leg_width, 0, 0},
    {0, bench_plank_width + distance_between_back_post_planks, 0},
    {0, 0, bench_plank_height * 0.9f});
    bench_plank_joint_2.uniform.color = {0.1, 0.1, 0.1};


    // Setting up the bottom layer
    bench_hierarchy.add(bench_plank, "body");
    bench_hierarchy.add(bench_plank, "bench_plank_1", "body", vec3(0.0f, bench_plank_width + distance_between_planks, 0.0f));
    bench_hierarchy.add(bench_plank_joint, "bench_joint_left", "body");
    bench_hierarchy.add(bench_plank_joint, "bench_joint_right", "body", vec3(bench_plank_length - bench_leg_width, 0.0f, 0.0f));

    // Legs of the Bench
    bench_hierarchy.add(bench_leg_front, "leg_front_left", "body");
    bench_hierarchy.add(bench_leg_front, "leg_front_right", "body", {vec3(bench_plank_length, 0.0f, 0.0f), {-1,0,0, 0,1,0, 0,0,1}});
    bench_hierarchy.add(bench_leg_back, "leg_back_left", "bench_plank_1", vec3(0.0f, bench_plank_width - bench_leg_width, 0.0f));
    bench_hierarchy.add(bench_leg_back, "leg_back_right", "bench_plank_1", {vec3(bench_plank_length, bench_plank_width - bench_leg_width, 0.0f), {-1,0,0, 0,1,0, 0,0,1}});

    // Setting up the back post layer
    bench_hierarchy.add(bench_plank_joint_2, "bench_joint_back_post_left" , "bench_plank_1", {vec3(0.0f, bench_plank_width, 0.0f), rotation_from_axis_angle_mat3({1,0,0}, angle_of_back_post)});
    bench_hierarchy.add(bench_plank_joint_2, "bench_joint_back_post_right", "bench_joint_back_post_left", vec3(bench_plank_length - bench_leg_width, 0.0f, 0.0f));
    bench_hierarchy.add(bench_plank, "bench_plank_2", "bench_joint_back_post_left", vec3(0.0f, distance_between_back_post_planks, 0.0f));


    // Sets the bench leg at the origin of the axis
    bench_hierarchy["body"].transform.translation = {2*bench_leg_width, 2*bench_leg_width, bench_leg_height};

    // Set the same shader for all the elements
    bench_hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    /** ********************* **/
    /** Display the hierarchy **/
    /** ********************* **/

    bench_hierarchy.update_local_to_global_coordinates();


    if(gui_scene.surface) // The default display
        draw(bench_hierarchy, scene.camera);

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
        draw(bench_hierarchy, scene.camera, shaders["wireframe"]);

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(bench_hierarchy, scene.camera);

}


void scene_model::set_gui()
{
    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe); ImGui::SameLine();
    ImGui::Checkbox("Surface", &gui_scene.surface);     ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);   ImGui::SameLine();

    ImGui::Spacing();

}





#endif

