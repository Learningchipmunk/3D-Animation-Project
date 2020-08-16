
#ifdef SCENE_FINAL_SCENE


using namespace vcl;

/*
    The use of hierarchy here is questionable because the bench is, in theory, an inanimated object.
    However, it is easier to place and rotate the many parts of the bench using a hierarchy.
    Thus, it led us to use the hierarchy_mesh_drawable instead of mesh like in tree modeling!
*/

hierarchy_mesh_drawable create_bench_hierarchy(float scaling = 1.0f);

void create_bench_hierarchy(hierarchy_mesh_drawable &bench_hierarchy, vec3 &bench_initial_position, float scaling)
{

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
    bench_initial_position = {2*bench_leg_width, 2*bench_leg_width, bench_leg_height};
    bench_hierarchy["body"].transform.translation = bench_initial_position;


}



#endif
