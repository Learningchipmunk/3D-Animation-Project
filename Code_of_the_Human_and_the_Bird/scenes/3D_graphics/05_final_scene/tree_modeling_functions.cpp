
#ifdef SCENE_FINAL_SCENE


using namespace vcl;

// ------------------ Function declarations ------------------ //

mesh create_tree_foliage(float radius, float height, float z_offset, float initial_z_offset = 0.0f);
mesh mesh_primitive_create_tree_foliage(float radius, float height, float z_offset, float initial_z_offset = 0.0f);

/**
    @param Scaling   parameter that adapts the distance between the tree parts.
    @param tree      is the mesh_drawable that is declared in final_scene.hpp
*/
void create_tree_mesh(mesh_drawable &tree, float scaling = 1.0f);


// ------------------ Function implementation ------------------ //

mesh create_cone(float radius, float height, float z_offset)
{
    mesh cone; // temporary cone storage (CPU only)


    const float prop = PI/10;
    float theta = 0;

    // the center of the cone
    vec3 center = {0.0f, 0.0f, z_offset};

    cone.position = {{center[0], center[1], center[2]},
                         {center[0], center[1], center[2] + height},
                         {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]}};

    theta+=prop;

    // Adds the new point
    cone.position.push_back(
    {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]});

    // Gives the drawing order of the points for the first triangle
    cone.connectivity = {{0, 2, 3}};
    cone.connectivity = {{1, 2, 3}};



    for(unsigned int i = 3; theta <= 2*PI; ++i)
    {
        theta += prop;
        // Adds the new point
        cone.position.push_back(
        {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]});

        // base triangle
        cone.connectivity.push_back({0, i, i+1});

        // top triangle
        cone.connectivity.push_back({1, i, i+1});
    }
    return cone;
}



mesh create_tree_foliage(float radius, float height, float z_offset, float initial_z_offset)
{
    mesh m = create_cone(radius, height, initial_z_offset);
    m.push_back( create_cone(radius, height, initial_z_offset + z_offset) );
    m.push_back( create_cone(radius, height, initial_z_offset + 2*z_offset) );

    return m;
}


mesh mesh_primitive_create_tree_foliage(float radius, float height, float z_offset, float initial_z_offset)
{
    mesh m = mesh_primitive_cone(radius, {0, 0, initial_z_offset}, {0, 0, initial_z_offset + height});
    m.push_back( mesh_primitive_cone(radius, {0, 0, initial_z_offset + z_offset}, {0, 0, initial_z_offset + height + z_offset}) );
    m.push_back( mesh_primitive_cone(radius, {0, 0, initial_z_offset + 2*z_offset}, {0, 0, initial_z_offset + height + 2*z_offset}) );

    return m;
}


void create_tree_mesh(mesh_drawable &tree, float scaling)
{
    // Temporary CPU mesh
    mesh tree_CPU;

    // Tree specs:
    const float trunk_height   = scaling * 1.0f;
    const float trunk_radius   = trunk_height * 0.2f;
    const float foliage_radius = trunk_height * 0.8f;
    const float foliage_height = trunk_height * 0.7f;
    const float foliage_offset = foliage_height / 2;




    // Create the trunk (cylinder)
    mesh trunk = (mesh_primitive_cylinder(trunk_radius, {0, 0, 0}, {0, 0, trunk_height}));
    trunk.fill_color_uniform({0.5f, 0.35f, 0.05f});


    // Create the foliage (multiple cones)
    mesh foliage = create_tree_foliage(foliage_radius, foliage_height, foliage_offset, trunk_height);
    foliage.fill_color_uniform({0.5f,0.7f,0.45f});



    // adds the foliage and trunk to the tree
    tree_CPU.push_back(foliage);
    tree_CPU.push_back(trunk);

    // Converts it to mesh_drawable in orger to draw it.
    tree = mesh_drawable(tree_CPU);

}

#endif

