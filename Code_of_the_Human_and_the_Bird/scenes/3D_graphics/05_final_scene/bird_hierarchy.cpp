
#ifdef SCENE_FINAL_SCENE


using namespace vcl;



// -------------------------------- Bird Hierarchy -------------------------------- //

void create_bird_hierarchy(hierarchy_mesh_drawable &bird_hierarchy,
                            vec3 &bird_initial_position,
                            float scaling = 1.0f)
{
    const float bird_radius_head = scaling * 0.25f;
    const float bird_radius_body = bird_radius_head / 1.5f;
    const float length_wings     = scaling * 0.3f;
    const float length_nose      = scaling * 0.2f;


    // Constants for the ellipsoide :
    const float a = 4, b=2, c=2;

    // The geometry of the body is an ellipsoide
    mesh_drawable bird_body = mesh_drawable( mesh_primitive_ellipsoide(bird_radius_body, {0,0,0}, a, b, c, 40, 40));
    bird_body.uniform.transform.rotation = rotation_from_axis_angle_mat3({0, 0, 1}, PI/2);

    // The geometry of the head is a sphere
    mesh_drawable bird_head = mesh_drawable( mesh_primitive_sphere(bird_radius_head, {0,0,0}, 40, 40));

    // Geometry of the eyes: black spheres
    mesh_drawable eye = mesh_drawable(mesh_primitive_sphere(0.05f, {0,0,0}, 20, 20));
    eye.uniform.color = {0,0,0};

    // Geometry of the nose: orange cone
    mesh_drawable nose = mesh_drawable(mesh_primitive_cone(length_nose/2, vec3{0.0f,0.0f,0.0f}, vec3{0.0f,length_nose * 0.9,0.0f}, 20, 20));
    nose.uniform.color = {1,0.55,0};


    // Shoulder part and arm are displayed as cylinder
    mesh_drawable wing1 = mesh_primitive_quad({0,-bird_radius_body * 1.5f,0}, {0,bird_radius_body* 1.5f,0}, {-length_wings,bird_radius_body* 1.5f,0}, {-length_wings,-bird_radius_body* 1.5f,0});
    mesh_drawable wing2 = mesh_primitive_quad({0,-bird_radius_body* 1.5f,0}, {0,bird_radius_body* 1.5f,0}, {-length_wings*0.5,bird_radius_body/2,0}, {-length_wings*0.5,-bird_radius_body/2,0});


    // Builds the Bird hierarchy:

    // Body position is set with respect to some ratio of the head
    bird_hierarchy.add(bird_body, "body");
    bird_initial_position = vec3(0.0f, 0.0f, 10.5f);
    // Puts the bird initially on top of the mountain
    bird_hierarchy["body"].transform.translation = bird_initial_position;
    // Aligns it with the x axis
    bird_hierarchy["body"].transform.rotation = rotation_from_axis_angle_mat3({0, 0, 1}, -PI/2);


    bird_hierarchy.add(bird_head, "head", "body" , bird_radius_body * vec3( 0.0f, a, b));

    // Eyes positions are set with respect to some ratio of the head
    bird_hierarchy.add(eye, "eye_left", "head" , bird_radius_head * vec3( 1/3.0f, 1/2.0f, 1/1.5f));
    bird_hierarchy.add(eye, "eye_right", "head", bird_radius_head * vec3(-1/3.0f, 1/2.0f, 1/1.5f));

    // Nose position is set
    bird_hierarchy.add(nose, "nose", "head" , bird_radius_head * vec3( 0.0f, 0.9f, 0.0f));

    // Set the left part of the head arm: shoulder-elbow-arm
    bird_hierarchy.add(wing1, "wing1_left", "body", {-b * bird_radius_body+0.05f,0,0}); // extremity of the spherical body
    bird_hierarchy.add(wing2, "wing2_left", "wing1_left", {-length_wings, 0, 0});                        // the arm start at the center of the elbow

    // Set the right part of the head arm: similar to the left part excepted a symmetry is applied along x direction for the shoulder
    bird_hierarchy.add(wing1, "wing1_right", "body",     {{b *bird_radius_body-0.05f,0,0}, {-1,0,0, 0,1,0, 0,0,1}/*Symmetry*/ } );
    bird_hierarchy.add(wing2, "wing2_right", "wing1_right", {-length_wings, 0, 0});


}
#endif
