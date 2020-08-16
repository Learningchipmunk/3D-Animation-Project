
#ifdef SCENE_FINAL_SCENE


using namespace vcl;


// Adjusts the loaded mesh with some roations and scaling
void adjustLoadedObjects(mesh_drawable &mesh, double scalingBody, bool isSymetrical=false)
{
    double signOfYRot = 1;
    if(isSymetrical)signOfYRot = -1;

    // Blender's axis are not the same as vcl's axis
    mesh.uniform.transform.rotation = rotation_from_axis_angle_mat3({1,0,0}, PI/2 ) * rotation_from_axis_angle_mat3({0,1,0}, signOfYRot * PI/2 );
    mesh.uniform.transform.scaling = scalingBody;

}

// -------------------------------- Human Hierarchy -------------------------------- //

void create_human_hierarchy(
        hierarchy_mesh_drawable &human_hierarchy,
        vec3 &human_body_z_translation,
        vec3 &human_initial_position,
        float terrain_length,
        float terrain_width,
        float scalingBody = 0.5f)
{

    // --- Setting the ratios for the body --- //
    // Source : http://villemin.gerard.free.fr/Biologie/CorpsPro.htm
    const float length_body =  5.0f * scalingBody;
    const float radius_body =  .35f * length_body / 6;
    const float length_neck =  .14f * length_body / 2;
    const float radius_neck =  1.0f * radius_body * 1.5 / 2;
    // Adjusted with trial and error:
    const float radius_shoulder_joint = 0.101f * length_body / 2;
    const float radius_arm_joint = 0.06f * length_body / 2;
    const float radius_leg_joint = 0.118f * length_body / 2;



    // ----------------- loading blender obj files ----------------- //

    // Loading the geometry of the body
    mesh_drawable body = mesh_load_file_obj("Blender_Assets/inf443_vcl-masterTorso.obj");//mesh_drawable( mesh_primitive_ellipsoide(radius_body, {0,0,0}, a, b, c, 40, 40));
    adjustLoadedObjects(body, scalingBody);
    //GLuint tshirt_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/black_t-shirt.png") );
    //body.texture_id = tshirt_texture_id;

    // The geometry of the head is imported from a file made with blender
    mesh_drawable head = mesh_load_file_obj("Blender_Assets/FaceOnly.obj");
    adjustLoadedObjects(head, scalingBody);

    // Shoulder part and arm and neck are displayed as cylinder
    mesh_drawable neck = mesh_primitive_cylinder(radius_neck, {0,0,0}, {0,0,-length_neck}, 20, 20);
    neck.uniform.transform.translation = {0.1,0,0.1};

    mesh_drawable shoulder_left = mesh_load_file_obj("Blender_Assets/inf443_vcl-masterleft_arm.obj");//mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    adjustLoadedObjects(shoulder_left, scalingBody);
    // Slight adjustment:
    shoulder_left.uniform.transform.rotation = rotation_from_axis_angle_mat3({0,0,1}, -0.1415f )
                                               * shoulder_left.uniform.transform.rotation;


    // Same mesh as shoulder_left, we copy it and use a symetry afterwards
    mesh_drawable shoulder_right = shoulder_left;//mesh_load_file_obj("Blender_Assets/inf443_vcl-masterleft_arm.obj");//mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    adjustLoadedObjects(shoulder_right, scalingBody, true);
    // Slight adjustment:
    shoulder_right.uniform.transform.rotation = rotation_from_axis_angle_mat3({0,0,1},  0.1415f )
                                                * shoulder_right.uniform.transform.rotation;



    mesh_drawable arm_left = mesh_load_file_obj("Blender_Assets/inf443_vcl-masterleft_arm_el.obj");//mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    adjustLoadedObjects(arm_left, scalingBody);
    // Slight adjustment:
    arm_left.uniform.transform.rotation = rotation_from_axis_angle_mat3({1,0,0}, 0.5415f ) * rotation_from_axis_angle_mat3({0,0,1}, -0.1415f )
                                               * arm_left.uniform.transform.rotation;
    // arm is not in resting position at t = 0s
    arm_left.uniform.transform.rotation = rotation_from_axis_angle_mat3({0,1,0}, -0.5f) * arm_left.uniform.transform.rotation;


    mesh_drawable arm_right = arm_left;//mesh_load_file_obj("Blender_Assets/inf443_vcl-masterleft_arm_el.obj");//mesh_primitive_cylinder(radius_arm, {0,0,0}, {-length_arm,0,0});
    adjustLoadedObjects(arm_right, scalingBody, true);
    // Slight adjustment:
    arm_right.uniform.transform.rotation = rotation_from_axis_angle_mat3({1,0,0}, -0.5415f ) * rotation_from_axis_angle_mat3({0,0,1}, -0.1415f )
                                               * arm_right.uniform.transform.rotation;
    // arm is not in resting position at t = 0s
    arm_right.uniform.transform.rotation = rotation_from_axis_angle_mat3({0,1,0}, 0.5f) * arm_right.uniform.transform.rotation;


    mesh_drawable left_leg = mesh_load_file_obj("Blender_Assets/left_leg.obj");
    adjustLoadedObjects(left_leg, scalingBody);


    mesh_drawable right_leg = left_leg;
    adjustLoadedObjects(right_leg, scalingBody, true);


    // Joints displayed as spheres
    mesh_drawable shoulder_joint = mesh_primitive_sphere(radius_shoulder_joint);
    mesh_drawable arm_joint = mesh_primitive_sphere(radius_arm_joint);
    mesh_drawable leg_joint = mesh_primitive_sphere(radius_leg_joint);


    // --------------------------- Building the human hierarchy --------------------------- //

    // Body position is set with respect to the offset for him to be grounded
    human_hierarchy.add(body, "body");
    human_body_z_translation = vec3(0.0f, 0.0f, 2.87f);// Set with trial and error method...
    human_initial_position = evaluate_terrain(0.2, 0.2, terrain_length, terrain_width) + human_body_z_translation;
    human_hierarchy["body"].transform.translation = human_initial_position;

    // The neck covering the gap:
    human_hierarchy.add(neck, "neck", "body" , length_neck * vec3(0.0f, 0.0f, 0.3f));
    // The head:
    human_hierarchy.add(head, "head", "body", length_neck * vec3(0.35f, 0.0f, 0.1f)); // , length_neck * vec3(0.0f, 0.0f, 1.0f)

    // Shoulders and shoulder joints:
    human_hierarchy.add(shoulder_joint, "shoulder_joint_right", "body", radius_body * vec3(0.6f, -2.5f, -1.7f));
    human_hierarchy.add(shoulder_right, "shoulder_right", "shoulder_joint_right", radius_shoulder_joint* vec3(0.0f, -0.1f, -0.1f));

    human_hierarchy.add(shoulder_joint, "shoulder_joint_left", "body", radius_body * vec3(0.6f, 2.5f, -1.7f));
    human_hierarchy.add(shoulder_left, "shoulder_left", "shoulder_joint_left", radius_shoulder_joint* vec3(0.0f, 0.1f, -0.1f));

    // Arms and arm joints:
    human_hierarchy.add(arm_joint, "arm_joint_right", "shoulder_right", radius_body * vec3(0.395f, -1.85f, -3.3f));
    human_hierarchy.add(arm_right, "arm_right", "arm_joint_right", radius_shoulder_joint* vec3(0.1f, 0.65f, -0.1f));


    human_hierarchy.add(arm_joint, "arm_joint_left", "shoulder_left", radius_body * vec3(0.13f, 1.85f, -3.3f));
    human_hierarchy.add(arm_left, "arm_left", "arm_joint_left", radius_shoulder_joint* vec3(-0.1f, -0.65f, -0.1f));

    // Legs and joints:
    human_hierarchy.add(leg_joint, "leg_joint_left", "body", radius_body * vec3(1.3f, 1.3f, -9.3f));
    human_hierarchy.add(left_leg, "left_leg", "leg_joint_left", radius_leg_joint * vec3(0.2f, 0.0f, 0.0f));


    human_hierarchy.add(leg_joint, "leg_joint_right", "body", radius_body * vec3(1.3f, -1.2f, -9.3f));
    human_hierarchy.add(right_leg, "right_leg", "leg_joint_right", radius_leg_joint * vec3(0.2f, 0.0f, 0.0f));


}
#endif
