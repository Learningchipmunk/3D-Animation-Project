
#include "articulated_hierarchy.hpp"


#ifdef SCENE_ARTICULATED_HIERARCHY


using namespace vcl;


// Adjusts the loaded mesh with some roations and scaling
void adjustLoadedObjects(mesh_drawable &mesh, double scalingBody, bool isSymetrical=false)
{
    double signOfYRot = 1;
    if(isSymetrical)signOfYRot = -1;

    // Blender's axis are not the same as vcl's axis
    mesh.uniform.transform.rotation = rotation_from_axis_angle_mat3({1,0,0}, 3.1415f/2 ) * rotation_from_axis_angle_mat3({0,1,0}, signOfYRot * 3.1415f/2 );
    mesh.uniform.transform.scaling = scalingBody;

}


void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{

    // --- Setting the ratios for the body --- //
    // Source : http://villemin.gerard.free.fr/Biologie/CorpsPro.htm
    const float scalingBody =  0.5f;
    const float length_body =  5.0f * scalingBody;
    const float radius_head =  .13f * length_body / 2;
    const float radius_arm  =  .05f * length_body / 2;
    const float length_arm  =  .45f * length_body / 2;
    const float radius_body =  .35f * length_body / 6;
    const float length_neck =  .14f * length_body / 2;
    const float radius_neck =  1.0f * radius_body * 1.5 / 2;
    // Adjusted with trial and error:
    const float radius_shoulder_joint = 0.101f * length_body / 2;
    const float radius_arm_joint = 0.06f * length_body / 2;
    const float radius_leg_joint = 0.118f * length_body / 2;





    // The geometry of the body is an ellipsoide
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


    // Build the hierarchy:
    // Syntax to add element
    //   hierarchy.add(visual_element, element_name, parent_name, (opt)[translation, rotation])

    // Body position is set with respect to some ratio of the head
    hierarchy.add(body, "body");


    // The neck covering the gap:
    hierarchy.add(neck, "neck", "body" , length_neck * vec3(0.0f, 0.0f, 0.3f));
    // The head:
    hierarchy.add(head, "head", "body", length_neck * vec3(0.35f, 0.0f, 0.1f)); // , length_neck * vec3(0.0f, 0.0f, 1.0f)

    // Shoulders and shoulder joints:
    hierarchy.add(shoulder_joint, "shoulder_joint_right", "body", radius_body * vec3(0.6f, -2.5f, -1.7f));
    hierarchy.add(shoulder_right, "shoulder_right", "shoulder_joint_right", radius_shoulder_joint* vec3(0.0f, -0.1f, -0.1f));

    hierarchy.add(shoulder_joint, "shoulder_joint_left", "body", radius_body * vec3(0.6f, 2.5f, -1.7f));
    hierarchy.add(shoulder_left, "shoulder_left", "shoulder_joint_left", radius_shoulder_joint* vec3(0.0f, 0.1f, -0.1f));

    // Arms and arm joints:
    hierarchy.add(arm_joint, "arm_joint_right", "shoulder_right", radius_body * vec3(0.395f, -1.85f, -3.3f));
    hierarchy.add(arm_right, "arm_right", "arm_joint_right", radius_shoulder_joint* vec3(0.1f, 0.65f, -0.1f));


    hierarchy.add(arm_joint, "arm_joint_left", "shoulder_left", radius_body * vec3(0.13f, 1.85f, -3.3f));
    hierarchy.add(arm_left, "arm_left", "arm_joint_left", radius_shoulder_joint* vec3(-0.1f, -0.65f, -0.1f));

    // Legs and joints:
    hierarchy.add(leg_joint, "leg_joint_left", "body", radius_body * vec3(1.3f, 1.3f, -9.3f));
    hierarchy.add(left_leg, "left_leg", "leg_joint_left", radius_leg_joint * vec3(0.2f, 0.0f, 0.0f));


    hierarchy.add(leg_joint, "leg_joint_right", "body", radius_body * vec3(1.3f, -1.2f, -9.3f));
    hierarchy.add(right_leg, "right_leg", "leg_joint_right", radius_leg_joint * vec3(0.2f, 0.0f, 0.0f));



    // Set the same shader for all the elements
    hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    timer.scale = 0.5f;
    // Initialize my head rotation timer:
    timeForHeadRot = 0.0f;
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    // stores delta t :
    float dt = timer.update();
    set_gui();

    // Current time
    const float t = timer.t;

    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the elements **/
    /** *************************************************************  **/

    // Updates the timer:
    timeForHeadRot += dt;

    // Period of rotation
    float rT = 2.0f;

    // acts like a timer for t_max=2 and t_min=0
    if(timeForHeadRot >= rT)timeForHeadRot -= rT;

    // Rotation of the head around the x axis
    mat3 const R_head = rotation_from_axis_angle_mat3({0,0,1}, 0.2f * std::sin(2*3.14f*(timeForHeadRot-0.4f)/rT) );


    // The head oscillate along the z direction
    hierarchy["head"].transform.rotation = R_head;

    // Rotation of the shoulder around the y axis
    mat3 const R_shoulder = rotation_from_axis_angle_mat3({0,1,0}, 0.7 * std::sin(2*3.14f*(t)) );
    // Rotation of the arm a round the y axis
    mat3 const R_arm = rotation_from_axis_angle_mat3({0,1,0}, 0.4f * std::sin(2*3.14f*(t)) );
    // Rotation of the arm a round the y axis
    mat3 const R_leg = rotation_from_axis_angle_mat3({0,1,0}, -0.3f * std::sin(2*3.14f*(t)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
    hierarchy["shoulder_left"].transform.rotation = R_shoulder;
    hierarchy["arm_left"].transform.rotation = R_arm;

    hierarchy["shoulder_right"].transform.rotation = Symmetry*R_shoulder; // apply the symmetry
    hierarchy["arm_right"].transform.rotation = R_arm; //note that the symmetry is already applied by the parent element

    hierarchy["left_leg"].transform.rotation = R_leg;
    hierarchy["right_leg"].transform.rotation = Symmetry*R_leg;


    hierarchy.update_local_to_global_coordinates();


    /** ********************* **/
    /** Display the hierarchy **/
    /** ********************* **/

    if(gui_scene.surface) // The default display
        draw(hierarchy, scene.camera);

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
        draw(hierarchy, scene.camera, shaders["wireframe"]);

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(hierarchy, scene.camera);

}


void scene_model::set_gui()
{
    ImGui::Text("Display: "); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe); ImGui::SameLine();
    ImGui::Checkbox("Surface", &gui_scene.surface);     ImGui::SameLine();
    ImGui::Checkbox("Skeleton", &gui_scene.skeleton);   ImGui::SameLine();

    ImGui::Spacing();
    ImGui::SliderFloat("Time", &timer.t, timer.t_min, timer.t_max);
    ImGui::SliderFloat("Time scale", &timer.scale, 0.1f, 3.0f);

}





#endif

