
#include "final_scene.hpp"
#include "terrain_functions.cpp"
#include "interpolation_functions.cpp"
#include "bench_hierarchy.cpp"
#include "tree_modeling_functions.cpp"
#include "human_hierarchy.cpp"
#include "bird_hierarchy.cpp"

#ifdef SCENE_FINAL_SCENE



using namespace vcl;




void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    // ----------------------------------- Textures ----------------------------------- //

    // Load the billboard grass texture (with transparent background)
    GLuint flower_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/flower_billboard.png"), GL_REPEAT, GL_REPEAT );

    // Loads the terrain texture
    GLuint terrain_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/grass_texture.png") );

    // ----------------------------------- Terrain ----------------------------------- //

    // Create visual terrain surface
    float terrain_length = 40, terrain_width = 40;
    // Stores the positions for placement purposes
    terrain = create_terrain(terrain_positions, terrain_length, terrain_width);// In file functions
    terrain.uniform.color = {0.6f,0.85f,0.5f};
    terrain.uniform.shading.specular = 0.0f; // non-specular terrain material
    terrain.texture_id = terrain_texture_id;

    // ------------------------------ Surface for the billboards ------------------------------ //

    const float billboard_scaling = 3.0f;

    // Creates and sets up the billboard mesh_drawable, see terrain_functions.cpp
    create_billboard_surface(billboard_surface, flower_texture_id, billboard_scaling);

    // ------------------------------------- Tree ------------------------------------- //

    const float tree_scaling = 3.0f;

    // Creates and sets up the tree mesh_drawable, see tree_modeling_functions.cpp
    create_tree_mesh(tree, tree_scaling);

    // -------------------------------- Bench Hierarchy -------------------------------- //

    const float bench_scaling = 1.0f;

    // Creates and sets up the bench_hierarchy, see bench_hierarchy.cpp
    create_bench_hierarchy(bench_hierarchy, bench_initial_position, bench_scaling);

    // Set the same shader for all the elements
    bench_hierarchy.set_shader_for_all_elements(shaders["mesh"]);


    // -------------------------------- Human Hierarchy -------------------------------- //

    // Creates and sets up the human hierarchy, see human_hierarchy.cpp
    create_human_hierarchy(human_hierarchy,
                           human_body_z_translation,
                           human_initial_position,
                           terrain_length, terrain_width);

    // Set the same shader for all the elements
    human_hierarchy.set_shader_for_all_elements(shaders["mesh"]);


    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    // --------------------------------- Bird hierarchy Setup --------------------------------- //

    // Creates and sets up the bird hierarchy, see bird_hierarchy.cpp
    create_bird_hierarchy(bird_hierarchy, bird_initial_position);

    // Set the same shader for all the elements
    bird_hierarchy.set_shader_for_all_elements(shaders["mesh"]);


    // -------------------------------------- Key frames -------------------------------------- //

    // !-Human walk keyframes-! //
    const int number_of_frames_human = 14;
    human_keyframes = vcl::buffer<vec3t>(number_of_frames_human);

    // Diameter of the semicircle
    const float walk_diameter = 0.2;

    // Initializing the human keyframe
    set_human_keyframes(human_keyframes, walk_diameter, human_initial_position, human_body_z_translation, terrain_positions, terrain_length, terrain_width);

    // !-Bird flight keyframes-! //
    // Initializing the bird keyframe
    const int number_of_frames_bird = number_of_frames_human;
    bird_keyframes = vcl::buffer<vec3t>(number_of_frames_bird);

    // Radius of the movement:
    const float R = 4.0f;
    // Ellipsoide movement constants:
    const float a2 = 2.0f, b2 = 1.0f;
    // Trajectory slope:
    const float slope = 0.5f;


    // Initializing the bird keyframe
    set_bird_keyframes(bird_keyframes, bird_initial_position[2], slope, R, a2, b2);
    // ellipsoide type movement, if a2 == b2 then it is a circular motion


    // --------------------------- Timers --------------------------- //

    // !-Human hierarchy animation-! //
    timer.scale = 0.5f;
    // Initialize my head rotation timer:
    timeForHeadRot = 0.0f;

    // !-Human movement-! //
    // Set timer bounds
    // You should adapt these extremal values to the type of interpolation
    human_walk_timer.t_min = human_keyframes[0].t;                   // first time of the keyframe
    human_walk_timer.t_max = human_keyframes[human_keyframes.size()-1].t - 1;  // last time of the keyframe
    human_walk_timer.t     = human_walk_timer.t_min;

    // !-Bird movement-! //
    bird_flight_timer.t_min = bird_keyframes[0].t;
    bird_flight_timer.t_max = bird_keyframes[bird_keyframes.size()-1].t - 1;
    bird_flight_timer.t     = bird_flight_timer.t_min;

    /*

      Although both animation have the same key frame times and same timer.t values at all times,
      I decided to store nonetheless seperately these timers in case I chose to decorrelate both
      of their movements afterwards.

     */
}




void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{

    // stores delta t :
    float dt = timer.update();
    set_gui();

    // Current time
    const float t = timer.t;

    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the Human    **/
    /** *************************************************************  **/

    // Updates the timer:
    timeForHeadRot += dt;

    // Period of rotation
    float rT = 2.0f;

    // acts like a timer for t_max=2 and t_min=0
    if(timeForHeadRot >= rT)timeForHeadRot -= rT;

    // Rotation of the head around the z axis
    mat3 const R_head = rotation_from_axis_angle_mat3({0,0,1}, 0.2f * std::sin(2*PI*(timeForHeadRot-0.4f)/rT) );


    // The head oscillate along the z direction
    human_hierarchy["head"].transform.rotation = R_head;

    // Rotation of the shoulder around the y axis
    mat3 const R_shoulder = rotation_from_axis_angle_mat3({0,1,0}, 0.7 * std::sin(2*PI*(t)) );
    // Rotation of the arm a round the y axis
    mat3 const R_arm = rotation_from_axis_angle_mat3({0,1,0}, 0.4f * std::sin(2*PI*(t)) );
    // Rotation of the arm a round the y axis
    mat3 const R_leg = rotation_from_axis_angle_mat3({0,1,0}, -0.3f * std::sin(2*PI*(t)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
    human_hierarchy["shoulder_left"].transform.rotation = R_shoulder;
    human_hierarchy["arm_left"].transform.rotation = R_arm;

    human_hierarchy["shoulder_right"].transform.rotation = Symmetry*R_shoulder; // apply the symmetry
    human_hierarchy["arm_right"].transform.rotation = R_arm; //note that the symmetry is already applied by the parent element

    human_hierarchy["left_leg"].transform.rotation = R_leg;
    human_hierarchy["right_leg"].transform.rotation = Symmetry*R_leg;

    // ********************************************* //
    // Compute interpolated position at time t
    // ********************************************* //

    // stores delta t :
    dt = human_walk_timer.update();
    const float t_Human_Interp = human_walk_timer.t;

    // la valeur de K:
    const float K_human = 0.2;
    // avec k=0.2 on dirait vraiment qu'il marche, mais ce n'est pas très continue....
    // avec k=0.4 très fluide, mais on dirait qu'il flotte.

    // Compute the next position of the Human
    const vec3 p = compute_new_position(t_Human_Interp, human_keyframes, K_human);

    // Computes the new orientation of the Human
    mat3 const R_z = compute_new_orientation_z(p, human_hierarchy["body"].transform.translation);

    // Orients the human
    human_hierarchy["body"].transform.rotation = R_z;
    // Places the human
    human_hierarchy["body"].transform.translation = p;



    human_hierarchy.update_local_to_global_coordinates();


    /** *************************************************************  **/
    /** Compute the (animated) transformations applied to the Bird     **/
    /** *************************************************************  **/

    // Same constants as Human:
    //rT
    //t
    //Symetry


    // Rotation of the head around the x axis
    mat3 const R_bird_head = rotation_from_axis_angle_mat3({1,0,0}, 0.4f * std::sin(2*3.14f*(timeForHeadRot-0.4f)/rT) );


    // The head oscillate along the z direction
    bird_hierarchy["head"].transform.rotation = R_bird_head;

    // Rotation of the first part of the wing around the y axis
    mat3 const R_wing1 = rotation_from_axis_angle_mat3({0,1,0}, std::sin(4*3.14f*(t-0.4f)) );
    // Rotation of the second part of the wing around the y axis (delayed with respect to the first part)
    mat3 const R_wing2 = rotation_from_axis_angle_mat3({0,1,0}, 0.85f * std::sin(4*3.14f*(t-0.45f)) );

    // Set the rotation to the elements in the hierarchy
    bird_hierarchy["wing1_left"].transform.rotation = R_wing1;
    bird_hierarchy["wing2_left"].transform.rotation = R_wing2;

    bird_hierarchy["wing1_right"].transform.rotation = Symmetry*R_wing1; // apply the symmetry
    bird_hierarchy["wing2_right"].transform.rotation = R_wing2; //note that the symmetry is already applied by the parent element

    // ********************************************* //
    // Compute interpolated position at time t
    // ********************************************* //

    // stores delta t :
    dt = bird_flight_timer.update();
    const float t_bird_Interp = bird_flight_timer.t;

    // la valeur de K:
    const float K_bird = 0.4;

    // Compute the next position of the Bird
    const vec3 bird_p = compute_new_position(t_bird_Interp, bird_keyframes, K_bird);

    // Computes the new orientation of the Bird
    mat3 const bird_R_z = compute_new_orientation_z(bird_p, bird_hierarchy["body"].transform.translation);
    mat3 const bird_R_y = compute_new_orientation_y(bird_p, bird_hierarchy["body"].transform.translation);

    // Orients the Bird
    bird_hierarchy["body"].transform.rotation = bird_R_z * bird_R_y * rotation_from_axis_angle_mat3({0, 0, 1}, -PI/2);
    // Places the Bird
    bird_hierarchy["body"].transform.translation = bird_p;

    bird_hierarchy.update_local_to_global_coordinates();

    // -------------------------- Billboard settings -------------------------- //

    // Billboard orientation :
    billboard_surface.uniform.transform.rotation = scene.camera.orientation;


    // -------------------------- Drawing Block -------------------------- //


    if(gui_scene.surface) // The default display
    {
        draw(bird_hierarchy, scene.camera, shaders["mesh"]);
        draw(human_hierarchy, scene.camera, shaders["mesh"]);
        draw(terrain, scene.camera, shaders["mesh"]);

        // -------------------------- Tree and Benches drawings -------------------------- //
        const int nbre_of_trees = 5;
        const int nbre_of_benches = 4;// is always lower than trees
        const float distance_between_trees = 0.2f;

        for(int i = 0; i < nbre_of_trees; ++i)
        {
            // !-Bench drawing-! //
            if(i < nbre_of_benches)
            {
                // Computes the new bench coord:
                vec3 new_bench_coord = terrain_coord(terrain_positions, 0.1 + i * distance_between_trees + 0.25f * distance_between_trees, 0.8f);

                // Updates the coordinates
                bench_hierarchy["body"].transform.translation = bench_initial_position + new_bench_coord;
                //bench_hierarchy["body"].transform.rotation = compute_new_orientation_y(terrain_coord(terrain_positions, 0.1 + i * distance_between_trees + 0.75f * distance_between_trees, 0.9f), new_bench_coord);
                bench_hierarchy.update_local_to_global_coordinates();
                draw(bench_hierarchy, scene.camera, shaders["mesh"]);

            }


            // !-Tree drawing-! //
            tree.uniform.transform.translation = terrain_coord(terrain_positions, 0.1 + i * distance_between_trees, 0.9f);
            bench_hierarchy["body"].transform.translation = bird_p;
            draw(tree, scene.camera, shaders["mesh"]);
        }

        // Drawing flower Billboards:
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        for(int i = 0; i < nbre_of_benches; ++i)
        {

            // First flower near bench
            billboard_surface.uniform.transform.translation = terrain_coord(terrain_positions, 0.1 + i * distance_between_trees + 0.125f * distance_between_trees, 0.8f);
            draw(billboard_surface, scene.camera, shaders["mesh"]);

            // Second flower near bench
            billboard_surface.uniform.transform.translation = terrain_coord(terrain_positions, 0.1 + i * distance_between_trees + 0.875f * distance_between_trees, 0.8f);
            draw(billboard_surface, scene.camera, shaders["mesh"]);
        }

    }

    if(gui_scene.wireframe) // Display the hierarchy as wireframe
    {
        draw(bird_hierarchy, scene.camera, shaders["wireframe"]);
        draw(human_hierarchy, scene.camera, shaders["wireframe"]);
        draw(terrain, scene.camera, shaders["wireframe"]);
    }

    if(gui_scene.skeleton) // Display the skeleton of the hierarchy (debug)
        hierarchy_visual_debug.draw(human_hierarchy, scene.camera);

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

