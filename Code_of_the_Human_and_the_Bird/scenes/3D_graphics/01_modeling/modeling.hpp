#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_3D_GRAPHICS

// Stores some parameters that can be set from the GUI
struct gui_scene_structure
{
    bool wireframe;


};

// Store a vec3 (p) + time (t)
struct vec3t{
    vcl::vec3 p; // position
    float t;     // time
};

struct scene_model : scene_base
{

    /** A part must define two functions that are called from the main function:
     * setup_data: called once to setup data before starting the animation loop
     * frame_draw: called at every displayed frame within the animation loop
     *
     * These two functions receive the following parameters
     * - shaders: A set of shaders.
     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
     * - data: The part-specific data structure defined previously
     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
    */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    // visual representation of a surface
    vcl::mesh_drawable terrain;

    // visual representation of a cylinder
    vcl::mesh_drawable cylinder;

    // visual representation of a cone
    vcl::mesh_drawable cone;

    // visual reprensentation of the tree foliage
    vcl::mesh_drawable foliage;

    // Surface of the billboards
    vcl::mesh_drawable surface;

    // visual representation of a bird
    vcl::hierarchy_mesh_drawable hierarchy;
    vcl::hierarchy_mesh_drawable_display_skeleton hierarchy_visual_debug;


    // Stores all the random positions of the trees
    std::vector<vcl::vec3> tree_positions;

    // Stores all the random positions of the flowers
    std::vector<vcl::vec3> flower_positions;

    // Stores all the random positions of the grass billboards
    std::vector<vcl::vec3> grass_positions;

    // The texture of the terrain
    GLuint terrain_texture_id;

    // The texture of the flower billboard
    GLuint flower_texture_id;

    // The texture of the grass billboard
    GLuint grass_texture_id;

    // Timers for the bird animation:
    vcl::timer_interval timer;
    float timeForHeadRot;

    // Timer for the interpolation
    vcl::timer_interval timerInterp;

    // Key Frames
    vcl::buffer<vec3t> keyframes; // Given (position,time)



    gui_scene_structure gui_scene;
};

#endif


