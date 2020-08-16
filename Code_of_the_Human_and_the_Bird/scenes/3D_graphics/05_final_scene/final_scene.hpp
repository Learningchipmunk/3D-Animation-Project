#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_FINAL_SCENE

// Global Var:
static const float PI = 3.1415f;


struct gui_scene_structure
{
    bool wireframe   = false;
    bool surface     = true;
    bool skeleton    = false;
};

// --- Key frames structure --- //
// Store a vec3 (p) + time (t)
struct vec3t
{
    vcl::vec3 p; // position
    float t;     // time
};

struct scene_model : scene_base
{
    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    // !-Terrain Variables-!//
    // visual representation of a surface
    vcl::mesh_drawable terrain;
    // Position matrix
    vcl::buffer<vcl::vec3> terrain_positions;
    // billboard mesh_drawable
    vcl::mesh_drawable billboard_surface;


    // !-Human Variables-!//
    // The translation required in order to make the human walk on the terrain.
    vcl::vec3 human_body_z_translation;
    // The initial position of the human
    vcl::vec3 human_initial_position;
    // the human visual representation
    vcl::hierarchy_mesh_drawable human_hierarchy;
    // The human walk keyframes
    vcl::buffer<vec3t> human_keyframes; // Given (position,time)
    // Timer for human interpolation
    vcl::timer_interval human_walk_timer;


    // !-Bird Variables-!//
    // The initial position of the bird
    vcl::vec3 bird_initial_position;
    // the human visual representation
    vcl::hierarchy_mesh_drawable bird_hierarchy;
    // The bird's flight path
    vcl::buffer<vec3t> bird_keyframes; // Given (position,time)
    // Timer for Bird interpolation
    vcl::timer_interval bird_flight_timer;

    // !-Bench Variables-!//
    // Bench Structure:
    vcl::hierarchy_mesh_drawable bench_hierarchy;
    // The initial position of the bench
    vcl::vec3 bench_initial_position;

    // Tree Structure:
    vcl::mesh_drawable tree;


    vcl::hierarchy_mesh_drawable_display_skeleton hierarchy_visual_debug;






    gui_scene_structure gui_scene;

    // timer for human animation
    vcl::timer_interval timer;
    float timeForHeadRot;
};

#endif


