#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_BENCH_HIERARCHY


static float PI = 3.1415f;

struct gui_scene_structure
{
    bool wireframe   = false;
    bool surface     = true;
    bool skeleton    = false;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    vcl::hierarchy_mesh_drawable bench_hierarchy;
    vcl::hierarchy_mesh_drawable_display_skeleton hierarchy_visual_debug;


    gui_scene_structure gui_scene;



};

#endif


