#pragma once

#include "main/scene_base/base.hpp"

#ifdef SCENE_TREE_MODELING


static float PI = 3.1415f;


struct gui_scene_structure
{
    bool wireframe   = false;
    bool surface     = true;
};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    vcl::mesh_drawable tree;

    gui_scene_structure gui_scene;
};

#endif


