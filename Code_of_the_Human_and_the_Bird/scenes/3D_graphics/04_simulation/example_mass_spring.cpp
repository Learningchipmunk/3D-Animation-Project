
#include "example_mass_spring.hpp"


#ifdef SCENE_MASS_SPRING_1D

using namespace vcl;


static void set_gui(timer_basic& timer);


/** Compute spring force applied on particle pi from particle pj */
vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
    // our u is from pi to pj, B to A.
    return (norm(pj - pi) - L0) * (pj - pi) / norm(pj - pi);;
}


void scene_model::setup_data(std::map<std::string,GLuint>& , scene_structure& , gui_structure& )
{
    // Initial position and speed of particles
    // ******************************************* //

    const int N = 12;   // number of points must be higher than 1

    for(int i = 0; i < N; ++i)
    {
        const vec3 p0 = {i*0.5f,0,0};
        const vec3 v0 = {0,0,0};

        particles.push_back({p0,v0});
    }
    pA.p = {0,0,0};     // Initial position of particle A
    pA.v = {0,0,0};     // Initial speed of particle A

    pB.p = {0.5f,0,0};  // Initial position of particle B
    pB.v = {0,0,0};     // Initial speed of particle B

    pC.p = {1.0f,0,0};  // Initial position of particle c
    pC.v = {0,0,0};     // Initial speed of particle C

    L0 = 0.4f; // Rest length between A and B


    // Display elements
    // ******************************************* //
    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0,0,1};

    sphere = mesh_primitive_sphere();
    sphere.uniform.transform.scaling = 0.05f;


    std::vector<vec3> borders_segments = {{-1,-1,-1},{1,-1,-1}, {1,-1,-1},{1,1,-1}, {1,1,-1},{-1,1,-1}, {-1,1,-1},{-1,-1,-1},
                                          {-1,-1,1} ,{1,-1,1},  {1,-1,1}, {1,1,1},  {1,1,1}, {-1,1,1},  {-1,1,1}, {-1,-1,1},
                                          {-1,-1,-1},{-1,-1,1}, {1,-1,-1},{1,-1,1}, {1,1,-1},{1,1,1},   {-1,1,-1},{-1,1,1}};
    borders = borders_segments;
    borders.uniform.color = {0,0,0};

}





void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();
    set_gui(timer);


    // Simulation time step (dt)
    float dt = timer.scale*0.01f;

    // Simulation parameters
    const float m  = 0.01f;        // particle mass in grams
    const float K  = 10.0f;         // spring stiffness
    const float mu = 0.01f;       // damping coefficient

    const vec3 g   = {0,-9.81f,0}; // gravity


//    // For C:
//    const vec3 f_springC  = spring_force(pC.p, pB.p, L0, K);
//    const vec3 f_dampingC  = -mu * pC.v;
//    const vec3 f_weightC = vec3{0,0,-m*9.8};
//    const vec3 FC = f_springC+f_weightC+f_dampingC;


    // Numerical Integration (Verlet)
    for(int i = 1; i < particles.size() -1; ++i)
    {
        // Forces
        const vec3 f_spring  = spring_force(particles[i].p, particles[i-1].p, L0, K) + spring_force(particles[i].p, particles[i+1].p, L0, K);
        const vec3 f_damping  = -mu * particles[i].v; // TO DO: correct this force value
        const vec3 f_weight = m*g; // TO DO: correct this force value
        const vec3 FB = f_spring+f_weight+f_damping;

        // update
        vec3& p = particles[i].p; // position of particle
        vec3& v = particles[i].v; // speed of particle

        v = v + dt*FB/m;
        p = p + dt*v;
    }

//    // Numerical Integration (Verlet)
//    {
//        // Only particle C should be updated
//        vec3& p = pC.p; // position of particle
//        vec3& v = pC.v; // speed of particle

//        v = v + dt*FC/m;
//        p = p + dt*v;
//    }



    // Display of the result

    // first mass
    sphere.uniform.transform.translation = particles[0].p;
    sphere.uniform.color = {0,0,0};
    draw(sphere, scene.camera, shaders["mesh"]);

    for(int i = 1; i < particles.size() -1; ++i)
    {
        sphere.uniform.transform.translation = particles[i].p;
        sphere.uniform.color = {1,0,0};
        draw(sphere, scene.camera, shaders["mesh"]);
    }

    // second mass
    sphere.uniform.transform.translation = particles[particles.size() -1].p;
    sphere.uniform.color = {0,0,0};
    draw(sphere, scene.camera, shaders["mesh"]);

    for(int i = 0; i < particles.size() -1; ++i)
    {
        segment_drawer.uniform_parameter.p1 = particles[i].p;
        segment_drawer.uniform_parameter.p2 = particles[i+1].p;
        segment_drawer.draw(shaders["segment_im"],scene.camera);
    }



    draw(borders, scene.camera, shaders["curve"]);
}


/** Part specific GUI drawing */
static void set_gui(timer_basic& timer)
{
    // Can set the speed of the animation
    float scale_min = 0.05f;
    float scale_max = 2.0f;
    ImGui::SliderScalar("Time scale", ImGuiDataType_Float, &timer.scale, &scale_min, &scale_max, "%.2f s");

    // Start and stop animation
    if (ImGui::Button("Stop"))
        timer.stop();
    if (ImGui::Button("Start"))
        timer.start();

}



#endif
