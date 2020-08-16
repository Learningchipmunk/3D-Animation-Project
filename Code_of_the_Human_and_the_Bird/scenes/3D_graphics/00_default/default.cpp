
#include "default.hpp"

#include <random>

#ifdef SCENE_DEFAULT_3D_GRAPHICS

// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;




mesh create_terrain()
{
    // Number of samples of the terrain is N x N
    const unsigned int N = 100;

    mesh terrain; // temporary terrain storage (CPU only)
    terrain.position.resize(N*N);

    // Fill terrain geometry
    for(unsigned int ku=0; ku<N; ++ku)
    {
        for(unsigned int kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);

            // Compute the local surface function
            const float x = 5*(u-0.5f);
            const float y = 5*(v-0.5f);
            const float z = std::exp(-(x*x*x+y*y*y));

            // Store vertex coordinates
            terrain.position[kv+N*ku] = {x,y,z};
        }
    }


    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    for(size_t ku=0; ku<N-1; ++ku)
    {
        for(size_t kv=0; kv<N-1; ++kv)
        {
            const unsigned int idx = kv + N*ku; // current vertex offset

            const uint3 triangle_1 = {idx, idx+1+N, idx+1};
            const uint3 triangle_2 = {idx, idx+N, idx+1+N};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    return terrain;
}




/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    // Create mesh structure (stored on CPU)
    // *************************************** //
    mesh quadrangle;
    // Fill position buffer to model a unit quadrangle
    quadrangle.position = {{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}};
    // Set the connectivity (quadrangle made of two triangles)
    quadrangle.connectivity = {{0,1,2}, {0,2,3}};


    // Transfert mesh onto GPU (ready to be displayed)
    // *************************************** //

    // Convert a mesh to mesh_drawable structure
    //  = mesh_drawable stores VBO and VAO, and allows to set uniform parameters easily
    surface = mesh_drawable(quadrangle);

    // Can attach a default shader to the mesh_drawable element
    surface.shader = shaders["mesh"];

    // Example of uniform parameter setting: color of the shape (used in the shader)
    surface.uniform.color = {1.0f, 1.0f, 0.6f};



    // Create a mesh approximating a sphere (unit radius by default)
    mesh sphere_cpu = mesh_primitive_sphere(); // mesh_primitive_sphere is a helper function (several primitive are available)
    // Send sphere_cpu data onto GPU
    sphere = mesh_drawable(sphere_cpu);
    // Set uniform parameter of the sphere
    sphere.uniform.color = {1,0,0};                  //red sphere
    sphere.uniform.transform.translation = {-0.1f,0.5f,0.25f}; // translate sphere display
    sphere.uniform.transform.scaling = 0.1f;                   // scale the sphere to new radius for its display
    sphere.shader = shaders["mesh"]; // associate default shader to sphere

    // Create the terrain
    mesh terrain_cpu = create_terrain();
    // Send sphere_cpu data onto GPU
    terrain = mesh_drawable(terrain_cpu);
    terrain.shader = shaders["mesh"]; // associate default shader to terrain


}




/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    // Drawing call: need to provide the camera information (use the default shader if it has been set previously)
    draw(surface, scene.camera);


    // Visualiser les arêtes des triangles associés au maillage.
    draw(surface, scene.camera, shaders["wireframe"]);


    // appel à l’affichage de la sphère
    //draw(sphere, scene.camera);



    // Il est possible de réutiliser une même sphère,
    // mais d’afficher celle-ci à plusieurs reprise en modifiant ces paramètres uniforms avant chaque affichage.
    sphere.uniform.color = {1,1,0};
    sphere.uniform.transform.translation = {0,0,0.5};
    draw(sphere, scene.camera);

    sphere.uniform.color = {0,0,1};
    sphere.uniform.transform.translation = {1,0,0.5};
    draw(sphere, scene.camera);


    // Drawing the terrain
    draw(terrain, scene.camera);

    // Drawing the maillage of the terrain
    draw(terrain, scene.camera, shaders["wireframe"]);









}













#endif

