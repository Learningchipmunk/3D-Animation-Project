
#ifdef SCENE_FINAL_SCENE


using namespace vcl;

// Functions declarations:
float evaluate_terrain_z(float u, float v);
vec3 evaluate_terrain(float u, float v, float length = 20.0, float width = 20.0);
mesh create_terrain(float length = 20.0, float width = 20.0);


// Evaluate height of the terrain for any (u,v) \in [0,1]
float evaluate_terrain_z(float u, float v)
{

    // Length of the arrays of variables :
    const int length = 6;

    //vec2 p0 ={0,0};
    vec2 pi[length]={{0,0}, {0.5,0.5}, {0.2,0.7}, {0.8,0.7}, {0.35,0.6}, {0.6, 0.2}};

    float hi[length] = {0.5, 3.5, 1.5, 0.5, -0.5, 2.0};

    float sigmai[length] = {0.5, 0.25, 0.2, 0.2, 0.15, 0.2};

    float temp = 0;

    for(int i=0; i<length; ++i)
    {
        const float d = norm(vec2(u,v) - pi[i])/sigmai[i];
        temp += hi[i]*std::exp(-d*d);
    }

    const float z = temp;

    return z;
}

// Evaluate 3D position of the terrain for any (u,v) \in [0,1]
vec3 evaluate_terrain(float u, float v, float length, float width)
{
    float height = 0.42f;
    float scaling = 3.0f;
    int octave = 7;
    float persistency = 0.47f;

    // Evaluate Perlin noise
    const float noise = perlin(scaling*u, scaling*v, octave, persistency);


    const float x = length*(u-0.5f);
    const float y = width*(v-0.5f);
    const float z = evaluate_terrain_z(u,v)+height*noise;

    return {x,y,z};
}

vec2 reverse_evaluate_terrain(float x, float y, float length, float width)
{
    const float u = x /length + 0.5f;
    const float v = v /width  + 0.5f;

    return {u, v};
}

// Generate terrain mesh
mesh create_terrain(buffer<vec3> &terrain_positions, float length, float width)
{
    // Number of samples of the terrain is N x N
    const size_t N = std::max((size_t)(100 *length /20), (size_t)(width *length /20));

    mesh terrain; // temporary terrain storage (CPU only)
    terrain.position.resize(N*N);

    // resizes the terrain texture
    terrain.texture_uv.resize(N*N);



    // Fill terrain geometry
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);

            // Compute coordinates
            terrain.position[kv+N*ku] = evaluate_terrain(u, v, length, width);

            // Compute texture coord
            terrain.texture_uv[kv+N*ku] = {ku%2,kv%2};
        }
    }


    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    const unsigned int Ns = N;
    for(unsigned int ku=0; ku<Ns-1; ++ku)
    {
        for(unsigned int kv=0; kv<Ns-1; ++kv)
        {
            const unsigned int idx = kv + N*ku; // current vertex offset

            const uint3 triangle_1 = {idx, idx+1+Ns, idx+1};
            const uint3 triangle_2 = {idx, idx+Ns, idx+1+Ns};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    terrain_positions = terrain.position;

    return terrain;
}

// Returns the 3D coordinates of the terrain for any (u,v) \in [0,1]
vec3 terrain_coord(buffer<vec3> terrain_positions, float u, float v)
{
    const size_t N = (size_t)std::sqrt(terrain_positions.size());
    const size_t ku = u*(N-1.0f);
    const size_t kv = v*(N-1.0f);

    return terrain_positions[kv + N * ku];
}


// Returns the z coordinate of the terrain for any (u,v) \in [0,1]
float terrain_z_coord(buffer<vec3> terrain_positions, float u, float v)
{
    const size_t N = (size_t)std::sqrt(terrain_positions.size());
    const size_t ku = u*(N-1.0f);
    const size_t kv = v*(N-1.0f);

    return terrain_positions[kv + N * ku][2];
}

// Billboard creation function:
void create_billboard_surface(mesh_drawable &billboard_surface, GLuint flower_texture_id, float billboard_scaling = 1.0f)
{
    const float billboard_length = billboard_scaling * 0.5f,
                billboard_width = billboard_scaling * 0.4f;

    // Create a quad with (u,v)-texture coordinates
    mesh billboard_surface_cpu;
    billboard_surface_cpu.position     = {{- billboard_width / 2,0,0},
                                          {billboard_width / 2,0,0},
                                          {billboard_width / 2, billboard_length,0},
                                          {- billboard_width / 2, billboard_length,0}};
    billboard_surface_cpu.texture_uv   = {{0,1}, {1,1}, {1,0}, {0,0}};
    billboard_surface_cpu.connectivity = {{0,1,2}, {0,2,3}};

    billboard_surface = mesh_drawable(billboard_surface_cpu);
    billboard_surface.uniform.shading = {1,0,0}; // set pure ambiant component (no diffuse, no specular) - allow to only see the color of the texture
    billboard_surface.texture_id = flower_texture_id;

}

#endif
