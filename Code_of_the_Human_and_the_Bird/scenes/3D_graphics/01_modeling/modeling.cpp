
#include "modeling.hpp"


#ifdef SCENE_3D_GRAPHICS

// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;




float evaluate_terrain_z(float u, float v);
vec3 evaluate_terrain(float u, float v);
mesh create_terrain();



std::vector<vec3> update_tree_position(const int Na)//O(n^2) time...
{
    std::vector<vec3> temp;

    for(int i = 0; i< Na; ++i)
    {
        float u = rand_interval();
        float v = rand_interval();

        // ---------- this part makes sure that the trees don't collide ------------- //

        // Distance between trees :
        const float step  = 0.7;
        // radius of the foliage is 0.4, in theory the min distance is 0.8 but 0.7 is fine

        // position of the trees
        vec3 position = evaluate_terrain(u, v);

        for(int j = 0; j < i; ++j)
        {
            if(temp[j][0] - step < position[0] && position[0] < temp[j][0] + step )
            {
                if(temp[j][1] - step < position[1] &&  position[1   ] < temp[j][1] + step)
                {
                    u = rand_interval();
                    v = rand_interval();
                    position = evaluate_terrain(u, v);

                    // Re initializes the loop
                    j = -1;
                }
            }
        }
        // ---------------------------  The end of the tests    ---------------------- //


        temp.push_back(position);
    }
    return temp;
}

std::vector<vec3> update_grass_position(std::vector<vec3> tree_position, const int Na)
{
    std::vector<vec3> temp;

    for(int i = 0; i< Na; ++i)
    {
        float u = rand_interval();
        float v = rand_interval();

        // ---------- this part makes sure that the grass don't collide ------------- //

        // Distance between grass bilboards :
        const float stepG  = 0.4;

        // Distance between grass bilboards and trees :
        const float stepT  = 2;

        // radius of the foliage is 0.4, in theory the min distance is 0.8 but 0.7 is fine

        // position of the trees
        vec3 position = evaluate_terrain(u, v);

        for(int j = 0; j < i; ++j)
        {
            if(temp[j][0] - stepG < position[0] && position[0] < temp[j][0] + stepG )
            {
                if(temp[j][1] - stepG < position[1] &&  position[1   ] < temp[j][1] + stepG)
                {
                    u = rand_interval();
                    v = rand_interval();
                    position = evaluate_terrain(u, v);

                    // Re initializes the loop
                    j = -1;
                }
            }

            // -- this part makes sure that the grass and the trees don't collide ------ //

            if(tree_position[j][0] - stepT < position[0] && position[0] < tree_position[j][0] + stepT)
            {
                if(tree_position[j][1] - stepT < position[1] &&  position[1   ] < tree_position[j][1] + stepT)
                {
                    u = rand_interval();
                    v = rand_interval();
                    position = evaluate_terrain(u, v);

                    // Re-initializes the loop
                    j = -1;
                }
            }
        }
        // ---------------------------  The end of the tests    ---------------------- //




        temp.push_back(position);
    }
    return temp;
}


mesh create_cylinder(float radius, float height)
{
    mesh cylinder; // temporary cylinder storage (CPU only)

    const float prop = 3.14159/10;
    float theta = 0;

    // the center of the cylinder
    vec2 center = {0.0f, 0.0f};

    cylinder.position = {{center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), 0},
                         {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), height},
                         {center[0] + radius*std::cos(theta + prop), center[1] + radius*std::sin(theta +prop), 0}};

    // Gives the drawing order of the points for the first triangle
    cylinder.connectivity = {{0, 1, 2}};

    theta+=prop;

    float tempHeight = height;

    for(int i = 3; theta <= 2*3.14159; ++i)
    {

        // Adds the new point
        cylinder.position.push_back(
        {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), tempHeight});

        // Makes sure it draws the new triangle
        cylinder.connectivity.push_back({i-2, i-1, i});


        switch(i%2)
        {
            case 1:
                theta += prop;
                tempHeight = 0;
                break;

            case 0:
                tempHeight = height;
                break;

            default:
                break;

        }
    }


    return cylinder;


}



mesh create_cone(float radius, float height, float z_offset)
{
    mesh cone; // temporary cone storage (CPU only)


    const float prop = 3.14159/10;
    float theta = 0;

    // the center of the cone
    vec3 center = {0.0f, 0.0f, 0.5f + z_offset};

    cone.position = {{center[0], center[1], center[2]},
                         {center[0], center[1], center[2] + height},
                         {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]}};

    theta+=prop;

    // Adds the new point
    cone.position.push_back(
    {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]});

    // Gives the drawing order of the points for the first triangle
    cone.connectivity = {{0, 2, 3}};
    cone.connectivity = {{1, 2, 3}};



    for(int i = 4; theta <= 2*3.14159; ++i)
    {
        theta += prop;
        // Adds the new point
        cone.position.push_back(
        {center[0] + radius*std::cos(theta), center[1] + radius*std::sin(theta), center[2]});

        // base triangle
        cone.connectivity.push_back({0, i-1, i});

        // top triangle
        cone.connectivity.push_back({1, i-1, i});
    }
    return cone;
}



mesh create_tree_foliage(float radius, float height, float z_offset)
{
    mesh m = create_cone(radius, height, 0);
    m.push_back( create_cone(radius, height, z_offset) );
    m.push_back( create_cone(radius, height, 2*z_offset) );

    return m;
}

static size_t index_at_value(float t, vcl::buffer<vec3t> const& v)
{
    const size_t N = v.size();
    assert(v.size()>=2);
    assert(t>=v[0].t);
    assert(t<v[N-1].t);

    size_t k=0;
    while( v[k+1].t<t )
        ++k;
    return k;
}

vec3 cardinal_spline_interpolation(float t, float t0, float t1, float t2, float t3, const vec3& p0, const vec3& p1, const vec3& p2, const vec3& p3, float mu)
{
    // t1 is t_n.
    const float s = (t-t1)/(t2-t1);

    // p1 is p_n
    const vec3 d1 = 2*mu*(p2 - p0)/(t2 - t0);
    const vec3 d2 = 2*mu*(p3 - p1)/(t3 - t1);

    const vec3 p = (2*pow(s,3) - 3*pow(s,2) + 1)*p1 + (pow(s,3) - 2*pow(s,2) + s)*d1 + (-2*pow(s,3) + 3*pow(s,2))*p2 + (pow(s,3) - pow(s,2))*d2;

    return p;
}


/** This function is called before the beginning of the animation loop
    It is used to initialize all part-specific data */
void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    // Create visual terrain surface
    terrain = create_terrain();
    terrain.uniform.color = {0.6f,0.85f,0.5f};
    terrain.uniform.shading.specular = 0.0f; // non-specular terrain material

    // Setup initial camera mode and position
    scene.camera.camera_type = camera_control_spherical_coordinates;
    scene.camera.scale = 10.0f;
    scene.camera.apply_rotation(0,0,0,1.2f);

    // Create the cylinder
    cylinder = mesh_drawable(create_cylinder(0.2, 0.5));
    cylinder.uniform.color= {0.5f, 0.35f, 0.05f};

    // Create the cone
    cone = mesh_drawable(create_cone(3.0, 3.0, 0.5));
    cone.uniform.color= {0.647059f,0.164706f,0.164706f};
    cone.uniform.transform.translation = {2,2,0};

    // Create the foliage
    foliage = mesh_drawable(create_tree_foliage(0.4,0.6,0.3));
    foliage.uniform.color= {0.5f,0.7f,0.45f};
    //foliage.uniform.transform.translation = {2,2,2};

    const int Na = 40;
    // Initialise the random positions :
    tree_positions = update_tree_position(Na);

    grass_positions = update_grass_position(tree_positions, (Na/4) * 3);

    flower_positions = update_grass_position(tree_positions, (Na/4) * 2);

    // ---------------- Textures ------------- //

    // Load the billboard grass texture (with transparent background)
    grass_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/billboard_grass.png"), GL_REPEAT, GL_REPEAT );

    // Load the billboard grass texture (with transparent background)
    flower_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/flower_billboard.png"), GL_REPEAT, GL_REPEAT );

    // Loads the terrain texture
    terrain_texture_id = create_texture_gpu( image_load_png("scenes/3D_graphics/02_texture/assets/grass_texture.png") );


    // --------------- Surface for the billboards ------------------------- //

    // Create a quad with (u,v)-texture coordinates
    mesh surface_cpu;
    surface_cpu.position     = {{-0.2f,0,0}, { 0.2f,0,0}, { 0.2f, 0.4f,0}, {-0.2f, 0.4f,0}};
    surface_cpu.texture_uv   = {{0,1}, {1,1}, {1,0}, {0,0}};
    surface_cpu.connectivity = {{0,1,2}, {0,2,3}};

    surface = surface_cpu;
    surface.uniform.shading = {1,0,0}; // set pure ambiant component (no diffuse, no specular) - allow to only see the color of the texture


    // ---------------- Bird Set_up part ------------------------------------ //

    const float radius_head = 0.25f;
    const float radius_body = radius_head/1.5f;
    const float length_wings = 0.3f;
    const float length_nose = 0.2f;


    // Constants for the ellipsoide :
    const float a = 4, b=2, c=2;

    // The geometry of the body is an ellipsoide
    mesh_drawable body = mesh_drawable( mesh_primitive_ellipsoide(radius_body, {0,0,0}, a, b, c, 40, 40));

    // The geometry of the head is a sphere
    mesh_drawable head = mesh_drawable( mesh_primitive_sphere(radius_head, {0,0,0}, 40, 40));

    // Geometry of the eyes: black spheres
    mesh_drawable eye = mesh_drawable(mesh_primitive_sphere(0.05f, {0,0,0}, 20, 20));
    eye.uniform.color = {0,0,0};

    // Geometry of the nose: orange cone
    mesh_drawable nose = mesh_drawable(mesh_primitive_cone(length_nose/2, vec3{0.0f,0.0f,0.0f}, vec3{0.0f,length_nose * 0.9,0.0f}, 20, 20));
    nose.uniform.color = {1,0.55,0};


    // Shoulder part and arm are displayed as cylinder
    mesh_drawable wing1 = mesh_primitive_quad({0,-radius_body * 1.5f,0}, {0,radius_body* 1.5f,0}, {-length_wings,radius_body* 1.5f,0}, {-length_wings,-radius_body* 1.5f,0});
    mesh_drawable wing2 = mesh_primitive_quad({0,-radius_body* 1.5f,0}, {0,radius_body* 1.5f,0}, {-length_wings*0.5,radius_body/2,0}, {-length_wings*0.5,-radius_body/2,0});



    // Body rotation:
    mat3 const R_body = rotation_from_axis_angle_mat3({0,0,1}, -3.14f/2);
    body.uniform.transform.rotation = R_body;


    // Build the hierarchy:
    // Syntax to add element
    //   hierarchy.add(visual_element, element_name, parent_name, (opt)[translation, rotation])

    // Body position is set with respect to some ratio of the head
    hierarchy.add(body, "body");

    hierarchy.add(head, "head", "body" , radius_body * vec3( 0.0f, a, b));


    // Eyes positions are set with respect to some ratio of the head
    hierarchy.add(eye, "eye_left", "head" , radius_head * vec3( 1/3.0f, 1/2.0f, 1/1.5f));
    hierarchy.add(eye, "eye_right", "head", radius_head * vec3(-1/3.0f, 1/2.0f, 1/1.5f));

    // Nose position is set
    hierarchy.add(nose, "nose", "head" , radius_head * vec3( 0.0f, 0.9f, 0.0f));

    // Set the left part of the head arm: shoulder-elbow-arm
    hierarchy.add(wing1, "wing1_left", "body", {-b * radius_body+0.05f,0,0}); // extremity of the spherical body
    hierarchy.add(wing2, "wing2_left", "wing1_left", {-length_wings, 0, 0});                        // the arm start at the center of the elbow

    // Set the right part of the head arm: similar to the left part excepted a symmetry is applied along x direction for the shoulder
    hierarchy.add(wing1, "wing1_right", "body",     {{b *radius_body-0.05f,0,0}, {-1,0,0, 0,1,0, 0,0,1}/*Symmetry*/ } );
    hierarchy.add(wing2, "wing2_right", "wing1_right", {-length_wings, 0, 0});


    // Set the same shader for all the elements
    hierarchy.set_shader_for_all_elements(shaders["mesh"]);



    // Initialize helper structure to display the hierarchy skeleton
    hierarchy_visual_debug.init(shaders["segment_im"], shaders["mesh"]);

    timer.scale = 0.5f;
    // Initialize my head rotation timer:
    timeForHeadRot = 0.0f;



    // Initializing timer for interpolation :
    const float hauteurInitiale = 2.5f;

    const float R = 3.0f;

    // Initial Keyframe data vector of (position, time)
    keyframes = { { {-0.1, 1, hauteurInitiale}   , 0.0f  },
                  { {0, 2, hauteurInitiale}    , 1.0f  },
                  { {1, 2, hauteurInitiale}    , 2.0f  },
                  { {2, 3, hauteurInitiale}    , 3.0f  },
                  { {1.5, 0, hauteurInitiale} , 4.0f  },
                  { {1.5, -1, hauteurInitiale} , 5.0f  },
                  { {1,-1, 0.5 + hauteurInitiale}   , 6.0f  },
                  { {0,-1, hauteurInitiale} , 7.0f },
                  { {0,0, hauteurInitiale}, 8.0f },
                  { {-0.1, 1, hauteurInitiale}   , 9.0f  },
                  { {0, 2,  hauteurInitiale}    , 10.0f  },
                  { {0, 2,  hauteurInitiale}    , 11.0f  },
                  { {0, 2,  hauteurInitiale}    , 12.0f  },
                  { {0, 2,  hauteurInitiale}    , 13.0f  },

                };

    // Ellipsoide movement constants :
    const float a2 = 1.0f, b2=1.0f;

    for(int i = 0; i < keyframes.size(); ++i)
    {
        if(i >= keyframes.size() - 2)
        {
            keyframes[i].p[0] = keyframes[i - keyframes.size() + 2].p[0];
            keyframes[i].p[1] = keyframes[i - keyframes.size() + 2].p[1];
        }
        else
        {
            keyframes[i].p[0] = a2 * R * std::cos(2 * 3.1415 / (keyframes[keyframes.size() - 2].t) * keyframes[i].t);
            keyframes[i].p[1] = b2 * R * std::sin(2 * 3.1415 / (keyframes[keyframes.size() - 2].t) * keyframes[i].t);
        }

    }

    // Set timer bounds
    // You should adapt these extremal values to the type of interpolation
    timerInterp.t_min = keyframes[0].t;                   // first time of the keyframe
    timerInterp.t_max = keyframes[keyframes.size()-1].t - 1;  // last time of the keyframe
    timerInterp.t = timer.t_min;



}



/** This function is called at each frame of the animation loop.
    It is used to compute time-varying argument and perform data data drawing */
void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    set_gui();

    glEnable( GL_POLYGON_OFFSET_FILL ); // avoids z-fighting when displaying wireframe

    // Before displaying a textured surface: bind the associated texture id
    //glBindTexture(GL_TEXTURE_2D, terrain_texture_id);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

    // Enable use of alpha component as color blending for transparent elements
    //  new color = previous color + (1-alpha) current color
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ------------------- Bird Drawing using hierarchy   ---------------------- //
    // stores delta t :
    float dt = timer.update();

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
    mat3 const R_head = rotation_from_axis_angle_mat3({1,0,0}, 0.4f * std::sin(2*3.14f*(timeForHeadRot-0.4f)/rT) );


    // The head oscillate along the z direction
    hierarchy["head"].transform.rotation = R_head;

    // Rotation of the shoulder around the y axis
    mat3 const R_shoulder = rotation_from_axis_angle_mat3({0,1,0}, std::sin(4*3.14f*(t-0.4f)) );
    // Rotation of the arm a round the y axis (delayed with respect to the shoulder)
    mat3 const R_arm = rotation_from_axis_angle_mat3({0,1,0}, 0.85f * std::sin(4*3.14f*(t-0.45f)) );
    // Symmetry in the x-direction between the left/right parts
    mat3 const Symmetry = {-1,0,0, 0,1,0, 0,0,1};

    // Set the rotation to the elements in the hierarchy
    hierarchy["wing1_left"].transform.rotation = R_shoulder;
    hierarchy["wing2_left"].transform.rotation = R_arm;

    hierarchy["wing1_right"].transform.rotation = Symmetry*R_shoulder; // apply the symmetry
    hierarchy["wing2_right"].transform.rotation = R_arm; //note that the symmetry is already applied by the parent element

    hierarchy.update_local_to_global_coordinates();


    // ********************************************* //
    // Compute interpolated position at time t
    // ********************************************* //

    // stores delta t :
    dt = timerInterp.update();
    const float tInterp = timerInterp.t;

    const int idx = index_at_value(tInterp, keyframes);

    // Assume a closed curve trajectory
    const size_t N = keyframes.size();

    // Preparation of data for the linear interpolation
    // Parameters used to compute the linear interpolation
    const float t0 = (idx==0)? keyframes[N - 2].t - keyframes[N - 1].t : keyframes[idx-1].t; // = t_{i-1}
    const float t1 = keyframes[idx  ].t; // = t_i
    const float t2 = keyframes[idx+1].t; // = t_{i+1}
    const float t3 = keyframes[idx+2].t;; // = t_{i+2}

    const vec3& p0 = (idx==0)? keyframes[N - 2].p : keyframes[idx-1].p; // = p_{i-1}
    const vec3& p1 = keyframes[idx  ].p; // = p_i
    const vec3& p2 = keyframes[idx+1].p; // = p_{i+1}
    const vec3& p3 = keyframes[idx+2].p; // = p_{i+2}


    // Compute the linear interpolation here
    //const vec3 p = linear_interpolation(t,t1,t2,p1,p2);

    // la valeur de K:
    const float K = 0.4;

    // Compute the cardinal spline interpolation here
    const vec3 p = cardinal_spline_interpolation(tInterp, t0, t1, t2, t3, p0, p1, p2, p3, K);

    vec3 dir = (p - hierarchy["body"].transform.translation) / dt;
    dir = dir / norm(dir);

    // computing the angle of the rotation depending on the axis:
    float angleOfRot_x = std::asin(dir[2]);
    float angleOfRot_z = (dir[1] >= 0)? std::acos(dir[0]) : -std::acos(dir[0]);
    if(timerInterp.t <= 0.1)angleOfRot_z = +3.1415f/2 + timerInterp.t/2;

    mat3 const R_z = rotation_from_axis_angle_mat3({0,0,1}, -3.1415f/2 + angleOfRot_z);
    mat3 const R_x = rotation_from_axis_angle_mat3({1,0,0}, angleOfRot_x);
    hierarchy["body"].transform.rotation = R_z*R_x;

    // Draws the bird
    hierarchy["body"].transform.translation = p;
    hierarchy.update_local_to_global_coordinates();
    draw(hierarchy, scene.camera);


    // ------------------- Draws the terrain ----------------------------------- //

    glPolygonOffset( 1.0, 1.0 );
    terrain.texture_id = terrain_texture_id;
    draw(terrain, scene.camera, shaders["mesh"]);

    // ------------------- Draws the trees and billboards ---------------------- //

    for(int i =0; i < tree_positions.size(); ++i)
    {
        // ------ for the trees
        foliage.uniform.transform.translation = tree_positions[i];
        cylinder.uniform.transform.translation = tree_positions[i];

        draw(cylinder, scene.camera, shaders["mesh"]);
        draw(foliage, scene.camera, shaders["mesh"]);

    }

    // ------ Draws the grass billboard :
    surface.texture_id = grass_texture_id;
    for(int i = 0; i < grass_positions.size(); ++i)
    {
        surface.uniform.transform.rotation = scene.camera.orientation;
        surface.uniform.transform.translation = grass_positions[i];
        draw(surface, scene.camera, shaders["mesh"]);
    }

    // ------ Draws the flower billboard :
    surface.texture_id = flower_texture_id;
    for(int i = 0; i < grass_positions.size(); ++i)
    {
        surface.uniform.transform.rotation = scene.camera.orientation;
        surface.uniform.transform.translation = flower_positions[i];
        draw(surface, scene.camera, shaders["mesh"]);
    }





    // ------------------- End of the the drawing block   ---------------------- //



    if( gui_scene.wireframe ){ // wireframe if asked from the GUI
        glPolygonOffset( 1.0, 1.0 );
        draw(terrain, scene.camera, shaders["wireframe"]);
        draw(cylinder, scene.camera, shaders["wireframe"]);
        //draw(cone, scene.camera, shaders["wireframe"]);
        draw(foliage, scene.camera, shaders["wireframe"]);
        draw(surface, scene.camera, shaders["wireframe"]);
        draw(hierarchy, scene.camera, shaders["wireframe"]);
    }
}



// Evaluate height of the terrain for any (u,v) \in [0,1]
float evaluate_terrain_z(float u, float v)
{

    // Length of the arrays of variables :
    const int length = 5;

    //vec2 p0 ={0,0};
    vec2 pi[length]={{0,0}, {0.5,0.5}, {0.2,0.7}, {0.8,0.7}, {0.35,0.6}};

    float hi[length] = {0.5, 1.5, 1.5, 0.5, -0.5};

    float sigmai[length] = {0.5, 0.15, 0.2, 0.2, 0.15};

    float temp =0;

    for(int i=0; i<length; ++i)
    {
        const float d = norm(vec2(u,v) - pi[i])/sigmai[i];
        temp += hi[i]*std::exp(-d*d);
    }

    const float z = temp;

    return z;
}

// Evaluate 3D position of the terrain for any (u,v) \in [0,1]
vec3 evaluate_terrain(float u, float v)
{
    float height = 0.42f;
    float scaling = 3.0f;
    int octave = 7;
    float persistency = 0.47f;

    // Evaluate Perlin noise
    const float noise = perlin(scaling*u, scaling*v, octave, persistency);


    const float x = 20*(u-0.5f);
    const float y = 20*(v-0.5f);
    const float z = evaluate_terrain_z(u,v)+height*noise;

    return {x,y,z};
}

// Generate terrain mesh
mesh create_terrain()
{
    // Number of samples of the terrain is N x N
    const size_t N = 100;

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
            terrain.position[kv+N*ku] = evaluate_terrain(u,v);

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

    return terrain;
}

void scene_model::set_gui()
{
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe);
}



#endif

