
#ifdef SCENE_FINAL_SCENE


using namespace vcl;

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

void set_human_keyframes(vcl::buffer<vec3t> &human_keyframes,
                         float walk_diameter,
                         vec3 human_initial_position,
                         vec3 human_body_z_translation,
                         buffer<vec3> terrain_position,
                         float terrain_length,
                         float terrain_width)
{

    for(int i = 0; i < human_keyframes.size(); ++i)
    {
        if(i <= 3)
        {
            human_keyframes[i].p = human_initial_position + evaluate_terrain(i * 0.1f + 0.5f, 0.5f, terrain_length, terrain_width);
        }
        else if(i < 6)
        {
            human_keyframes[i].p = human_keyframes[3].p + evaluate_terrain(walk_diameter /2 * std::sin((i - 3) *PI /3) + 0.5f,
                                                   walk_diameter /2 * (1 - std::cos((i - 3) *PI /3)) + 0.5f, terrain_length, terrain_width);
        }
        else if(i == 6)//We readjust the path in midwalk to remove aprox errors
        {
            human_keyframes[i].p = human_initial_position +  evaluate_terrain(0.8f, walk_diameter + 0.5f,  terrain_length, terrain_width);
        }
        else if(i <= 9)
        {
            human_keyframes[i].p = human_keyframes[6].p + evaluate_terrain(-( i - 6) * 0.1f + 0.5f, 0.5f, terrain_length, terrain_width);
        }
        else if(i < 12)
        {
            human_keyframes[i].p = human_keyframes[9].p - evaluate_terrain(walk_diameter /2 * std::sin((i - 9) *PI /3) + 0.5f,
                                                   walk_diameter /2 * (1 - std::cos((i - 9) *PI /3)) + 0.5f, terrain_length, terrain_width);
        }
        else if(i == 12)
        {
            human_keyframes[i].p = human_keyframes[0].p;
        }
        else
        {
            human_keyframes[i].p = human_keyframes[1].p;
        }

        // normalizes the coordinates
        const vec2 uv = reverse_evaluate_terrain(human_keyframes[i].p[0], human_keyframes[i].p[1], terrain_length, terrain_width);

        // The Human needs to be on the surface of the grass
        human_keyframes[i].p[2] = terrain_z_coord(terrain_position, uv[0], uv[1])
                                + human_body_z_translation[2];

        human_keyframes[i].t = i;
    }


}

vec3 compute_new_position(float t_Interp, vcl::buffer<vec3t> keyframe, float K)
{
    const int idx = index_at_value(t_Interp, keyframe);

    // We assume it is a closed curve trajectory
    const size_t N = keyframe.size();

    // Preparation of data for the linear interpolation
    // Parameters used to compute the linear interpolation
    const float t0 = (idx==0)? keyframe[N - 3].t - keyframe[N - 2].t : keyframe[idx-1].t; // = t_{i-1}
    const float t1 = keyframe[idx  ].t; // = t_i
    const float t2 = keyframe[idx+1].t; // = t_{i+1}
    const float t3 = keyframe[idx+2].t;; // = t_{i+2}

    const vec3& p0 = (idx==0)? keyframe[N - 3].p : keyframe[idx-1].p; // = p_{i-1}
    const vec3& p1 = keyframe[idx  ].p; // = p_i
    const vec3& p2 = keyframe[idx+1].p; // = p_{i+1}
    const vec3& p3 = keyframe[idx+2].p; // = p_{i+2}

    return cardinal_spline_interpolation(t_Interp, t0, t1, t2, t3, p0, p1, p2, p3, K);
}

mat3 compute_new_orientation_z(vec3 new_pos, vec3 old_pos)
{
    vec3 dir = (new_pos - old_pos);
    dir = dir / norm(dir);

    // computing the angle of the rotation depending on the z axis:
    float angleOfRot_z = (dir[0] >= 0)? std::atan(dir[1] /dir[0]) : std::atan(dir[1] /dir[0]) + PI;

    return rotation_from_axis_angle_mat3({0,0,1}, angleOfRot_z);
}

// Rotation on y axis
mat3 compute_new_orientation_y(vec3 new_pos, vec3 old_pos)
{
    vec3 dir = (new_pos - old_pos);
    dir = dir / norm(dir);

    // computing the angle of the rotation depending on the y axis:
    float angleOfRot_y = -std::asin(dir[2]);

    return rotation_from_axis_angle_mat3({0,1,0}, angleOfRot_y);
}

// Rotation on x axis
mat3 compute_new_orientation_x(vec3 new_pos, vec3 old_pos)
{
    vec3 dir = (new_pos - old_pos);
    dir = dir / norm(dir);

    // computing the angle of the rotation depending on the y axis:
    float angleOfRot_x = -std::asin(dir[2]);

    return rotation_from_axis_angle_mat3({1,0,0}, angleOfRot_x);
}

void set_bird_keyframes(vcl::buffer<vec3t> &bird_keyframes, float bird_z_coord, float slope, float R, float a2, float b2)
{

    // Size of the keyframe :
    const size_t N = bird_keyframes.size();

    // For period purposes
    bird_keyframes[N - 2].t = N-2;

    for(int i = 0; i < N; ++i)
    {
        bird_keyframes[i].t = i;

        // Middle of the trajectory
        int mid_point = (int)N/2 - 1;

        if(i <= mid_point)
        {
            bird_keyframes[i].p[2] = bird_z_coord - i * slope;
        }
        else  if(i< N-2)
        {
            bird_keyframes[i].p[2] = bird_keyframes[mid_point].p[2] + (i - mid_point) * slope;
        }

        if(i >= N - 2)
        {
            bird_keyframes[i].p = bird_keyframes[i - N + 2].p;
        }
        else
        {
            bird_keyframes[i].p[0] = a2 * R * std::cos(2 * 3.1415 / (bird_keyframes[N - 2].t) * bird_keyframes[i].t);
            bird_keyframes[i].p[1] = b2 * R * std::sin(2 * 3.1415 / (bird_keyframes[N - 2].t) * bird_keyframes[i].t);
        }



    }
}

#endif

