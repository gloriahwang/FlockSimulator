#ifndef CLOTHSIM_FLOCKUNIT_H
#define CLOTHSIM_FLOCKUNIT_H


#include "clothMesh.h"
#include "CGL/CGL.h"
#include "obstacle.h"
#include "nanogui/nanogui.h"

using namespace CGL;
using namespace std;

class Predator;
struct FlockUnit {
    FlockUnit(const Vector3D &position, const Vector3D &direction, nanogui::Color color)
            : position(position), direction(direction), color(color) {}

    void simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground, vector<FlockUnit *> *flock,
                  vector<Obstacle *> *obstacles, vector<Obstacle *> *food);
    void check_bounds(vector<Vector3D> *playground);
    void check_collisions_with_food(vector<FlockUnit *> *flock, vector<Obstacle *> *food);
    void reproduce(vector<FlockUnit *> *flock);
    void update_vertices();

    Vector3D position;
    Vector3D new_position;
    Vector3D direction;
    Vector3D acceleration;
    double MAX_SEE_AHEAD = 0.05; // For collision avoidance
    nanogui::Color color;
    nanogui::Color averaged_color;
    vector<Vector3D> vertices;
    Vector3D scatter_force = Vector3D();
    vector<Vector3D> *force_debug = new vector<Vector3D>();

    void generate_new_direction(vector<FlockUnit *> *flock, vector<Obstacle *> *obstacles, Vector3D mouse_position,
                                vector<Obstacle *> *food, vector<Predator *> *predators);
    void average_nearby_colors(vector<FlockUnit *> *flock);
    Vector3D avoid_predator_direction(vector<Predator *> *predators);
    Vector3D calculate_separation(vector<FlockUnit *> *flock);
    Vector3D calculate_cohesion(vector<FlockUnit *> *flock);
    Vector3D calculate_alignment(vector<FlockUnit *> *flock);
    Vector3D calculate_static_seek(vector<Obstacle *> *obstacles);
    Vector3D calculate_avoid_direction(vector<Obstacle *> *obstacles);
    Vector3D calculate_seek_or_avoid(Vector3D mouse_position);
    Vector3D calculate_seek_food(vector<Obstacle *> *food);

    static bool alignment_enabled;
    static bool cohesion_enabled;
    static bool separation_enabled;
    static bool noise_enabled;
    static bool seek_mouse_enabled;
    static bool avoid_mouse_enabled;
};

#endif //CLOTHSIM_FLOCKUNIT_H
