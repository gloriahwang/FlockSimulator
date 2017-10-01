#ifndef CLOTHSIM_PREDATOR_H
#define CLOTHSIM_PREDATOR_H

#include "clothMesh.h"
#include "clothSimulator.h"
#include "CGL/CGL.h"
#include "obstacle.h"
#include "flockUnit.h"
#include "nanogui/nanogui.h"

using namespace CGL;
using namespace std;

class ClothSimulator;
struct Predator {
    Predator(const Vector3D &position, const Vector3D &direction, nanogui::Color color)
            : position(position), direction(direction), color(color), base_color(color) {}

    void simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground,
                  vector<Obstacle *> *obstacles, vector<FlockUnit *> *prey, ClothSimulator *cs);
    void check_bounds(vector<Vector3D> *playground);
    bool check_collisions_with_food(vector<FlockUnit *> *prey, ClothSimulator *cs);
    FlockUnit *find_closest_prey(vector<FlockUnit *, allocator<FlockUnit *>> *prey, int *found_index);
    void update_vertices();

    Vector3D position;
    Vector3D new_position;
    Vector3D direction;
    Vector3D acceleration;
    Vector3D speed_up_direction;
    nanogui::Color color;
    nanogui::Color boost_color;
    nanogui::Color base_color;
    nanogui::Color full_color = nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f);
    nanogui::Color cant_speed_color = nanogui::Color(0.5f, 0.5f, 0.5f, 1.0f);
    nanogui::Color can_speed_color = nanogui::Color(144.0f/255, 245.0f/255, 0.0f, 1.0f);
    vector<Vector3D> vertices;
    double EAT_RANGE = 10.0;
    double SPEED_UP_ACCELERATION = 6.0;
    double last_eat_time = 0.0;
    double initiate_speed_up = 0.0; // Duration of the speed up
    double last_speed_up_time = 0.0; // Last time speed up was used
    double EAT_COOLDOWN = 80.0;
    double SPEED_COOLDOWN = 40.0;
    double USE_SPEED_EVERY = SPEED_COOLDOWN + 25.0;
    double DEATH_TIME = -600.0;

    void generate_new_direction(vector<Predator *> *predators, vector<Obstacle *> *obstacles, vector<FlockUnit *> *prey);
    Vector3D calculate_avoid_direction(vector<Obstacle *> *obstacles);
    Vector3D calculate_separation(vector<Predator *> *predators);
    Vector3D calculate_seek_food(vector<FlockUnit *> *prey);

};

#endif //CLOTHSIM_PREDATOR_H
