#ifndef CLOTHSIM_OBSTACLE_H
#define CLOTHSIM_OBSTACLE_H

#include "clothMesh.h"
#include "CGL/CGL.h"

using namespace CGL;
using namespace std;

struct Obstacle {
    enum e_obstacle_type {AVOID_TYPE = 1, STATIC_SEEK_TYPE = 2, FOOD_TYPE = 3};

    Obstacle(const Vector3D &position, double size, double detection_radius, e_obstacle_type obstacle_type)
      : position(position), size(size), detection_radius(detection_radius), obstacle_type(obstacle_type) {}

    Vector3D position;
    double size;
    double detection_radius;

    e_obstacle_type obstacle_type;
};

#endif //CLOTHSIM_OBSTACLE_H