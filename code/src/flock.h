#ifndef CLOTHSIM_FLOCK_H
#define CLOTHSIM_FLOCK_H

#include "clothMesh.h" // TODO: Get rid of this include but use something else so we have access to Vector3D
#include "flockUnit.h"
#include "predator.h"
#include "CGL/CGL.h"
#include "clothSimulator.h"

using namespace CGL;
using namespace std;

class ClothSimulator;
struct Flock {
    Flock(vector<FlockUnit *> *units, vector<Predator *> *predators)
            : units(units), predators(predators) {}

    void simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground, vector<Obstacle *> *obstacles,
                  vector<Obstacle *> *food, Vector3D mouse_position, vector<Predator *> *predators, ClothSimulator *cs);
    vector<FlockUnit *> *units;
    vector<Predator *> *predators;
};
#endif //CLOTHSIM_FLOCK_H
