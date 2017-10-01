#include "flock.h"
#include "flockUnit.h"
#include "predator.h"
#include "obstacle.h"
#include "clothSimulator.h"

using namespace CGL;

void Flock::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground, vector<Obstacle *> *obstacles,
                     vector<Obstacle *> *food, Vector3D mouse_position, vector<Predator *> *predators, ClothSimulator *cs) {
  // Flocking behavior influences new direction
  for (FlockUnit * f : *units) {
    f->generate_new_direction(units, obstacles, mouse_position, food, predators);
    f->average_nearby_colors(units);
  }

  for (Predator *p : *predators) {
    p->generate_new_direction(predators, obstacles, units);
  }

  // Check to see if you hit the boundaries and simulate next time step
  for (FlockUnit * f : *units) {
    f->simulate(frames_per_sec, simulation_steps, playground, units, obstacles, food);
  }

  int prey_size = units->size();
  for (Predator *p : *predators) {
    p->simulate(frames_per_sec, simulation_steps, playground, obstacles, units, cs);
  }


  int num_predators = predators->size();
  int i = 0;
  while (i < num_predators) {
    Predator *p = (*predators)[i];
    if (p->last_eat_time <= p->DEATH_TIME) {
      delete p;
      predators->erase(std::remove(predators->begin(), predators->end(), p), predators->end());
      num_predators--;
    } else {
      i++;
    }
  }
}