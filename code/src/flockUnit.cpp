#include <fcntl.h>
#include "flockUnit.h"
#include "predator.h"

#define FLOCK_UNIT_HEIGHT 8
#define FLOCK_UNIT_WIDTH 4

#define ALIGNMENT_NEIGHBORHOOD 25
#define COHESION_NEIGHBORHOOD 50
#define SEPARATION_NEIGHBORHOOD 20
#define NEARBY_NEIGHBORHOOD 25
#define AVOID_PREDATOR_NEIGHBORHOOD 50

#define AVOID_PREDATOR_SPEED 3
#define AVOID_NEIGHBORHOOD 35
#define FOOD_SEEK_RADIUS 75
#define MAX_FORCE 0.5
#define SPEED 2

using namespace CGL;

bool FlockUnit::alignment_enabled = true;
bool FlockUnit::cohesion_enabled = true;
bool FlockUnit::separation_enabled = true;
bool FlockUnit::noise_enabled = true;
bool FlockUnit::seek_mouse_enabled = false;
bool FlockUnit::avoid_mouse_enabled = false;

void FlockUnit::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground, vector<FlockUnit *> *flock, vector<Obstacle *> *obstacles, vector<Obstacle *> *food) {
  // Generate the next position and use this to check collisions and if the unit leaves the playground
  direction = direction + acceleration;
  new_position = position + direction;
  check_bounds(playground);
  check_collisions_with_food(flock, food);

  position = new_position;
  update_vertices();
  color = averaged_color;
}

void FlockUnit::average_nearby_colors(vector<FlockUnit *> *units) {
  averaged_color = color;
  int nearby_tot = 1;
  for (FlockUnit *f : *units) {
    if (f == this) {
      continue;
    }
    double dist = (f->position - position).norm();
    if (dist <= 25) {
      averaged_color += f->color;
      nearby_tot++;
    }
  }
  if (nearby_tot > 1) {
    averaged_color[0] = max(averaged_color[0] + ((rand()/(float) RAND_MAX) * 0.7f - 0.35f), 0.0f);
    averaged_color[1] = max(averaged_color[1] + ((rand()/(float) RAND_MAX) * 0.7f - 0.35f), 0.4f);
    averaged_color[2] = max(averaged_color[2] + ((rand()/(float) RAND_MAX) * 0.7f - 0.35f), 0.0f);
    averaged_color /= (float) (nearby_tot);
    averaged_color[3] = 1.0f;
  }
}

void FlockUnit::generate_new_direction(vector<FlockUnit *> *flock, vector<Obstacle *> *obstacles, Vector3D mouse_position,
                                       vector<Obstacle *> *food, vector<Predator *> *predators) {
  force_debug->clear();

  Vector3D alignment = Vector3D(0, 0, 0);
  Vector3D cohesion = Vector3D(0, 0, 0);
  Vector3D separation = Vector3D(0, 0, 0);
  Vector3D noise = Vector3D(0, 0, 0);
  Vector3D mouse_seek = Vector3D(0, 0, 0);
  Vector3D mouse_avoid = Vector3D(0, 0, 0);
  Vector3D cruise = Vector3D(0, 0, 0);

  double alignment_factor = 5;
  double cohesion_factor = 9;
  double separation_factor = 13;
  double avoid_factor = 15;
  double noise_factor = 0.5;
  double mouse_seek_factor = 4;
  double mouse_avoid_factor = 150;
  double seek_food_factor = 100;
  double scatter_factor = 1000;
  double relaxation_time = 0.2;
  double static_seek_factor = 2;
  double avoid_predator_factor = 15;

  if (alignment_enabled)
    alignment = calculate_alignment(flock);
  if (cohesion_enabled)
    cohesion = calculate_cohesion(flock);
  if (separation_enabled)
    separation = calculate_separation(flock);
  if (noise_enabled) {
    double rx = (rand() / (double) RAND_MAX) * 2 - 1;
    double ry = (rand() / (double) RAND_MAX) * 2 - 1;
    noise = Vector3D(rx, ry, 0.0);
  }

  Vector3D avoid = calculate_avoid_direction(obstacles);
  Vector3D static_seek = calculate_static_seek(obstacles);
  Vector3D seek_food = calculate_seek_food(food);
  Vector3D avoid_predator_dir = avoid_predator_direction(predators);

  if (direction != Vector3D(0, 0, 0)) {
    cruise = direction.unit() * (1 / relaxation_time) * (SPEED - direction.norm());
  }

  if (mouse_position != NULL) {
    double distance_to_mouse = (mouse_position - position).norm();
    if (seek_mouse_enabled)
      mouse_seek = calculate_seek_or_avoid(mouse_position);
    else if (avoid_mouse_enabled && distance_to_mouse < AVOID_NEIGHBORHOOD)
      mouse_avoid = calculate_seek_or_avoid(mouse_position);
  }

  acceleration = alignment_factor * alignment +
                 cohesion_factor * cohesion +
                 separation_factor * separation +
                 avoid_factor * avoid +
                 mouse_seek_factor * mouse_seek +
                 mouse_avoid_factor * mouse_avoid +
                 noise_factor * noise +
                 scatter_factor * scatter_force +
                 static_seek_factor * static_seek +
                 seek_food_factor * seek_food +
                 avoid_predator_factor * avoid_predator_dir +
                 cruise;
  if (acceleration.norm() > MAX_FORCE) {
    acceleration = MAX_FORCE * acceleration.unit();
  }

  force_debug->push_back(5 * alignment_factor * alignment);
  force_debug->push_back(5 * cohesion_factor * cohesion);
  force_debug->push_back(5 * separation_factor * separation);
  force_debug->push_back(5 * avoid_factor * avoid);
  force_debug->push_back(5 * noise_factor * noise);
  force_debug->push_back(5 * static_seek_factor * static_seek);
  force_debug->push_back(5 * avoid_predator_factor * avoid_predator_dir);
  force_debug->push_back(5 * cruise);

  scatter_force *= 0.5;
}

void FlockUnit::check_bounds(vector<Vector3D> *playground) {
  double playground_min_x = numeric_limits<double>::max(), playground_min_y = numeric_limits<double>::max();
  double playground_max_x = -1.0 * numeric_limits<double>::max(), playground_max_y = -1.0 * numeric_limits<double>::max();
  for (int i = 0; i < playground->size(); i++) {
    double x = (*playground)[i].x;
    double y = (*playground)[i].y;

    playground_min_x = (x < playground_min_x) ? x : playground_min_x;
    playground_max_x = (x > playground_max_x) ? x : playground_max_x;

    playground_min_y = (y < playground_min_y) ? y : playground_min_y;
    playground_max_y = (y > playground_max_y) ? y : playground_max_y;
  }

  // If we cross the boundary region, appear from the other side
  if (new_position.x <= playground_min_x) {
    new_position.x = playground_max_x;
  } else if (new_position.x >= playground_max_x) {
    new_position.x = playground_min_x;
  }

  if (new_position.y <= playground_min_x) {
    new_position.y = playground_max_y;
  } else if (new_position.y >= playground_max_y) {
    new_position.y = playground_min_y;
  }
}

void FlockUnit::check_collisions_with_food(vector<FlockUnit *> *flock, vector<Obstacle *> *food) {
  int num_food = food->size();
  int i = 0;
  while (i < num_food) {
    Obstacle *f = (*food)[i];
    double distance = (f->position - position).norm();
    if (distance < f->size) {
      delete f;
      food->erase(std::remove(food->begin(), food->end(), f), food->end());
      num_food--;
      reproduce(flock);
    } else {
      i++;
    }
  }
}

void FlockUnit::reproduce(vector<FlockUnit*> *flock) {
  double rx = (rand() / (double) RAND_MAX) * 2 - 1;
  double ry = (rand() / (double) RAND_MAX) * 2 - 1;
  FlockUnit *u = new FlockUnit(position, Vector3D(rx, ry, 0), color);
  u->update_vertices();
  flock->push_back(u);
}

Vector3D FlockUnit::calculate_alignment(vector<FlockUnit *> *flock) {
  Vector3D alignment = Vector3D(0, 0, 0);
  int total = 0;

  for (FlockUnit *u : *flock) {
    if (u == this)
      continue;

    double distance = (u->position - position).norm();
    if (distance > SEPARATION_NEIGHBORHOOD && distance < ALIGNMENT_NEIGHBORHOOD) {
      if (u->direction != Vector3D(0, 0, 0))
        alignment += (u->direction).unit();
      total++;
    }
  }

  if (total > 0 && alignment.norm() != 0) {
    alignment /= total;
    alignment.normalize();
  }
  return alignment;
}


Vector3D FlockUnit::calculate_cohesion(vector<FlockUnit *> *flock) {
  Vector3D cohesion = Vector3D(0, 0, 0);
  int total = 0;
  for (FlockUnit *u : *flock) {
    if (u == this)
      continue;

    double distance = (u->position - position).norm();
    if (distance > ALIGNMENT_NEIGHBORHOOD && distance < COHESION_NEIGHBORHOOD) {
      cohesion += (u->position - position) / distance;
      total++;
    }
  }

  if (total > 0 && cohesion.norm() != 0) {
    cohesion /= total;
    cohesion.normalize();
  }
  return cohesion;
}

Vector3D FlockUnit::calculate_separation(vector<FlockUnit *> *flock) {
  Vector3D separation = Vector3D(0, 0, 0);
  int total = 0;

  for (FlockUnit *u : *flock) {
    if (u == this)
      continue;

    double distance = (u->position - position).norm();
    if (distance > 0 && distance < SEPARATION_NEIGHBORHOOD) {
      separation += (position - u->position) / distance;
      total++;
    }
  }

  if (total > 0 && separation.norm() != 0) {
    separation /= total;
    separation.normalize();
  }
  return separation;
}

Vector3D FlockUnit::calculate_avoid_direction(vector<Obstacle *> *obstacles) {
  Vector3D avoid = Vector3D(0, 0, 0);
  int total = 0;
  for (Obstacle * o : *obstacles) {
    if (o->obstacle_type == Obstacle::AVOID_TYPE) {
      double distance = (o->position - position).norm();
      if (distance < AVOID_NEIGHBORHOOD) {
        avoid += (position - o->position) / distance;
        total++;
      }
    }
  }
  if (total > 0 && avoid.norm() != 0) {
    avoid /= total;
    avoid.normalize();
  }
  return avoid;
}

Vector3D FlockUnit::calculate_static_seek(vector<Obstacle *> *obstacles) {
  Vector3D attract = Vector3D(0, 0, 0);
  int total = 0;
  for (Obstacle *o : *obstacles) {
    if (o->obstacle_type == Obstacle::STATIC_SEEK_TYPE) {
      Vector3D target = (o->position - position).unit() * SPEED;
      Vector3D movement = target - direction;
      attract += movement;
      total++;
    }
  }
  if (total > 0) {
    attract/=total;
  }
  return attract;
}

Vector3D FlockUnit::calculate_seek_food(vector<Obstacle *> *food) {
  Vector3D attract = Vector3D(0, 0, 0);
  double closest_distance = -1;

  for (Obstacle *f : *food) {
    double distance = (f->position - position).norm();
    if (distance < FOOD_SEEK_RADIUS) {
      if (closest_distance == -1 || distance < closest_distance) {
        Vector3D target = (f->position - position).unit();
        Vector3D movement = target - direction.unit();
        attract = movement;
      }
    }
  }

  return attract;
}

Vector3D FlockUnit::calculate_seek_or_avoid(Vector3D mouse_position) {
  double factor = (seek_mouse_enabled) ? 1 : -1;
  Vector3D target = factor * (mouse_position - position).unit();
  Vector3D movement = target - direction.unit();
  return movement.unit();
}

// Calculate the vertices of the triangle representing the unit here based on position and direction
void FlockUnit::update_vertices() {
  vertices.clear();

  Vector3D dir_to_use = Vector3D(1, 0, 0);
  if (direction != Vector3D(0, 0, 0)) {
    dir_to_use = direction.unit();
  }

  Vector3D position_to_use = position;

  Vector3D front_point = position_to_use + FLOCK_UNIT_HEIGHT * dir_to_use;

  Vector3D base_point = position_to_use - FLOCK_UNIT_HEIGHT * dir_to_use;
  Vector3D base_point_to_position_direction = (position_to_use - base_point).unit();
  Vector3D rotate_90_direction = Vector3D(-base_point_to_position_direction.y, base_point_to_position_direction.x, 0);

  Vector3D left_point = base_point + FLOCK_UNIT_WIDTH * rotate_90_direction.unit();
  Vector3D right_point = base_point - FLOCK_UNIT_WIDTH * rotate_90_direction.unit();

  vertices.push_back(front_point);
  vertices.push_back(left_point);
  vertices.push_back(right_point);
}

Vector3D FlockUnit::avoid_predator_direction(vector<Predator *> *predators) {
  Vector3D avoid = Vector3D(0, 0, 0);
  int total = 0;
  for (Predator * p : *predators) {
    double distance = (p->position - position).norm();
    if (distance < AVOID_PREDATOR_NEIGHBORHOOD) {
      avoid += (position - p->position) / distance;
      total++;
    }
  }
  if (total > 0 && avoid.norm() != 0) {
    avoid /= total;
    avoid.normalize();
  }
  return avoid;
}