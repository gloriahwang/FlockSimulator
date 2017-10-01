#include <fcntl.h>
#include "predator.h"
#include "clothSimulator.h"

#define PREDATOR_HEIGHT 8
#define PREDATOR_WIDTH 8
#define AVOID_NEIGHBORHOOD 50
#define SEPARATION_NEIGHBORHOOD 50
#define PREY_SEEK_RADIUS 300
#define SPEED_UP_RADIUS 80
#define MAX_FORCE 0.1
#define SPEED 1.5

using namespace CGL;

void Predator::simulate(double frames_per_sec, double simulation_steps, vector<Vector3D> *playground, vector<Obstacle *> *obstacles, vector<FlockUnit *> *prey, ClothSimulator *cs) {
  /* Colors on main body:
   * - Full from eating = red
   * - Else we are the base color which should slowly degrade to white
   *
   * Colors on booster:
   * - Speed on cool down = grey
   * - Speed available = green
   */

  // If we are ready to eat again and our speed is not on cool down OR
  if (last_eat_time == 0) {
    color = base_color;
  } else if (last_eat_time < 0) {

    // If we are waiting to eat, we get whiter as we approach death or check to see if we've eaten
    if (check_collisions_with_food(prey, cs)) {
      last_eat_time = EAT_COOLDOWN;
      color = full_color;
    } else {
      color[0] = min(1.0f, color[0] + 1.0f / (float) abs(DEATH_TIME));
      color[1] = min(1.0f, color[1] + 1.0f / (float) abs(DEATH_TIME));
      color[2] = min(1.0f, color[2] + 1.0f / (float) abs(DEATH_TIME));
    }
  }

  if (last_speed_up_time == 0) {
    boost_color = can_speed_color;
  }

  /* Check if we can speed up:
   * - We aren't full
   * - We aren't currently speeding
   * - We have waited long enough since the last use of speed
   */
  if (last_eat_time < 0 && initiate_speed_up == 0 && last_speed_up_time == 0) {
    int found_index;
    FlockUnit * closest_prey = find_closest_prey(prey, &found_index);
    if (closest_prey != NULL) {
      double distance = (position - closest_prey->position).norm();
      if (distance < SPEED_UP_RADIUS && distance != 0) {
        acceleration = (closest_prey->position - position).unit() * SPEED_UP_ACCELERATION;
        speed_up_direction = acceleration.unit(); // Store for slowing down
        direction = Vector3D(0, 0, 0);
        initiate_speed_up = SPEED_COOLDOWN;
        last_speed_up_time = USE_SPEED_EVERY;
        boost_color = cant_speed_color;
      }
    }
  } else if (initiate_speed_up > 0) {
    acceleration = -0.75 * speed_up_direction * (SPEED_UP_ACCELERATION / SPEED_COOLDOWN);
  }

  // Update counters
  last_eat_time--;
  if (initiate_speed_up != 0)
    initiate_speed_up--;
  if (last_speed_up_time != 0)
    last_speed_up_time--;

  direction = direction + acceleration;
  // Limit max speed so predators don't speed out of control
  if (direction.norm() > SPEED && initiate_speed_up == 0) {
    direction = direction.unit() * SPEED;
  }

  new_position = position + direction;
  check_bounds(playground);
  position = new_position;
  update_vertices();
}


void Predator::check_bounds(vector<Vector3D> *playground) {
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

bool Predator::check_collisions_with_food(vector<FlockUnit *> *prey, ClothSimulator *cs) {

  int found_index = 0;
  FlockUnit * closest_prey = find_closest_prey(prey, &found_index);

  if (closest_prey != NULL) {
    double distance = (closest_prey->position - position).norm();
    if (distance < EAT_RANGE) {
      delete closest_prey;
      prey->erase(std::remove(prey->begin(), prey->end(), closest_prey), prey->end());

      // If we're in tracking mode, we need to adjust so the dead index is not used
      if (cs->enable_tracking_mode) {
        if (prey->size() == 0) {
          cs->enable_tracking_mode = false;
        } else if (closest_prey == cs->unit_to_track) {
          cs->select_unit_for_tracking(-1);
        } else {
          cs->unit_to_track_index = (found_index < cs->unit_to_track_index) ? cs->unit_to_track_index - 1 : cs->unit_to_track_index;
        }
      }
      return true;
    }
  }

  return false;
}

FlockUnit * Predator::find_closest_prey(vector<FlockUnit *> *prey, int *found_index) {
  int num_food = prey->size();
  int i = 0;

  double closest_distance = -1;
  FlockUnit *closest_prey = NULL;

  while (i < num_food) {
    FlockUnit *p = (*prey)[i];
    double distance = (p->position - position).norm();
    if (closest_distance == -1 || distance < closest_distance) {
      closest_distance = distance;
      closest_prey = p;
      *found_index = i;
      num_food--;
    } else {
      i++;
    }
  }

  return closest_prey;
}

void Predator::generate_new_direction(vector<Predator *> *predators, vector<Obstacle *> *obstacles, vector<FlockUnit *> *prey) {
  double separation_factor = 50;
  double avoid_factor = 15;
  double seek_food_factor = 50;
  double noise_factor = 0.5;
  double relaxation_time = 0.2;

  Vector3D separation = calculate_separation(predators);
  Vector3D avoid = calculate_avoid_direction(obstacles);
  Vector3D seek_food = Vector3D(0, 0, 0);
  if (last_eat_time <= 0)
    seek_food = calculate_seek_food(prey);

  double rx = (rand() / (double) RAND_MAX) * 2 - 1;
  double ry = (rand() / (double) RAND_MAX) * 2 - 1;
  Vector3D noise = Vector3D(rx, ry, 0.0);
  Vector3D cruise = Vector3D(0, 0, 0);

  if (direction.norm() != 0) {
    cruise = direction.unit() * (1 / relaxation_time) * (SPEED - direction.norm());
  }

  acceleration = separation_factor * separation +
                 avoid_factor * avoid +
                 noise_factor * noise +
                 seek_food_factor * seek_food +
                 cruise;
  if (acceleration.norm() > MAX_FORCE) {
    acceleration = MAX_FORCE * acceleration.unit();
  }
}

void Predator::update_vertices() {
  vertices.clear();

  Vector3D dir_to_use = Vector3D(1, 0, 0);
  if (direction != Vector3D(0, 0, 0)) {
    dir_to_use = direction.unit();
  }

  // Calculate vertices for body triangle

  Vector3D position_to_use = position;

  Vector3D front_point = position_to_use + PREDATOR_HEIGHT * dir_to_use;

  Vector3D base_point = position_to_use - PREDATOR_HEIGHT * dir_to_use;
  Vector3D base_point_to_position_direction = (position_to_use - base_point).unit();
  Vector3D rotate_90_direction = Vector3D(-base_point_to_position_direction.y, base_point_to_position_direction.x, 0);

  Vector3D left_point = base_point + PREDATOR_WIDTH * rotate_90_direction.unit();
  Vector3D right_point = base_point - PREDATOR_WIDTH * rotate_90_direction.unit();

  vertices.push_back(front_point);
  vertices.push_back(left_point);
  vertices.push_back(right_point);

  // Calculate vertices for booster triangle

  Vector3D booster_front = position_to_use - PREDATOR_HEIGHT * dir_to_use;
  Vector3D booster_base = booster_front - PREDATOR_HEIGHT * dir_to_use;
  Vector3D booster_left = booster_base + PREDATOR_WIDTH * 0.75 * rotate_90_direction.unit();
  Vector3D booster_right = booster_base - PREDATOR_WIDTH * 0.75 * rotate_90_direction.unit();

  vertices.push_back(booster_front);
  vertices.push_back(booster_left);
  vertices.push_back(booster_right);
}

Vector3D Predator::calculate_avoid_direction(vector<Obstacle *> *obstacles) {
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


Vector3D Predator::calculate_seek_food(vector<FlockUnit *> *prey) {
  Vector3D attract = Vector3D(0, 0, 0);
  double closest_distance = -1;
  int total = 0;

  for (FlockUnit *p : *prey) {
    double distance = (p->position - position).norm();
    if (distance < PREY_SEEK_RADIUS) {
      if ((closest_distance == -1 || distance < closest_distance) && direction.norm() != 0) {
        attract += (p->position - position).unit() / distance;
        total++;
      }
    }
  }
  if (total > 0 && attract.norm() != 0) {
    attract /= total;
    attract.normalize();
  }
  return attract;
}

Vector3D Predator::calculate_separation(vector<Predator *> *predators) {
  Vector3D separation = Vector3D(0, 0, 0);
  int total = 0;

  for (Predator *p : *predators) {
    if (p == this)
      continue;

    double distance = (p->position - position).norm();
    if (distance > 0 && distance < SEPARATION_NEIGHBORHOOD) {
      separation += (position - p->position) / distance;
      total++;
    }
  }

  if (total > 0 && separation.norm() != 0) {
    separation /= total;
    separation.normalize();
  }
  return separation;
}