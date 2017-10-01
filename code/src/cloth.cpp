#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double h_space = this->height / (double) (num_height_points - 1);
  double w_space = this->width / (double) (num_width_points - 1);

  // point mass construction
  if (orientation == HORIZONTAL) {   // horizontal
    for (int y = 0; y < num_height_points; y++) {
      for (int x = 0; x < num_width_points; x++) {
        Vector3D pos = Vector3D(x * w_space, 1.0,  y * h_space);
        PointMass *m = new PointMass(pos, false);

        if (pinned.size() > 0) {
          for (int i = 0; i < pinned.size(); i++) {
            if (pinned[i][0] == x && pinned[i][1] == y) {
              m->pinned = true;
              break;
            }
          }
        }

        point_masses.push_back(*m);
      }
    }
  } else if (orientation == VERTICAL) {    // vertical
    for (int y = 0; y < num_height_points; y++) {
      for (int x = 0; x < num_width_points; x++) {
        double r = rand()/(1.0 * RAND_MAX);
        double z = (r * 2.0 - 1.0)/ 1000.0;
        Vector3D pos = Vector3D(x * w_space, y * h_space, z);
        PointMass *m = new PointMass(pos, false);

        if (pinned.size() > 0) {
          for (int i = 0; i < pinned.size(); i++) {
            if (pinned[i][0] == pos.x && pinned[i][1] == pos.y) {
              m->pinned = true;
              break;
            }
          }
        }

        point_masses.push_back(*m);
      }
    }
  }

  // spring construction
  for (int y = 0; y < num_height_points; y++) {
    for (int x = 0; x < num_width_points; x++) {
      int mainIndex = (y * num_width_points) + x;
      PointMass* mainMass = &(point_masses[mainIndex]);

      // structural (left & above)
      if (x != 0) {
        int leftIndex = (y * num_width_points) + (x - 1);
        PointMass* leftMass = &(point_masses[leftIndex]);
        Spring* s1 = new Spring(mainMass, leftMass, STRUCTURAL);
        springs.push_back(*s1);
      }
      if (y != 0) {
        int topIndex = ((y - 1) * num_width_points) + x;
        PointMass* topMass = &(point_masses[topIndex]);
        Spring* s2 = new Spring(mainMass, topMass, STRUCTURAL);
        springs.push_back(*s2);
      }

      // shearing (diagonal upper left & right)
      if (y != 0 && x != 0) {
        int upperLeftIndex = ((y - 1) * num_width_points) + (x - 1);
        PointMass* upperLeftMass = &(point_masses[upperLeftIndex]);
        Spring* s3 = new Spring(mainMass, upperLeftMass, SHEARING);
        springs.push_back(*s3);
      }
      if (y != 0 && x != (num_width_points - 1)) {
        int upperRightIndex = ((y - 1) * num_width_points) + (x + 1);
        PointMass* upperRightMass = &(point_masses[upperRightIndex]);
        Spring* s4 = new Spring(mainMass, upperRightMass, SHEARING);
        springs.push_back(*s4);
      }

      // bending (2 to the right & 2 above)
      if (y >= 2) {
        int twoAboveIndex = ((y - 2) * num_width_points) + x;
        PointMass* twoAboveMass = &(point_masses[twoAboveIndex]);
        Spring* s5 = new Spring(mainMass, twoAboveMass, BENDING);
        springs.push_back(*s5);
      }
      if (x <= num_width_points - 3) {    // last index on a row = num_width_points - 1
        int twoRightIndex = (y * num_width_points) + (x + 2);
        PointMass* twoRightMass = &(point_masses[twoRightIndex]);
        Spring* s6 = new Spring(mainMass, twoRightMass, BENDING);
        springs.push_back(*s6);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D totalExtAccel = Vector3D();
  for (int accel = 0; accel < external_accelerations.size(); accel++) {
    totalExtAccel += external_accelerations[accel];
  }
  totalExtAccel *= mass;

  for (PointMass &pm : point_masses) {
      pm.forces = Vector3D();
      pm.forces += totalExtAccel;
  }

  for (Spring &s : springs) {
    e_spring_type typeSpring = s.spring_type;
    if (typeSpring == STRUCTURAL && cp->enable_structural_constraints) {
      double forceApplied = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      s.pm_a->forces += (s.pm_b->position - s.pm_a->position).unit() * forceApplied;
      s.pm_b->forces += (s.pm_a->position - s.pm_b->position).unit() * forceApplied;
    } else if (typeSpring == SHEARING && cp->enable_shearing_constraints) {
      double forceApplied = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      s.pm_a->forces += (s.pm_b->position - s.pm_a->position).unit() * forceApplied;
      s.pm_b->forces += (s.pm_a->position - s.pm_b->position).unit() * forceApplied;
    } else if (typeSpring == BENDING && cp->enable_bending_constraints) {
      double forceApplied = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
      s.pm_a->forces += (s.pm_b->position - s.pm_a->position).unit() * forceApplied;
      s.pm_b->forces += (s.pm_a->position - s.pm_b->position).unit() * forceApplied;
    }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  for (PointMass& pm : point_masses) {
      if (!pm.pinned) {
        Vector3D temp = pm.position;
        pm.position += (1.0 - (cp->damping / 100.0)) * (pm.position - pm.last_position) + ((pm.forces/mass) * (delta_t * delta_t));
        pm.last_position = temp;
      }
  }

  // TODO (Part 4): Handle self-collisions.
  // This won't do anything until you complete Part 4.
//  build_spatial_map();
//  for (PointMass &pm : point_masses) {
//    self_collide(pm, simulation_steps);
//  }


  // TODO (Part 3): Handle collisions with other primitives.
  // This won't do anything until you complete Part 3.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

  for (Spring &s : springs) {
    if (s.pm_a->pinned && s.pm_b->pinned) {
      continue;
    } else if (s.pm_a->pinned && !s.pm_b->pinned) {   // a pinned, b loose
      double springLength = (s.pm_a->position - s.pm_b->position).norm();
      if (springLength > s.rest_length * 1.1) {
        double diff = springLength - (s.rest_length * 1.1);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * diff;
      }
    } else if (!s.pm_a->pinned && s.pm_b->pinned) {   // a loose, b pinned
      double springLength = (s.pm_a->position - s.pm_b->position).norm();
      if (springLength > s.rest_length * 1.1) {
        double diff = springLength - (s.rest_length * 1.1);
        s.pm_a->position += (s.pm_b->position - s.pm_a->position).unit() * diff;
      }
    }
    else {
      double springLength = (s.pm_a->position - s.pm_b->position).norm();
      if (springLength > s.rest_length * 1.1) {
        double diff = springLength - (s.rest_length * 1.1);
        s.pm_a->position += (s.pm_b->position - s.pm_a->position).unit() * (diff/2.0);
        s.pm_b->position += (s.pm_a->position - s.pm_b->position).unit() * (diff/2.0);
      }
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
	delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &pm : point_masses) {
	  float hash_pos = hash_position(pm.position);
	  if (map[hash_pos] == NULL) {
        vector<PointMass *> * masses = new vector<PointMass *>();
        (*masses).push_back(&pm);
        map[hash_pos]=  masses;

	  } else {
		  vector<PointMass *> * masses = map[hash_pos];
          (*masses).push_back(&pm);
	  }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float hash_pos = hash_position(pm.position);
  vector<PointMass *> * masses = map[hash_pos];
  Vector3D total_corrections = Vector3D();
  int num_corrections = 0;
  for (PointMass* candidate : *masses) {
      if (candidate == &pm) {
          continue;
      }
      if ((candidate->position - pm.position).norm() < 2.0 * thickness) {
          double dist_diff = (2.0 * thickness) - ((candidate->position - pm.position).norm());
          Vector3D correction_vector = (pm.position - candidate->position).unit() * dist_diff;
          total_corrections += correction_vector;
          num_corrections ++;
      }
  }

  if (num_corrections > 0) {
    pm.position += (total_corrections / (double) num_corrections) / simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents
  // membership in some uniquely identified 3D box volume.
  double w = 3.0 * width / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, h);
  int x = (int) floor(pos.x / w);
  int y = (int) floor(pos.y / h);
  int z = (int) floor(pos.z / t);
  float hash_pos = (float) ((100000 * x) + (1000 * y) + z);
  return hash_pos;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm, pm + num_width_points, pm + 1));
      triangles.push_back(new Triangle(pm + 1, pm + num_width_points,
                                       pm + num_width_points + 1));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
