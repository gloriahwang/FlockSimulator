#include <cmath>
#include <glad/glad.h>

#include <CGL/vector3D.h>
#include <nanogui/nanogui.h>

#include <iostream>
#include <fstream>

#include "clothSimulator.h"

#include "camera.h"
#include "cloth.h"
#include "flockUnit.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "misc/camera_info.h"

using namespace nanogui;
using namespace std;

int ClothSimulator::INITIAL_UNIT_COUNT_DEFAULT = 200;
double ClothSimulator::PLAYGROUND_MIN_X_DEFAULT = -400.0;
double ClothSimulator::PLAYGROUND_MIN_Y_DEFAULT = -400.0;
double ClothSimulator::PLAYGROUND_MAX_X_DEFAULT = 400.0;
double ClothSimulator::PLAYGROUND_MAX_Y_DEFAULT = 400.0;

ClothSimulator::ClothSimulator(Screen *screen) {
  this->screen = screen;

  // Initialize OpenGL buffers and shaders

  wireframeShader.initFromFiles("Wireframe", "../shaders/camera.vert",
                                "../shaders/wireframe.frag");
  normalShader.initFromFiles("Normal", "../shaders/camera.vert",
                             "../shaders/normal.frag");
  phongShader.initFromFiles("Phong", "../shaders/camera.vert",
                            "../shaders/phong.frag");

  shaders.push_back(wireframeShader);
  shaders.push_back(normalShader);
  shaders.push_back(phongShader);

  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_DEPTH_TEST);
}

ClothSimulator::~ClothSimulator() {
  for (auto shader : shaders) {
    shader.free();
  }

  if (cloth) delete cloth;
  if (cp) delete cp;
  if (collision_objects) delete collision_objects;
}

void ClothSimulator::loadCloth(Cloth *cloth) { this->cloth = cloth; }

void ClothSimulator::loadClothParameters(ClothParameters *cp) { this->cp = cp; }

void ClothSimulator::loadFlock(Flock *flock) { this->flock = flock; };

void ClothSimulator::loadObstacles(vector<Obstacle *> *obstacles) { this->obstacles = obstacles; }

void ClothSimulator::loadFood(vector<Obstacle *> *food) { this->food = food; }

void ClothSimulator::loadCollisionObjects(vector<CollisionObject *> *objects) { this->collision_objects = objects; }

void ClothSimulator::loadInitialUnitCount(int i) { initial_unit_count = i; }

void ClothSimulator::loadPlaygroundParams(Vector4D p) {
  playground_min_x = p[0];
  playground_min_y = p[1];
  playground_max_x = p[2];
  playground_max_y = p[3];
}

/**
 * Initializes the cloth simulation and spawns a new thread to separate
 * rendering from simulation.
 */
void ClothSimulator::init() {
  // Initialize GUI
  initGUI(screen);
  screen->setSize(default_window_size);

  const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
  int monitor_width = mode->width;
  int monitor_height = mode->height;

  // Center the window
  glfwSetWindowPos(screen->glfwWindow(), (monitor_width / 2) - (default_window_size[0] / 2), (monitor_height / 2) - (default_window_size[1] / 2));

  // Initialize camera

  CGL::Collada::CameraInfo camera_info;
  camera_info.hFov = 50;
  camera_info.vFov = 35;
  camera_info.nClip = 0.01;
  camera_info.fClip = 10000;

  // Try to intelligently figure out the camera target

  Vector3D avg_pm_position(0, 0, 0);

  for (auto &pm : cloth->point_masses) {
    avg_pm_position += pm.position / cloth->point_masses.size();
  }

  CGL::Vector3D target(avg_pm_position.x, avg_pm_position.y / 2,
                       avg_pm_position.z);
  CGL::Vector3D c_dir(0., 0., 0.);
  canonical_view_distance = max(playground_max_x - playground_min_x, playground_max_y - playground_min_y) * 0.9;
  scroll_rate = canonical_view_distance / 10;

  view_distance = canonical_view_distance * 2;
  min_view_distance = canonical_view_distance / 10.0;
  max_view_distance = canonical_view_distance * 20.0;

  // canonicalCamera is a copy used for view resets

  camera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z), view_distance,
               min_view_distance, max_view_distance);
  canonicalCamera.place(target, acos(c_dir.y), atan2(c_dir.x, c_dir.z),
                        view_distance, min_view_distance, max_view_distance);

  screen_w = default_window_size(0);
  screen_h = default_window_size(1);

  camera.configure(camera_info, screen_w, screen_h);
  canonicalCamera.configure(camera_info, screen_w, screen_h);

  // Initialize the playground
  playground = new vector<Vector3D>();
  playground->push_back(Vector3D(playground_min_x, playground_min_y, 0.0));
  playground->push_back(Vector3D(playground_max_x, playground_min_y, 0.0));
  playground->push_back(Vector3D(playground_max_x, playground_max_y, 0.0));
  playground->push_back(Vector3D(playground_min_x, playground_max_y, 0.0));

  // Initialize the bordering obstacles
//  double step_size = 2.0 * OBSTACLE_SIZE;
//  for (double pos = playground_min_x + OBSTACLE_SIZE; pos <= playground_max_x - OBSTACLE_SIZE; pos += step_size) {
//    Obstacle * o1 = new Obstacle(Vector3D(pos, playground_min_y + OBSTACLE_SIZE, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::AVOID_TYPE);
//    Obstacle * o2 = new Obstacle(Vector3D(pos, playground_max_y - OBSTACLE_SIZE, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::AVOID_TYPE);
//
//    obstacles->push_back(o1);
//    obstacles->push_back(o2);
//  }
//
//  for (double pos = playground_min_y + OBSTACLE_SIZE; pos < playground_max_y - OBSTACLE_SIZE; pos += step_size) {
//    Obstacle * o1 = new Obstacle(Vector3D(playground_min_x + OBSTACLE_SIZE, pos, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::AVOID_TYPE);
//    Obstacle * o2 = new Obstacle(Vector3D(playground_max_x - OBSTACLE_SIZE, pos, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::AVOID_TYPE);
//
//    obstacles->push_back(o1);
//    obstacles->push_back(o2);
//  }

  double width = playground_max_x - playground_min_x;
  double height = playground_max_y - playground_min_y;

  for (int i = 0; i < initial_unit_count; i++) {
    double rx = (rand() / (double) RAND_MAX) * (width / 2) + width / 4;
    double ry = (rand() / (double) RAND_MAX) * (height / 2) + height / 4;
    double rdx = (rand() / (double) RAND_MAX) * 2 - 1;
    double rdy = (rand() / (double) RAND_MAX) * 2 - 1;
    float r = (rand() / (float) RAND_MAX) * 2 - 1;
    float g = (rand() / (float) RAND_MAX) * 2 - 1;
    float b = (rand() / (float) RAND_MAX) * 2 - 1;
    nanogui::Color c = nanogui::Color(r, g, b, 1.0f);

    FlockUnit * u = new FlockUnit(Vector3D(rx + playground_min_x, ry + playground_min_y, 0), Vector3D(rdx, rdy, 0), c);
    flock->units->push_back(u);
  }
}

bool ClothSimulator::isAlive() { return is_alive; }

void ClothSimulator::drawContents() {
  glEnable(GL_DEPTH_TEST);

  if (!is_paused) {
    update_total_units_message();
    update_total_obstacles_message();
    update_total_food_message();
    update_tracking_unit_message();
    update_num_units_added_message();
    update_total_predators_added_message();

    for (int i = 0; i < simulation_steps; i++) {
      Vector2f mouse_position = getProjectedMousePosition();
      Vector3D converted_mouse_position = Vector3D(mouse_position[0], mouse_position[1], 0);
      if (enable_tracking_mode)
        converted_mouse_position = NULL;
      flock->simulate(frames_per_sec, simulation_steps, playground, obstacles, food, converted_mouse_position, flock->predators, this);

      if (enable_random_food_generation) {
        if (steps_passed == GENERATE_FOOD_AFTER) {
          generate_food();
          steps_passed = 0;
        } else {
          steps_passed++;
        }
      }

    }

    if (enable_tracking_mode) {
      tracking_camera();
    }

  }

  // Bind the active shader

  GLShader shader = shaders[activeShader];
  shader.bind();

  // Prepare the camera projection matrix

  Matrix4f model;
  model.setIdentity();

  Matrix4f view = getViewMatrix();
  Matrix4f projection = getProjectionMatrix();

  Matrix4f viewProjection = projection * view;

  shader.setUniform("model", model);
  shader.setUniform("viewProjection", viewProjection);

  if (!enable_tracking_mode)
    drawCursor(shader);
  drawObstacles(shader);
  drawFood(shader);
  drawForceVectors(shader);
  drawPredators(shader);
  drawFlock(shader);
  drawFlockingRegion(shader);

  for (CollisionObject *co : *collision_objects) {
    co->render(shader);
  }
}

void ClothSimulator::drawFlock(GLShader &shader) {
  int num_tris = (*(flock->units)).size();

  MatrixXf positions(3, 3);

  for (int i = 0; i < num_tris; i++) {
    FlockUnit * u = (*(flock->units))[i];

    Vector3D p1 = u->vertices[0];
    Vector3D p2 = u->vertices[1];
    Vector3D p3 = u->vertices[2];

    positions.col(0) << p1.x, p1.y, p1.z;
    positions.col(1) << p2.x, p2.y, p2.z;
    positions.col(2) << p3.x, p3.y, p3.z;

    if (enable_tracking_mode && u == unit_to_track)
      shader.setUniform("in_color", tracked_unit_color);
    else
      shader.setUniform("in_color", u->color);
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_TRIANGLES, 0, 3);
  }

}

void ClothSimulator::drawPredators(GLShader shader) {
  int num_tris = (*(flock->predators)).size();

  MatrixXf positions(3, 3);

  for (int i = 0; i < num_tris; i++) {
    Predator * u = (*(flock->predators))[i];

    Vector3D p1 = u->vertices[0];
    Vector3D p2 = u->vertices[1];
    Vector3D p3 = u->vertices[2];
    Vector3D p4 = u->vertices[3];
    Vector3D p5 = u->vertices[4];
    Vector3D p6 = u->vertices[5];

    positions.col(0) << p1.x, p1.y, p1.z;
    positions.col(1) << p2.x, p2.y, p2.z;
    positions.col(2) << p3.x, p3.y, p3.z;

    shader.setUniform("in_color", u->color);
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_TRIANGLES, 0, 3);

    positions.col(0) << p4.x, p4.y, p4.z;
    positions.col(1) << p5.x, p5.y, p5.z;
    positions.col(2) << p6.x, p6.y, p6.z;

    shader.setUniform("in_color", u->boost_color);
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_TRIANGLES, 0, 3);

  }
}

void ClothSimulator::drawFlockingRegion(GLShader shader) {
  MatrixXf positions(3, 8);

  for (int i = 0; i < playground->size(); i++) {
    int partner_i = (i < playground->size() - 1) ? i + 1 : 0;
    positions.col(i * 2) << (*playground)[i].x, (*playground)[i].y, (*playground)[i].z;
    positions.col(i * 2 + 1) << (*playground)[partner_i].x, (*playground)[partner_i].y, (*playground)[partner_i].z;
  }

  shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
  shader.uploadAttrib("in_position", positions);
  shader.drawArray(GL_LINES, 0, 8);
}

void ClothSimulator::drawObstacles(GLShader shader) {
  for (Obstacle *o : *obstacles) {
    if (o->obstacle_type == Obstacle::AVOID_TYPE) {
      int num_tris = 25;
      double theta = 2.0 * PI / num_tris;
      MatrixXf positions(3, num_tris * 3);

      Vector3D p = o->position;
      double s = o->size;

      double next_x, next_y;
      int col_count = 0;
      for (int j = 0; j < num_tris; j++) {
        positions.col(col_count) << p.x, p.y, MOUSE_DRAW_Z_OFFSET;
        next_x = p.x + (s * cos(j * theta));
        next_y = p.y + (s * sin(j * theta));
        col_count++;
        positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
        next_x = p.x + (s * cos((j + 1) * theta));
        next_y = p.y + (s * sin((j + 1) * theta));
        col_count++;
        positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
        col_count++;
      }

      shader.setUniform("in_color", obstacle_cursor);
      shader.uploadAttrib("in_position", positions);
      shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);
    } else if (o->obstacle_type == Obstacle::STATIC_SEEK_TYPE) {
      int num_tris = 6;
      double theta = 2.0 * PI / num_tris;
      MatrixXf positions(3, num_tris * 3);

      int col_count = 0;
      Vector3D p = o->position;
      double s = o->size;

      double next_x, next_y;
      for (int j = 0; j < num_tris; j++) {
        positions.col(col_count) << p.x, p.y, MOUSE_DRAW_Z_OFFSET;
        next_x = p.x + (s * cos(j * theta));
        next_y = p.y + (s * sin(j * theta));
        col_count++;
        positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
        next_x = p.x + (s * cos((j + 1) * theta));
        next_y = p.y + (s * sin((j + 1) * theta));
        col_count++;
        positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
        col_count++;
      }

      shader.setUniform("in_color", hexagon_cursor);
      shader.uploadAttrib("in_position", positions);
      shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);
    }
  }
}

void ClothSimulator::drawFood(GLShader shader) {
  for (Obstacle *o : *food) {
    int num_tris = 25;
    double theta = 2.0 * PI / num_tris;
    MatrixXf positions(3, num_tris * 3);

    Vector3D p = o->position;
    double s = o->size;

    double next_x, next_y;
    int col_count = 0;
    for (int j = 0; j < num_tris; j++) {
      positions.col(col_count) << p.x, p.y, MOUSE_DRAW_Z_OFFSET;
      next_x = p.x + (s * cos(j * theta));
      next_y = p.y + (s * sin(j * theta));
      col_count++;
      positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
      next_x = p.x + (s * cos((j + 1) * theta));
      next_y = p.y + (s * sin((j + 1) * theta));
      col_count++;
      positions.col(col_count) << next_x, next_y, MOUSE_DRAW_Z_OFFSET;
      col_count++;
    }

    shader.setUniform("in_color", food_cursor);
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_TRIANGLES, 0, num_tris * 3);
  }
}

void ClothSimulator::delete_unit(double x, double y) {

  double bound_min_x = max(playground_min_x, x - CURSOR_MARKER_WIDTH/2);
  double bound_max_x = min(playground_max_x, x + CURSOR_MARKER_WIDTH/2);
  double bound_min_y = max(playground_min_y, y - CURSOR_MARKER_WIDTH/2);
  double bound_max_y = min(playground_max_y, y + CURSOR_MARKER_WIDTH/2);

  int num_obstacles = obstacles->size();
  int j = 0;
  while (j < num_obstacles) {
    Obstacle *o = (*obstacles)[j];
    if (o->position.x >= bound_min_x && o->position.x <= bound_max_x && o->position.y >= bound_min_y && o->position.y <= bound_max_y) {
      delete o;
      obstacles->erase(std::remove(obstacles->begin(), obstacles->end(), o), obstacles->end());
      num_obstacles--;
    } else {
      j++;
    }
  }

  int num_food = food->size();
  int k = 0;
  while (k < num_food) {
    Obstacle *o = (*food)[k];
    if (o->position.x >= bound_min_x && o->position.x <= bound_max_x && o->position.y >= bound_min_y && o->position.y <= bound_max_y) {
      delete o;
      food->erase(std::remove(food->begin(), food->end(), o), food->end());
      num_food--;
    } else {
      k++;
    }
  }

  int flock_size = (*(flock->units)).size();
  int i = 0;
  while (i < flock_size) {
    FlockUnit *f = (*(flock->units))[i];
    if (f->position.x >= bound_min_x && f->position.x <= bound_max_x && f->position.y >= bound_min_y && f->position.y <= bound_max_y) {
      f->vertices.clear();
      delete f;
      (*(flock->units)).erase(std::remove((*(flock->units)).begin(), (*(flock->units)).end(), f), (*(flock->units)).end());
      flock_size--;
    } else {
      i++;
    }
  }

  int predator_size = (*(flock->predators)).size();
  int l = 0;
  while (l < predator_size) {
    Predator *p = (*(flock->predators))[l];
    if (p->position.x >= bound_min_x && p->position.x <= bound_max_x && p->position.y >= bound_min_y && p->position.y <= bound_max_y) {
      p->vertices.clear();
      delete p;
      (*(flock->predators)).erase(std::remove((*(flock->predators)).begin(), (*(flock->predators)).end(), p), (*(flock->predators)).end());
      predator_size--;
    } else {
      l++;
    }
  }
}

Vector2f ClothSimulator::getProjectedMousePosition() {
  Vector2i mouse_coord = screen->mousePos();
  double x_coord = (double) mouse_coord[0] - NON_MOUSE_TRACKING_BUFFER[0];
  double y_coord = (double) screen_h - mouse_coord[1] - NON_MOUSE_TRACKING_BUFFER[1];

  x_coord /= screen_w - 2 * NON_MOUSE_TRACKING_BUFFER[0];
  y_coord /= screen_h - 2 * NON_MOUSE_TRACKING_BUFFER[1];

  x_coord = (playground_max_x - playground_min_x) * x_coord + playground_min_x;
  y_coord = (playground_max_y - playground_min_y) * y_coord + playground_min_y;

  if (x_coord < playground_min_x)
    x_coord = playground_min_x;
  if (x_coord > playground_max_x)
    x_coord = playground_max_x;
  if (y_coord < playground_min_y)
    y_coord = playground_min_y;
  if (y_coord > playground_max_y)
    y_coord = playground_max_y;

  Vector2f result = Vector2f::Zero();
  result[0] = (float) x_coord;
  result[1] = (float) y_coord;
  return result;
}

void ClothSimulator::drawCursor(GLShader shader){
  MatrixXf positions(3, 6);

  Vector2f mouse_coord = getProjectedMousePosition();
  double x_coord = (double) mouse_coord[0];
  double y_coord = (double) mouse_coord[1];

  if (click_generator == UNIT) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", unit_cursor);
  } else if (click_generator == CIRCLE) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", obstacle_cursor);
  } else if (click_generator == SEEK) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", seek_cursor);
  } else if (click_generator == AVOID) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", avoid_cursor);
  } else if (click_generator == STATIC_SEEK) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", hexagon_cursor);
  } else if (click_generator == FOOD) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", food_cursor);
  } else if (click_generator == PREDATOR) {
    CURSOR_MARKER_WIDTH = 10;
    shader.setUniform("in_color", predator_cursor);
  } else if (click_generator == DELETE) {
    shader.setUniform("in_color", delete_cursor);
    vector<Vector3D> cursor_vertices;
    cursor_vertices.push_back(Vector3D((x_coord - CURSOR_MARKER_WIDTH/2), (y_coord - CURSOR_MARKER_WIDTH/2), MOUSE_DRAW_Z_OFFSET));
    cursor_vertices.push_back(Vector3D((x_coord + CURSOR_MARKER_WIDTH/2), (y_coord - CURSOR_MARKER_WIDTH/2), MOUSE_DRAW_Z_OFFSET));
    cursor_vertices.push_back(Vector3D((x_coord + CURSOR_MARKER_WIDTH/2), (y_coord + CURSOR_MARKER_WIDTH/2), MOUSE_DRAW_Z_OFFSET));
    cursor_vertices.push_back(Vector3D((x_coord - CURSOR_MARKER_WIDTH/2), (y_coord + CURSOR_MARKER_WIDTH/2), MOUSE_DRAW_Z_OFFSET));

    MatrixXf positions(3, 8);
    for (int i = 0; i < 4; i++) {
      int partner_i = (i < 3) ? i + 1 : 0;
      positions.col(i * 2) << cursor_vertices[i].x, cursor_vertices[i].y, cursor_vertices[i].z;
      positions.col(i * 2 + 1) << cursor_vertices[partner_i].x, cursor_vertices[partner_i].y, cursor_vertices[partner_i].z;
    }

    shader.setUniform("in_color", nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f));
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_LINES, 0, 8);
    return;
  }

  positions.col(0) << x_coord - CURSOR_MARKER_WIDTH / 2, y_coord - CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;
  positions.col(1) << x_coord - CURSOR_MARKER_WIDTH / 2, y_coord + CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;
  positions.col(2) << x_coord + CURSOR_MARKER_WIDTH / 2, y_coord - CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;

  positions.col(3) << x_coord + CURSOR_MARKER_WIDTH / 2, y_coord - CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;
  positions.col(4) << x_coord + CURSOR_MARKER_WIDTH / 2, y_coord + CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;
  positions.col(5) << x_coord - CURSOR_MARKER_WIDTH / 2, y_coord + CURSOR_MARKER_WIDTH / 2, MOUSE_DRAW_Z_OFFSET;



  shader.uploadAttrib("in_position", positions);
  shader.drawArray(GL_TRIANGLES, 0, 6);
}

void ClothSimulator::drawForceVectors(GLShader shader){
  if (!enable_tracking_mode || unit_to_track == NULL || (*(unit_to_track->force_debug)).size() == 0)
    return;

  MatrixXf positions(3, 6);

  Vector3D base_position = unit_to_track->position;
  vector<Vector3D> *force_debug = unit_to_track->force_debug;
  Vector3D dir;
  Vector3D ortho;
  Vector3D corner_1;
  Vector3D corner_2;
  Vector3D corner_3;
  Vector3D corner_4;

  for (int i = 0; i < force_debug->size(); i++) {
    if (!DEBUG_ENABLED[i])
      continue;

    Vector3D force = ((*force_debug)[i]);
    if (force.norm() > DEBUG_VECTOR_RENDER_LENGTH)
      force = force.unit() * DEBUG_VECTOR_RENDER_LENGTH;
    force += base_position;
    dir = ((*force_debug)[i]).unit();
    ortho = Vector3D(-dir.y, dir.x, 0);

    corner_1 = base_position + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
    corner_2 = base_position - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
    corner_3 = force + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
    corner_4 = force - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);

    positions.col(0) << corner_1.x, corner_1.y, MOUSE_DRAW_Z_OFFSET;
    positions.col(1) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
    positions.col(2) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;

    positions.col(3) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
    positions.col(4) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
    positions.col(5) << corner_4.x, corner_4.y, MOUSE_DRAW_Z_OFFSET;

    nanogui::Color c = nanogui::Color();

    switch ((e_debug_vector_type) i) {
      case e_debug_vector_type::ALIGNMENT:
        c = debug_alignment_color;
        break;
      case e_debug_vector_type::COHESION:
        c = debug_cohesion_color;
        break;
      case e_debug_vector_type::SEPARATION:
        c = debug_separation_color;
        break;
      case e_debug_vector_type::AVOID_OBSTACLE:
        c = debug_avoid_obstacle_color;
        break;
      case e_debug_vector_type::NOISE:
        c = debug_noise_color;
        break;
      case e_debug_vector_type::SEEK_OBSTACLE:
        c = debug_seek_color;
        break;
      case e_debug_vector_type::AVOID_PREDATOR:
        c = debug_avoid_predator_color;
        break;
      case e_debug_vector_type::CRUISE:
        c = debug_cruise_color;
        break;
    }

    shader.setUniform("in_color", c);
    shader.uploadAttrib("in_position", positions);
    shader.drawArray(GL_TRIANGLES, 0, 6);
  }

//  Vector3D alignment = ((*force_debug)[0]);
//  if (alignment.norm() != 0) {
//    if (alignment.norm() > DEBUG_VECTOR_RENDER_LENGTH)
//      alignment = alignment.unit() * DEBUG_VECTOR_RENDER_LENGTH;
//    alignment += base_position;
//    dir = ((*force_debug)[0]).unit();
//    ortho = Vector3D(-dir.y, dir.x, 0);
//
//    corner_1 = base_position + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_2 = base_position - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_3 = alignment + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_4 = alignment - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//
//    positions.col(0) << corner_1.x, corner_1.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(1) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(2) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//
//    positions.col(3) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(4) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(5) << corner_4.x, corner_4.y, MOUSE_DRAW_Z_OFFSET;
//
//    shader.setUniform("in_color", debug_alignment_color);
//    shader.uploadAttrib("in_position", positions);
//    shader.drawArray(GL_TRIANGLES, 0, 6);
//  }
//
//  Vector3D cohesion = ((*force_debug)[1]);
//  if (cohesion.norm() != 0) {
//    if (cohesion.norm() > DEBUG_VECTOR_RENDER_LENGTH)
//      cohesion = cohesion.unit() * DEBUG_VECTOR_RENDER_LENGTH;
//    cohesion += base_position;
//    dir = ((*force_debug)[1]).unit();
//    ortho = Vector3D(-dir.y, dir.x, 0);
//    corner_1 = base_position + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_2 = base_position - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_3 = cohesion + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_4 = cohesion - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//
//    positions.col(0) << corner_1.x, corner_1.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(1) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(2) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//
//    positions.col(3) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(4) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(5) << corner_4.x, corner_4.y, MOUSE_DRAW_Z_OFFSET;
//
//    shader.setUniform("in_color", debug_cohesion_color);
//    shader.uploadAttrib("in_position", positions);
//    shader.drawArray(GL_TRIANGLES, 0, 6);
//  }
//
//  Vector3D separation = ((*force_debug)[2]);
//  if (separation.norm() != 0) {
//    if (separation.norm() > DEBUG_VECTOR_RENDER_LENGTH)
//      separation = separation.unit() * DEBUG_VECTOR_RENDER_LENGTH;
//    separation += base_position;
//    dir = ((*force_debug)[2]).unit();
//    ortho = Vector3D(-dir.y, dir.x, 0);
//    corner_1 = base_position + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_2 = base_position - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_3 = separation + (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//    corner_4 = separation - (ortho * DEBUG_VECTOR_RENDER_WIDTH / 2);
//
//    positions.col(0) << corner_1.x, corner_1.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(1) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(2) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//
//    positions.col(3) << corner_2.x, corner_2.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(4) << corner_3.x, corner_3.y, MOUSE_DRAW_Z_OFFSET;
//    positions.col(5) << corner_4.x, corner_4.y, MOUSE_DRAW_Z_OFFSET;
//
//    shader.setUniform("in_color", debug_separation_color);
//    shader.uploadAttrib("in_position", positions);
//    shader.drawArray(GL_TRIANGLES, 0, 6);
//  }
}

// ----------------------------------------------------------------------------
// CAMERA CALCULATIONS
//
// OpenGL 3.1 deprecated the fixed pipeline, so we lose a lot of useful OpenGL
// functions that have to be recreated here.
// ----------------------------------------------------------------------------

void ClothSimulator::resetCamera() { camera.copy_placement(canonicalCamera); }

void ClothSimulator::tracking_camera() {
  if (camera.position().z > zoom_end_location)
    camera.move_forward(50);

  Vector3D position_to_track = unit_to_track->position;
  Vector3D difference = position_to_track - camera.position();
  camera.move_by(difference.x, difference.y, canonical_view_distance);

}

void ClothSimulator::select_unit_for_tracking(int r) {
  if (r == -1)
    r = (int) (rand() / (double) RAND_MAX * (*(flock->units)).size());
  unit_to_track_index = r;
  unit_to_track = (*(flock->units))[r];
}

Matrix4f ClothSimulator::getProjectionMatrix() {
  Matrix4f perspective;
  perspective.setZero();

  double near = camera.near_clip();
  double far = camera.far_clip();

  double theta = camera.v_fov() * M_PI / 360;
  double range = far - near;
  double invtan = 1. / tanf(theta);

  perspective(0, 0) = invtan / camera.aspect_ratio();
  perspective(1, 1) = invtan;
  perspective(2, 2) = -(near + far) / range;
  perspective(3, 2) = -1;
  perspective(2, 3) = -2 * near * far / range;
  perspective(3, 3) = 0;

  return perspective;
}

Matrix4f ClothSimulator::getViewMatrix() {
  Matrix4f lookAt;
  Matrix3f R;

  lookAt.setZero();

  // Convert CGL vectors to Eigen vectors
  // TODO: Find a better way to do this!

  CGL::Vector3D c_pos = camera.position();
  CGL::Vector3D c_udir = camera.up_dir();
  CGL::Vector3D c_target = camera.view_point();

  Vector3f eye(c_pos.x, c_pos.y, c_pos.z);
  Vector3f up(c_udir.x, c_udir.y, c_udir.z);
  Vector3f target(c_target.x, c_target.y, c_target.z);

  R.col(2) = (eye - target).normalized();
  R.col(0) = up.cross(R.col(2)).normalized();
  R.col(1) = R.col(2).cross(R.col(0));

  lookAt.topLeftCorner<3, 3>() = R.transpose();
  lookAt.topRightCorner<3, 1>() = -R.transpose() * eye;
  lookAt(3, 3) = 1.0f;

  return lookAt;
}

// ----------------------------------------------------------------------------
// EVENT HANDLING
// ----------------------------------------------------------------------------

bool ClothSimulator::cursorPosCallbackEvent(double x, double y) {
  if (left_down && !middle_down && !right_down) {
    if (ctrl_down) {
      mouseRightDragged(x, y);
    } else {
      single_button_press = false;
      mouseLeftDragged(x, y);
    }
  } else if (!left_down && !middle_down && right_down) {
    mouseRightDragged(x, y);
  } else if (!left_down && !middle_down && !right_down) {
    mouseMoved(x, y);
  }

  mouse_x = x;
  mouse_y = y;

  return true;
}

bool ClothSimulator::mouseButtonCallbackEvent(int button, int action,
                                              int modifiers) {
  switch (action) {
    case GLFW_PRESS:
      switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
          left_down = true;
          single_button_press = true;
          break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
          middle_down = true;
          break;
        case GLFW_MOUSE_BUTTON_RIGHT:
          right_down = true;
          break;
      }
      return true;

    case GLFW_RELEASE:
      switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
          left_down = false;
          if (single_button_press)
            leftClickToGenerateEntity();
          break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
          middle_down = false;
          break;
        case GLFW_MOUSE_BUTTON_RIGHT:
          right_down = false;
          break;
      }
      return true;
  }

  return false;
}

bool ClothSimulator::leftClickToGenerateEntity() {
  if (enable_tracking_mode)
    return false;

  Vector2f mouse_coord = getProjectedMousePosition();
  single_button_press = false;
  double x_coord = (double) mouse_coord[0];
  double y_coord = (double) mouse_coord[1];

  if (!(x_coord > playground_min_x + OBSTACLE_SIZE / 2 &&
        x_coord < playground_max_x - OBSTACLE_SIZE / 2 &&
        y_coord > playground_min_y + OBSTACLE_SIZE / 2  &&
        y_coord < playground_max_y - OBSTACLE_SIZE / 2)) {
    return false;
  }


  if (click_generator == UNIT) {
    for (int i = 0; i < num_units_to_generate; i++) {
      float r = (rand() / (float) RAND_MAX) * 2 - 1;
      float g = (rand() / (float) RAND_MAX) * 2 - 1;
      float b = (rand() / (float) RAND_MAX) * 2 - 1;
      nanogui::Color c = nanogui::Color(r, g, b, 1.0f);
      FlockUnit* unit = new FlockUnit(Vector3D(x_coord, y_coord, 0), Vector3D(0, 0, 0), c);
      (*(flock->units)).push_back(unit);
    }

  } else if (click_generator == CIRCLE) {
    Obstacle *o = new Obstacle(Vector3D(x_coord, y_coord, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::AVOID_TYPE);
    obstacles->push_back(o);

  } else if (click_generator == STATIC_SEEK) {
    Obstacle *o = new Obstacle(Vector3D(x_coord, y_coord, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::STATIC_SEEK_TYPE);
    obstacles->push_back(o);

  } else if (click_generator == DELETE) {
    delete_unit(x_coord, y_coord);

  } else if (click_generator == FOOD) {
    Obstacle *o = new Obstacle(Vector3D(x_coord, y_coord, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::FOOD_TYPE);
    food->push_back(o);

  } else if (click_generator == PREDATOR) {
    Predator* predator = new Predator(Vector3D(x_coord, y_coord, 0), Vector3D(0, 0, 0), predator_cursor);
    (*(flock->predators)).push_back(predator);

  } else {
    return false;
  }

  return true;
}

void ClothSimulator::generate_food() {
  double rx = (rand() / (float) RAND_MAX) * (playground_max_x - playground_min_x) + playground_min_x;
  double ry = (rand() / (float) RAND_MAX) * (playground_max_y - playground_min_y) + playground_min_y;
  Obstacle *f = new Obstacle(Vector3D(rx, ry, 0), OBSTACLE_SIZE, DETECTION_RADIUS, Obstacle::FOOD_TYPE);
  food->push_back(f);
}

void ClothSimulator::scatter_flock_at_mouse_location() {
  Vector2f mouse_location = getProjectedMousePosition();
  Vector3D converted_mouse_location = Vector3D(mouse_location[0], mouse_location[1], 0);
  for (FlockUnit* u : (*(flock->units))) {
    double distance = (u->position - converted_mouse_location).norm();
    if (distance < scatter_radius) {
      Vector3D scatter_force = (u->position - converted_mouse_location).unit();
      u->scatter_force = scatter_force;
    }
  }
}

void ClothSimulator::mouseMoved(double x, double y) {
  y = screen_h - y;

  Vector2i mouse_coord = screen->mousePos();

  double x_coord = (double) mouse_coord[0];
  double y_coord = (double) screen_h - mouse_coord[1];

  bool out_of_bounds_x = screen_w - x_coord < NON_MOUSE_TRACKING_BUFFER[0] || x_coord < NON_MOUSE_TRACKING_BUFFER[0];
  bool out_of_bounds_y = screen_h - y_coord < NON_MOUSE_TRACKING_BUFFER[1] || y_coord < NON_MOUSE_TRACKING_BUFFER[1];

  if (out_of_bounds_x || out_of_bounds_y) {
    glfwSetInputMode(screen->glfwWindow(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  } else {
    glfwSetInputMode(screen->glfwWindow(), GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
  }
}

void ClothSimulator::mouseLeftDragged(double x, double y) {
  float dx = x - mouse_x;
  float dy = y - mouse_y;

//  camera.rotate_by(-dy * (PI / screen_h), -dx * (PI / screen_w));
}

void ClothSimulator::mouseRightDragged(double x, double y) {
//  camera.move_by(mouse_x - x, y - mouse_y, canonical_view_distance);
}

bool ClothSimulator::keyCallbackEvent(int key, int scancode, int action,
                                      int mods) {
  bool save = false;
  ctrl_down = (bool)(mods & GLFW_MOD_CONTROL);

  if (action == GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_ESCAPE:
        is_alive = false;
        break;
      case 'p':
      case 'P':
        is_paused = !is_paused;
        break;
      case 'n':
      case 'N':
        if (is_paused) {
          is_paused = false;
          drawContents();
          is_paused = true;
        }
        break;
      case '1':
        click_generator = e_click_generator_type::UNIT;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '2':
        click_generator = e_click_generator_type::CIRCLE;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '3':
        click_generator = e_click_generator_type::STATIC_SEEK;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '4':
        click_generator = e_click_generator_type::DELETE;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '5':
        click_generator = e_click_generator_type::SEEK;
        FlockUnit::seek_mouse_enabled = true;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '6':
        click_generator = e_click_generator_type::AVOID;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = true;
        break;
      case '7':
        click_generator = e_click_generator_type::FOOD;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case '8':
        click_generator = e_click_generator_type::PREDATOR;
        FlockUnit::seek_mouse_enabled = false;
        FlockUnit::avoid_mouse_enabled = false;
        break;
      case 't':
      case 'T':
        enable_tracking_mode = !enable_tracking_mode;
        if (!enable_tracking_mode) {
          resetCamera();
          debug_vector_options->setVisible(false);
          tracking_options->setVisible(false);
          window->setVisible(true);
          legend->setVisible(true);
        } else {
          if ((*(flock->units)).size() == 0)
            enable_tracking_mode = false;

          select_unit_for_tracking(-1);
          debug_vector_options->setVisible(true);
          tracking_options->setVisible(true);
          window->setVisible(false);
          legend->setVisible(false);
        }
        break;
      case 'c':
      case 'C':
        save = true;
        break;
      case GLFW_KEY_LEFT:
        if (enable_tracking_mode) {
          unit_to_track_index = (unit_to_track_index == 0) ? (int) (*(flock->units)).size() - 1 : unit_to_track_index - 1;
          select_unit_for_tracking(unit_to_track_index);
        }
        break;
      case GLFW_KEY_RIGHT:
        if (enable_tracking_mode) {
          unit_to_track_index = (unit_to_track_index == (*(flock->units)).size() - 1) ? 0 : unit_to_track_index + 1;
          select_unit_for_tracking(unit_to_track_index);
        }
        break;
      case 's':
      case 'S':
        scatter_flock_at_mouse_location();
        break;
      case GLFW_KEY_UP:
        if (!enable_tracking_mode)
          num_units_to_generate  = min(MAX_UNITS_TO_GENERATE, num_units_to_generate * 2);
        break;
      case GLFW_KEY_DOWN:
        if (!enable_tracking_mode)
          num_units_to_generate = max(1, num_units_to_generate / 2);
        break;
    }
  }

  if (save) {

    int count = 0;
    string name = "../scene/out";
    string ext = ".json";

    while (std::ifstream(name + std::to_string(count) + ext)) {
      count++;
    }

    cout << "Writing to " << name + std::to_string(count) + ext << "\n";
    ofstream myfile;
    myfile.open(name + std::to_string(count) + ext);
    if (myfile.is_open()) {
      myfile << "{ \n";

      myfile << "\"playground\": {\n";
      myfile << "\"min x\": " << playground_min_x << ",\n";
      myfile << "\"min y\": " << playground_min_y << ",\n";
      myfile << "\"max x\": " << playground_max_x << ",\n";
      myfile << "\"max y\": " << playground_max_y << "\n";
      if (obstacles->size() > 0 || flock->units->size() > 0 || flock->predators->size() > 0)
        myfile << "},\n";
      else
        myfile << "}\n";

      if (obstacles->size() > 0) {
        myfile << "\"obstacles\": [\n";
        for (int i = obstacles->size()-1; i >= 0; i--) {
          Obstacle *o = (*obstacles)[i];
          myfile << "{\n";
          myfile << " \"size\": " + to_string(o->size) + ",\n";
          myfile << " \"detection_radius\": " + to_string(o->detection_radius) + ",\n";
          myfile << " \"position\": [" + to_string(o->position.x) + ", " + to_string(o->position.y) + ", " + to_string(o->position.z) + "],\n";
          myfile << " \"type\": " + to_string(o->obstacle_type) + "\n";
          if (i > 0)
            myfile << "},\n";
          else {
            myfile << "}\n";
            if ((*(flock->units)).size() > 0 || flock->predators->size() > 0)
              myfile << "],\n";
            else
              myfile << "]\n";
          }
        }
      }

      if ((*(flock->units)).size() > 0) {
        myfile << "\"units\": [\n";
        for (int j = (flock->units->size()) - 1; j >= 0; j--) {
          FlockUnit *f = (*(flock->units))[j];
          myfile << "{\n";
          myfile << " \"position\": [" + to_string(f->position.x) + ", " + to_string(f->position.y) + ", " + to_string(f->position.z) + "],\n";
          myfile << " \"color\": [" + to_string(f->color[0]) + ", " + to_string(f->color[1]) + ", "
                    + to_string(f->color[2]) + ", " + to_string(f->color[3]) + "],\n";
          myfile << " \"direction\": [" + to_string(f->direction.x) + ", " + to_string(f->direction.y) + ", " + to_string(f->direction.z) + "]\n";
          if (j > 0)
            myfile << "},\n";
          else {
            myfile << "}\n";
            if (flock->predators->size() > 0)
              myfile << "],\n";
            else
              myfile << "]\n";
          }
        }
      }

      if (flock->predators->size() > 0) {
        myfile << "\"predators\": [\n";
        for (int k = (flock->predators->size()) - 1; k >= 0; k--) {
          Predator *p = (*(flock->predators))[k];
          myfile << "{\n";
          myfile << " \"position\": [" + to_string(p->position.x) + ", " + to_string(p->position.y) + ", " + to_string(p->position.z) + "],\n";
          myfile << " \"color\": [" + to_string(p->color[0]) + ", " + to_string(p->color[1]) + ", "
                    + to_string(p->color[2]) + ", " + to_string(p->color[3]) + "],\n";
          myfile << " \"direction\": [" + to_string(p->direction.x) + ", " + to_string(p->direction.y) + ", " + to_string(p->direction.z) + "]\n";
          if (k > 0)
            myfile << "},\n";
          else {
            myfile << "}\n";
            myfile << "]\n";
          }
        }
      }

      myfile << "}\n";
      myfile.close();
      cout << "Finished writing to " << name + std::to_string(count) + ext << "\n";
    }
    else cout << "Unable to open file";
  }

  return true;
}

bool ClothSimulator::dropCallbackEvent(int count, const char **filenames) {
  return true;
}

bool ClothSimulator::scrollCallbackEvent(double x, double y) {
  if (ctrl_down) {
    CURSOR_MARKER_WIDTH += 10.0 * -y;
    if (CURSOR_MARKER_WIDTH < 0) {
      CURSOR_MARKER_WIDTH = 0;
    }
    return true;
  }
  return false;
}

bool ClothSimulator::resizeCallbackEvent(int width, int height) {
  screen_w = width;
  screen_h = height;

  camera.set_screen_size(screen_w, screen_h);
  return true;
}

void ClothSimulator::initGUI(Screen *screen) {
  window = new Window(screen, "Settings");
  window->setPosition(Vector2i(15, 15));
  window->setLayout(new GroupLayout(15, 6, 14, 5));

  //Options for how units flock

  total_units_message = new Label(window, ".", "sans-bold");
  total_obstacles_message = new Label(window, ".", "sans-bold");
  total_food_message = new Label(window, ".", "sans-bold");
  total_predators_message = new Label(window, ".", "sans-bold");
  num_units_to_generate_message = new Label(window, ".", "sans-bold");

  new Label(window, "Unit Flocking Options", "sans-bold");

  {
    Button *b = new Button(window, "Alignment");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { FlockUnit::alignment_enabled = state; });

    b = new Button(window, "Cohesion");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { FlockUnit::cohesion_enabled = state; });

    b = new Button(window, "Separation");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { FlockUnit::separation_enabled = state; });

    b = new Button(window, "Noise");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { FlockUnit::noise_enabled = state; });
  }

  new Label(window, "Food Chain Options", "sans-bold");
  {
    Button *b = new Button(window, "Random Food Generation");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { enable_random_food_generation = state; });
  }


  // Mass-spring parameters

//  new Label(window, "Parameters", "sans-bold");
//
//  {
//    Widget *panel = new Widget(window);
//    GridLayout *layout =
//            new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
//    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
//    layout->setSpacing(0, 10);
//    panel->setLayout(layout);
//
//    new Label(panel, "density :", "sans-bold");
//
//    FloatBox<double> *fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(cp->density / 10);
//    fb->setUnits("g/cm^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { cp->density = (double)(value * 10); });
//
//    new Label(panel, "ks :", "sans-bold");
//
//    fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(cp->ks);
//    fb->setUnits("N/m");
//    fb->setSpinnable(true);
//    fb->setMinValue(0);
//    fb->setCallback([this](float value) { cp->ks = value; });
//  }

  // Simulation constants

//  new Label(window, "Simulation", "sans-bold");
//
//  {
//    Widget *panel = new Widget(window);
//    GridLayout *layout =
//            new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
//    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
//    layout->setSpacing(0, 10);
//    panel->setLayout(layout);
//
//    new Label(panel, "frames/s :", "sans-bold");
//
//    IntBox<int> *fsec = new IntBox<int>(panel);
//    fsec->setEditable(true);
//    fsec->setFixedSize(Vector2i(100, 20));
//    fsec->setFontSize(14);
//    fsec->setValue(frames_per_sec);
//    fsec->setSpinnable(true);
//    fsec->setCallback([this](int value) { frames_per_sec = value; });
//
//    new Label(panel, "steps/frame :", "sans-bold");
//
//    IntBox<int> *num_steps = new IntBox<int>(panel);
//    num_steps->setEditable(true);
//    num_steps->setFixedSize(Vector2i(100, 20));
//    num_steps->setFontSize(14);
//    num_steps->setValue(simulation_steps);
//    num_steps->setSpinnable(true);
//    num_steps->setMinValue(0);
//    num_steps->setCallback([this](int value) { simulation_steps = value; });
//  }

  // Damping slider and textbox

//  new Label(window, "Damping", "sans-bold");
//
//  {
//    Widget *panel = new Widget(window);
//    panel->setLayout(
//            new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
//
//    Slider *slider = new Slider(panel);
//    slider->setValue(cp->damping);
//    slider->setFixedWidth(105);
//
//    TextBox *percentage = new TextBox(panel);
//    percentage->setFixedWidth(75);
//    percentage->setValue(to_string(cp->damping));
//    percentage->setUnits("%");
//    percentage->setFontSize(14);
//
//    slider->setCallback([percentage](float value) {
//        percentage->setValue(std::to_string(value));
//    });
//    slider->setFinalCallback([&](float value) {
//        cp->damping = (double)value;
//        // cout << "Final slider value: " << (int)(value * 100) << endl;
//    });
//  }

  // Gravity

//  new Label(window, "Gravity", "sans-bold");
//
//  {
//    Widget *panel = new Widget(window);
//    GridLayout *layout =
//            new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 5, 5);
//    layout->setColAlignment({Alignment::Maximum, Alignment::Fill});
//    layout->setSpacing(0, 10);
//    panel->setLayout(layout);
//
//    new Label(panel, "x :", "sans-bold");
//
//    FloatBox<double> *fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.x);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.x = value; });
//
//    new Label(panel, "y :", "sans-bold");
//
//    fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.y);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.y = value; });
//
//    new Label(panel, "z :", "sans-bold");
//
//    fb = new FloatBox<double>(panel);
//    fb->setEditable(true);
//    fb->setFixedSize(Vector2i(100, 20));
//    fb->setFontSize(14);
//    fb->setValue(gravity.z);
//    fb->setUnits("m/s^2");
//    fb->setSpinnable(true);
//    fb->setCallback([this](float value) { gravity.z = value; });
//  }

  // Appearance

//  new Label(window, "Appearance", "sans-bold");
//
//  {
//    ComboBox *cb = new ComboBox(window, {"Wireframe", "Normals", "Shaded"});
//    cb->setFontSize(14);
//    cb->setCallback(
//            [this, screen](int idx) { activeShader = static_cast<e_shader>(idx); });
//  }

//  new Label(window, "Color", "sans-bold");
//
//  {
//    ColorWheel *cw = new ColorWheel(window, color);
//    cw->setCallback(
//            [this](const nanogui::Color &color) { this->color = color; });
//  }

  legend = new Window(screen, "Tools");
  legend->setFixedWidth(190);
  legend->setPosition(Vector2i(default_window_size[0] - legend->fixedWidth() - 15, 15));
  legend->setLayout(new GroupLayout(15, 6, 14, 5));

  new Label(legend, "1: Unit", "sans-bold");
  new Label(legend, "2: Avoidance Obstacle", "sans-bold");
  new Label(legend, "3: Attraction Obstacle", "sans-bold");
  new Label(legend, "4: Delete", "sans-bold");
  new Label(legend, "    [Ctrl + Scroll] Change Size", "sans-bold");
  new Label(legend, "5: Follow Mouse", "sans-bold");
  new Label(legend, "6: Avoid Mouse", "sans-bold");
  new Label(legend, "7: Food", "sans-bold");
  new Label(legend, "8: Predator", "sans-bold");
  new Label(legend, "S: Scatter", "sans-bold");
  new Label(legend, "T: Track Random Unit", "sans-bold");

  debug_vector_options = new Window(screen, "Debug Vectors");
  debug_vector_options->setFixedHeight(60);
  debug_vector_options->setFixedWidth(635);
  debug_vector_options->setPosition(Vector2i(default_window_size[0] / 2 - debug_vector_options->fixedWidth() / 2,
                                             default_window_size[1] - debug_vector_options->fixedHeight() - 15));
  debug_vector_options->setLayout(new GridLayout(Orientation::Vertical, 1, Alignment::Middle, 6, 6));
  debug_vector_options->setVisible(false);

  {
    Button *b = new Button(debug_vector_options, "Alignment");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::ALIGNMENT] = state; });

    b = new Button(debug_vector_options, "Cohesion");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::COHESION] = state; });

    b = new Button(debug_vector_options, "Separation");
    b->setFlags(Button::ToggleButton);
    b->setPushed(true);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::SEPARATION] = state; });

    b = new Button(debug_vector_options, "Avoid Obstacles");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::AVOID_OBSTACLE] = state; });

    b = new Button(debug_vector_options, "Avoid Predators");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::AVOID_PREDATOR] = state; });

    b = new Button(debug_vector_options, "Noise");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::NOISE] = state; });

    b = new Button(debug_vector_options, "Seek Obstacle");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::SEEK_OBSTACLE] = state; });

    b = new Button(debug_vector_options, "Cruise");
    b->setFlags(Button::ToggleButton);
    b->setPushed(false);
    b->setFontSize(14);
    b->setChangeCallback(
            [this](bool state) { DEBUG_ENABLED[(int) e_debug_vector_type::CRUISE] = state; });
  }

  tracking_options = new Window(screen, "Tracking");
  tracking_options->setFixedWidth(140);
  tracking_options->setPosition(Vector2i(default_window_size[0] - tracking_options->fixedWidth() - 15, 15));
  tracking_options->setLayout(new GroupLayout(15, 6, 14, 5));
  tracking_options->setVisible(false);

  tracking_unit_message = new Label(tracking_options, ".", "sans-bold");
  new Label(tracking_options, "<- Previous Unit", "sans-bold");
  new Label(tracking_options, "-> Next Unit", "sans-bold");
}

void ClothSimulator::update_total_units_message() {
  total_units_message_str = "Total Units : ";
  total_units_message_str += std::to_string((*(flock->units)).size());
  total_units_message->setCaption(total_units_message_str);
}

void ClothSimulator::update_total_obstacles_message() {
  total_obstacles_message_str = "Total Obstacles : ";
  total_obstacles_message_str += std::to_string(obstacles->size());
  total_obstacles_message->setCaption(total_obstacles_message_str);
}

void ClothSimulator::update_total_food_message() {
  total_food_message_str = "Total Food : ";
  total_food_message_str += std::to_string(food->size());;
  total_food_message->setCaption(total_food_message_str);
}

void ClothSimulator::update_tracking_unit_message() {
  tracking_unit_message_str = "Tracking Index : ";
  if (enable_tracking_mode)
    tracking_unit_message_str += std::to_string(unit_to_track_index);
  else
    tracking_unit_message_str += "N/A";
  tracking_unit_message->setCaption(tracking_unit_message_str);
}

void ClothSimulator::update_num_units_added_message() {
  num_units_to_generate_message_str = "Add units : ";
  num_units_to_generate_message_str += std::to_string(num_units_to_generate);
  num_units_to_generate_message->setCaption(num_units_to_generate_message_str);
}

void ClothSimulator::update_total_predators_added_message() {
  total_predators_message_str = "Total Predators : ";
  total_predators_message_str += std::to_string(flock->predators->size());
  total_predators_message->setCaption(total_predators_message_str);
}

