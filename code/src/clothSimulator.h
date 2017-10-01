#ifndef CGL_CLOTH_SIMULATOR_H
#define CGL_CLOTH_SIMULATOR_H

#include <nanogui/nanogui.h>
#include <png.h>

#include "camera.h"
#include "cloth.h"
#include "collision/collisionObject.h"
#include "flock.h"
#include "obstacle.h"
#include "predator.h"

using namespace nanogui;
class Flock;

class ClothSimulator {
public:
  ClothSimulator(Screen *screen);
  ~ClothSimulator();

  void init();

//  GLuint png_texture_load(const char * file_name, int * width, int * height);
  void loadCloth(Cloth *cloth);
  void loadClothParameters(ClothParameters *cp);
  void loadFlock(Flock *flock);
  void loadObstacles(vector<Obstacle *> *obstacles);
  void loadFood(vector<Obstacle *> *food);
  void loadCollisionObjects(vector<CollisionObject *> *objects);
  void loadInitialUnitCount(int i);
  void loadPlaygroundParams(Vector4D p);
  void delete_unit(double x, double y);
  virtual bool isAlive();
  virtual void drawContents();

  // Screen events

  virtual bool cursorPosCallbackEvent(double x, double y);
  virtual bool mouseButtonCallbackEvent(int button, int action, int modifiers);
  virtual bool keyCallbackEvent(int key, int scancode, int action, int mods);
  virtual bool dropCallbackEvent(int count, const char **filenames);
  virtual bool scrollCallbackEvent(double x, double y);
  virtual bool resizeCallbackEvent(int width, int height);

  static int INITIAL_UNIT_COUNT_DEFAULT;
  static double PLAYGROUND_MIN_X_DEFAULT;
  static double PLAYGROUND_MIN_Y_DEFAULT;
  static double PLAYGROUND_MAX_X_DEFAULT;
  static double PLAYGROUND_MAX_Y_DEFAULT;

  bool enable_tracking_mode = false;
  int unit_to_track_index;
  FlockUnit *unit_to_track;
  void select_unit_for_tracking(int r);

private:
  virtual void initGUI(Screen *screen);
  void drawFlock(GLShader &shader);
  void drawPredators(GLShader shader);
  void drawFlockingRegion(GLShader shader);
  void drawCursor(GLShader shader);
  void drawForceVectors(GLShader shader);
  void drawObstacles(GLShader shader);
  void drawFood(GLShader shader);

  // Camera methods

  virtual void resetCamera();
  virtual void tracking_camera();
  virtual Matrix4f getProjectionMatrix();
  virtual Matrix4f getViewMatrix();

  // Default simulation values

  int frames_per_sec = 10;
  int simulation_steps = 1;
  int steps_passed = 0;
  int GENERATE_FOOD_AFTER = 100;

  CGL::Vector3D gravity = CGL::Vector3D(0, -9.8, 0);
  nanogui::Color color = nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f);

  Cloth *cloth;
  ClothParameters *cp;
  Flock *flock;
  vector<Obstacle *> *obstacles;
  vector<Obstacle *> *food;
  vector<CollisionObject *> *collision_objects;

  // OpenGL attributes

  enum e_shader { WIREFRAME = 0, NORMALS = 1, PHONG = 2 };
  e_shader activeShader = WIREFRAME;

  vector<GLShader> shaders;

  GLShader wireframeShader;
  GLShader normalShader;
  GLShader phongShader;

  // Camera attributes

  CGL::Camera camera;
  CGL::Camera canonicalCamera;

  double view_distance;
  double canonical_view_distance;
  double min_view_distance;
  double max_view_distance;

  double scroll_rate;

  // Screen methods

  Screen *screen;
  void mouseLeftDragged(double x, double y);
  void mouseRightDragged(double x, double y);
  void mouseMoved(double x, double y);
  bool leftClickToGenerateEntity();
  void generate_food();

  // Mouse flags

  bool left_down = false;
  bool right_down = false;
  bool middle_down = false;
  bool single_button_press = false;

  // Keyboard flags

  bool ctrl_down = false;

  // Simulation flags

  bool is_paused = false;

  // Screen attributes

  int mouse_x;
  int mouse_y;

  int screen_w;
  int screen_h;

  bool is_alive = true;

  Vector2i default_window_size = Vector2i(1300, 800);

  vector<Vector3D> *playground;
  double playground_min_x;
  double playground_max_x;
  double playground_min_y;
  double playground_max_y;


  enum e_click_generator_type { UNIT = 0, DELETE = 1, CIRCLE = 2, SEEK = 3, AVOID = 4, STATIC_SEEK = 5, FOOD = 6, PREDATOR = 7};
  e_click_generator_type click_generator = UNIT;
  nanogui::Color unit_cursor = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);
  nanogui::Color obstacle_cursor = nanogui::Color(47.0f/255, 100.0f/255, 1.0f, 1.0f);
  nanogui::Color seek_cursor = nanogui::Color(202.0f/255, 0.0f/255, 42.0f/255, 1.0f);
  nanogui::Color avoid_cursor = nanogui::Color(254.0f/255, 226.0f/255, 62.0f/255, 1.0f);
  nanogui::Color hexagon_cursor = nanogui::Color(191.0f/255, 63.0f/255, 63.0f/255, 0.3f);
  nanogui::Color delete_cursor = nanogui::Color(157.0f/255, 251.0f/255, 172.0f/255, 1.0f);
  nanogui::Color food_cursor = nanogui::Color(145.0f/255, 85.0f/255, 77.0f/255, 1.0f);
  nanogui::Color predator_cursor = nanogui::Color(254.0f/255, 226.0f/255, 62.0f/255, 1.0f);
  nanogui::Color debug_alignment_color = nanogui::Color(1.0f, 0.0f, 0.0f, 1.0f);
  nanogui::Color debug_cohesion_color = nanogui::Color(0.0f, 1.0f, 0.0f, 1.0f);
  nanogui::Color debug_separation_color = nanogui::Color(0.0f, 0.0f, 1.0f, 1.0f);
  nanogui::Color debug_avoid_obstacle_color = avoid_cursor;
  nanogui::Color debug_noise_color = nanogui::Color(0.5f, 0.5f, 0.5f, 1.0f);
  nanogui::Color debug_seek_color = seek_cursor;
  nanogui::Color debug_avoid_predator_color = predator_cursor;
  nanogui::Color debug_cruise_color = nanogui::Color(1.0f, 1.0f, 1.0f, 1.0f);

  Vector2f getProjectedMousePosition();
  void scatter_flock_at_mouse_location();
  double scatter_radius = 70;

  // Mouse

  Vector2i NON_MOUSE_TRACKING_BUFFER = Vector2i(300, 45); // Padding: must be changed relative to default window size
  double GENERATION_GAP = 0.03; // gap from edge of playground to not generate units too close
  double CURSOR_MARKER_WIDTH = 10;
  double MOUSE_DRAW_Z_OFFSET = 1; // so that the mouse is always on top

  double OBSTACLE_SIZE = 10;
  double DETECTION_RADIUS = 10;

  int initial_unit_count = 0;
  int num_units_to_generate = 1;
  int MAX_UNITS_TO_GENERATE = 128;
  bool enable_random_food_generation = false;
  double zoom_end_location = 500;
  nanogui::Color tracked_unit_color = nanogui::Color(0.75f, 0.25f, 0.25f, 1.0f); // TODO: Need to find a better color for this
  double DEBUG_VECTOR_RENDER_WIDTH = 1.5;
  double DEBUG_VECTOR_RENDER_LENGTH = 100;

  bool DEBUG_ENABLED[8] = {true, true, true, false, false, false, false, false};
  enum e_debug_vector_type { ALIGNMENT = 0, COHESION = 1, SEPARATION = 2, AVOID_OBSTACLE = 3, NOISE = 4, SEEK_OBSTACLE = 5, AVOID_PREDATOR = 6, CRUISE = 7 };
  Window *window;
  Window *legend;
  Window *debug_vector_options;
  Window *tracking_options;

  // GUI and Legend

  string total_units_message_str;
  string total_obstacles_message_str;
  string total_food_message_str;
  string tracking_unit_message_str;
  string num_units_to_generate_message_str;
  string total_predators_message_str;
  Label *total_units_message;
  Label *total_obstacles_message;
  Label *total_food_message;
  Label *tracking_unit_message;
  Label *num_units_to_generate_message;
  Label *total_predators_message;
  void update_total_units_message();
  void update_total_obstacles_message();
  void update_total_food_message();
  void update_tracking_unit_message();
  void update_num_units_added_message();
  void update_total_predators_added_message();
};

#endif // CGL_CLOTH_SIM_H
