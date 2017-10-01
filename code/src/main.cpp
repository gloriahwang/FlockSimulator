#include <getopt.h>
#include <iostream>
#include <fstream>
#include <nanogui/nanogui.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unordered_set>

#include "CGL/CGL.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "cloth.h"
#include "clothSimulator.h"
#include "json.hpp"

typedef uint32_t gid_t;

using namespace std;
using namespace nanogui;

using json = nlohmann::json;

#define msg(s) cerr << "[ClothSim] " << s << endl;

const string SPHERE = "sphere";
const string PLANE = "plane";
const string CLOTH = "cloth";
const string OBSTACLES = "obstacles";
const string UNITS = "units";
const string INITIAL_UNIT_COUNT = "initial_unit_count";
const string PLAYGROUND = "playground";
const string PREDATORS = "predators";

const unordered_set<string> VALID_KEYS = {SPHERE, PLANE, CLOTH, OBSTACLES, UNITS, INITIAL_UNIT_COUNT, PLAYGROUND, PREDATORS};

ClothSimulator *app = nullptr;
GLFWwindow *window = nullptr;
Screen *screen = nullptr;

void error_callback(int error, const char* description) {
  puts(description);
}

void createGLContexts() {
  if (!glfwInit()) {
    return;
  }

  glfwSetTime(0);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  glfwWindowHint(GLFW_SAMPLES, 0);
  glfwWindowHint(GLFW_RED_BITS, 8);
  glfwWindowHint(GLFW_GREEN_BITS, 8);
  glfwWindowHint(GLFW_BLUE_BITS, 8);
  glfwWindowHint(GLFW_ALPHA_BITS, 8);
  glfwWindowHint(GLFW_STENCIL_BITS, 8);
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

  // Create a GLFWwindow object
  window = glfwCreateWindow(800, 800, "Flock Simulator", nullptr, nullptr);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    throw std::runtime_error("Could not initialize GLAD!");
  }
  glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

  glClearColor(0.2f, 0.25f, 0.3f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // Create a nanogui screen and pass the glfw pointer to initialize
  screen = new Screen();
  screen->initialize(window, true);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  glViewport(0, 0, width, height);
  glfwSwapInterval(1);
  glfwSwapBuffers(window);
}

void setGLFWCallbacks() {
  glfwSetCursorPosCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->cursorPosCallbackEvent(x, y)) {
      app->cursorPosCallbackEvent(x / screen->pixelRatio(),
                                  y / screen->pixelRatio());
    }
  });

  glfwSetMouseButtonCallback(
      window, [](GLFWwindow *, int button, int action, int modifiers) {
        if (!screen->mouseButtonCallbackEvent(button, action, modifiers) ||
            action == GLFW_RELEASE) {
          app->mouseButtonCallbackEvent(button, action, modifiers);
        }
      });

  glfwSetKeyCallback(
      window, [](GLFWwindow *, int key, int scancode, int action, int mods) {
        if (!screen->keyCallbackEvent(key, scancode, action, mods)) {
          app->keyCallbackEvent(key, scancode, action, mods);
        }
      });

  glfwSetCharCallback(window, [](GLFWwindow *, unsigned int codepoint) {
    screen->charCallbackEvent(codepoint);
  });

  glfwSetDropCallback(window,
                      [](GLFWwindow *, int count, const char **filenames) {
                        screen->dropCallbackEvent(count, filenames);
                        app->dropCallbackEvent(count, filenames);
                      });

  glfwSetScrollCallback(window, [](GLFWwindow *, double x, double y) {
    if (!screen->scrollCallbackEvent(x, y)) {
      app->scrollCallbackEvent(x, y);
    }
  });

  glfwSetFramebufferSizeCallback(window,
                                 [](GLFWwindow *, int width, int height) {
                                   screen->resizeCallbackEvent(width, height);
                                   app->resizeCallbackEvent(width, height);
                                 });
}

void usageError(const char *binaryName) {
  printf("Usage: %s [options]\n", binaryName);
  printf("Required program options:\n");
  printf("  -f     <STRING>    Filename of scene");
  printf("\n");
  exit(-1);
}

void incompleteObjectError(const char *object, const char *attribute) {
  cout << "Incomplete " << object << " definition, missing " << attribute << endl;
  exit(-1);
}

void loadObjectsFromFile(string filename, Cloth *cloth, ClothParameters *cp, vector<CollisionObject *>* objects, vector<Obstacle *> *obstacles,
                         vector<FlockUnit *> *units, int *initial_unit_count, Vector4D *playground_params, vector<Predator *> *predator_vector) {
  // Read JSON from file
  ifstream i(filename);
  json j;
  i >> j;

  bool initial_unit_count_updated = false;
  bool unit_positions_provided = false;

  // Loop over objects in scene
  for (json::iterator it = j.begin(); it != j.end(); ++it) {
    string key = it.key();

    // Check that object is valid
    unordered_set<string>::const_iterator query = VALID_KEYS.find(key);
    if (query == VALID_KEYS.end()) {
      cout << "Invalid scene object found: " << key << endl;
      exit(-1);
    }

    // Retrieve object
    json object = it.value();

    // Parse object depending on type (cloth, sphere, or plane)
    if (key == CLOTH) {
      // Cloth
      double width, height;
      int num_width_points, num_height_points;
      float thickness;
      e_orientation orientation;
      vector<vector<int>> pinned;

      auto it_width = object.find("width");
      if (it_width != object.end()) {
        width = *it_width;
      } else {
        incompleteObjectError("cloth", "width");
      }

      auto it_height = object.find("height");
      if (it_height != object.end()) {
        height = *it_height;
      } else {
        incompleteObjectError("cloth", "height");
      }

      auto it_num_width_points = object.find("num_width_points");
      if (it_num_width_points != object.end()) {
        num_width_points = *it_num_width_points;
      } else {
        incompleteObjectError("cloth", "num_width_points");
      }

      auto it_num_height_points = object.find("num_height_points");
      if (it_num_height_points != object.end()) {
        num_height_points = *it_num_height_points;
      } else {
        incompleteObjectError("cloth", "num_height_points");
      }

      auto it_thickness = object.find("thickness");
      if (it_thickness != object.end()) {
        thickness = *it_thickness;
      } else {
        incompleteObjectError("cloth", "thickness");
      }

      auto it_orientation = object.find("orientation");
      if (it_orientation != object.end()) {
        orientation = *it_orientation;
      } else {
        incompleteObjectError("cloth", "orientation");
      }

      auto it_pinned = object.find("pinned");
      if (it_pinned != object.end()) {
        vector<json> points = *it_pinned;
        for (auto pt : points) {
          vector<int> point = pt;
          pinned.push_back(point);
        }
      }

      cloth->width = width;
      cloth->height = height;
      cloth->num_width_points = num_width_points;
      cloth->num_height_points = num_height_points;
      cloth->thickness = thickness;
      cloth->orientation = orientation;
      cloth->pinned = pinned;

      // Cloth parameters
      bool enable_structural_constraints, enable_shearing_constraints, enable_bending_constraints;
      double damping, density, ks;

      auto it_enable_structural = object.find("enable_structural");
      if (it_enable_structural != object.end()) {
        enable_structural_constraints = *it_enable_structural;
      } else {
        incompleteObjectError("cloth", "enable_structural");
      }

      auto it_enable_shearing = object.find("enable_shearing");
      if (it_enable_shearing != object.end()) {
        enable_shearing_constraints = *it_enable_shearing;
      } else {
        incompleteObjectError("cloth", "it_enable_shearing");
      }

      auto it_enable_bending = object.find("enable_bending");
      if (it_enable_bending != object.end()) {
        enable_bending_constraints = *it_enable_bending;
      } else {
        incompleteObjectError("cloth", "it_enable_bending");
      }

      auto it_damping = object.find("damping");
      if (it_damping != object.end()) {
        damping = *it_damping;
      } else {
        incompleteObjectError("cloth", "damping");
      }

      auto it_density = object.find("density");
      if (it_density != object.end()) {
        density = *it_density;
      } else {
        incompleteObjectError("cloth", "density");
      }

      auto it_ks = object.find("ks");
      if (it_ks != object.end()) {
        ks = *it_ks;
      } else {
        incompleteObjectError("cloth", "ks");
      }

      cp->enable_structural_constraints = enable_structural_constraints;
      cp->enable_shearing_constraints = enable_shearing_constraints;
      cp->enable_bending_constraints = enable_bending_constraints;
      cp->density = density;
      cp->damping = damping;
      cp->ks = ks;
    } else if (key == SPHERE) {
      Vector3D origin;
      double radius, friction;

      auto it_origin = object.find("origin");
      if (it_origin != object.end()) {
        vector<double> vec_origin = *it_origin;
        origin = Vector3D(vec_origin[0], vec_origin[1], vec_origin[2]);
      } else {
        incompleteObjectError("sphere", "origin");
      }

      auto it_radius = object.find("radius");
      if (it_radius != object.end()) {
        radius = *it_radius;
      } else {
        incompleteObjectError("sphere", "radius");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("sphere", "friction");
      }

      Sphere *s = new Sphere(origin, radius, friction);
      objects->push_back(s);
    } else if (key == PLANE) { // PLANE
      Vector3D point, normal;
      double friction;

      auto it_point = object.find("point");
      if (it_point != object.end()) {
        vector<double> vec_point = *it_point;
        point = Vector3D(vec_point[0], vec_point[1], vec_point[2]);
      } else {
        incompleteObjectError("plane", "point");
      }

      auto it_normal = object.find("normal");
      if (it_normal != object.end()) {
        vector<double> vec_normal = *it_normal;
        normal = Vector3D(vec_normal[0], vec_normal[1], vec_normal[2]);
      } else {
        incompleteObjectError("plane", "normal");
      }

      auto it_friction = object.find("friction");
      if (it_friction != object.end()) {
        friction = *it_friction;
      } else {
        incompleteObjectError("plane", "friction");
      }

      Plane *p = new Plane(point, normal, friction);
      objects->push_back(p);
    } else if (key == OBSTACLES) {
      Vector3D position;
      double size, detection_radius;
      Obstacle::e_obstacle_type type;

      for (json json_obstacle : object) {

        auto it_size = json_obstacle.find("size");
        if (it_size != json_obstacle.end()) {
          size = *it_size;
        } else {
          incompleteObjectError("obstacle", "size");
        }

        auto it_detection_radius = json_obstacle.find("detection_radius");
        if (it_detection_radius != json_obstacle.end()) {
          detection_radius = *it_detection_radius;
        } else {
          incompleteObjectError("obstacle", "detection_radius");
        }

        auto it_type = json_obstacle.find("type");
        if (it_type != json_obstacle.end()) {
          type = *it_type;
        } else {
          incompleteObjectError("obstacle", "type");
        }

        auto it_position = json_obstacle.find("position");
        if (it_position != json_obstacle.end()) {
          vector<json> positions = *it_position;
          position = Vector3D(positions[0], positions[1], positions[2]);
        } else {
          incompleteObjectError("obstacle", "position");
        }

        Obstacle *o = new Obstacle(position, size, detection_radius, type);
        obstacles->push_back(o);
      }
    } else if (key == UNITS) {
      Vector3D position;
      Vector3D direction;
      nanogui::Color c;

      for (json json_unit : object) {
        auto it_position = json_unit.find("position");
        if (it_position != json_unit.end()) {
          vector<json> positions = *it_position;
          position = Vector3D(positions[0], positions[1], positions[2]);
        } else {
          incompleteObjectError("unit", "position");
        }

        auto it_color = json_unit.find("color");
        if (it_color != json_unit.end()) {
          vector<json> color = *it_color;
          c = nanogui::Color((float) color[0], (float) color[1], (float) color[2], (float) color[3]);
        } else {
          incompleteObjectError("unit", "color");
        }

        auto it_direction = json_unit.find("direction");
        if (it_direction != json_unit.end()) {
          vector<json> d = *it_direction;
          direction = Vector3D(d[0], d[1], d[2]);
        } else {
          incompleteObjectError("unit", "direction");
        }

        FlockUnit *u = new FlockUnit(position, direction, c);
        units->push_back(u);
      }
      unit_positions_provided = true;
    } else if (key == INITIAL_UNIT_COUNT) {
      *initial_unit_count = (int) object.get<int>();
      initial_unit_count_updated = true;
    } else if (key == PLAYGROUND) {

      auto it_min_x = object.find("min x");
      if (it_min_x != object.end()) {
        double min_x = *it_min_x;
        (*playground_params)[0] = min_x;
      } else {
        incompleteObjectError("playground", "min x");
      }

      auto it_min_y = object.find("min y");
      if (it_min_y != object.end()) {
        double min_y = *it_min_y;
        (*playground_params)[1] = min_y;
      } else {
        incompleteObjectError("playground", "min y");
      }

      auto it_max_x = object.find("max x");
      if (it_max_x != object.end()) {
        double max_x = *it_max_x;
        (*playground_params)[2] = max_x;
      } else {
        incompleteObjectError("playground", "max x");
      }

      auto it_max_y = object.find("max y");
      if (it_max_y != object.end()) {
        double max_y = *it_max_y;
        (*playground_params)[3] = max_y;
      } else {
        incompleteObjectError("playground", "max y");
      }
    } else if (key == PREDATORS) {
      for (json json_unit : object) {
        Vector3D position;
        Vector3D direction;
        nanogui::Color c;

        auto it_position = json_unit.find("position");
        if (it_position != json_unit.end()) {
          vector<json> positions = *it_position;
          position = Vector3D(positions[0], positions[1], positions[2]);
        } else {
          incompleteObjectError("unit", "position");
        }

        auto it_color = json_unit.find("color");
        if (it_color != json_unit.end()) {
          vector<json> color = *it_color;
          c = nanogui::Color((float) color[0], (float) color[1], (float) color[2], (float) color[3]);
        } else {
          incompleteObjectError("unit", "color");
        }

        auto it_direction = json_unit.find("direction");
        if (it_direction != json_unit.end()) {
          vector<json> d = *it_direction;
          direction = Vector3D(d[0], d[1], d[2]);
        } else {
          incompleteObjectError("unit", "direction");
        }

        Predator *p = new Predator(position, direction, c);
        predator_vector->push_back(p);
      }
    }
  }

  i.close();
  if (!initial_unit_count_updated && unit_positions_provided)
    *initial_unit_count = 0;
}

int main(int argc, char **argv) {
  Cloth cloth;
  ClothParameters cp;
  vector<CollisionObject *> objects;
  vector<FlockUnit *> * flock_vector = new vector<FlockUnit *>();
  vector<Predator *> * predator_vector = new vector<Predator *>();
  Flock flock(flock_vector, predator_vector);
  vector<Obstacle *> *obstacles = new vector<Obstacle *>();
  vector<Obstacle *> *food = new vector<Obstacle *>();
  int initial_unit_count = ClothSimulator::INITIAL_UNIT_COUNT_DEFAULT;
  Vector4D playground_params = Vector4D(ClothSimulator::PLAYGROUND_MIN_X_DEFAULT, ClothSimulator::PLAYGROUND_MIN_Y_DEFAULT, ClothSimulator::PLAYGROUND_MAX_X_DEFAULT, ClothSimulator::PLAYGROUND_MAX_Y_DEFAULT);

  if (argc == 1) { // No arguments, default initialization
    string default_file_name = "../scene/pinned2.json";
    loadObjectsFromFile(default_file_name, &cloth, &cp, &objects, obstacles, flock_vector, &initial_unit_count, &playground_params, predator_vector);
  } else {
    int c;

    while ((c = getopt (argc, argv, "f:")) != -1) {
      switch (c) {
        case 'f':
          loadObjectsFromFile(optarg, &cloth, &cp, &objects, obstacles, flock_vector, &initial_unit_count, &playground_params, predator_vector);
          break;
        default:
          usageError(argv[0]);
      }
    }
  }
  glfwSetErrorCallback(error_callback);

  createGLContexts();

  // Initialize the Cloth object
  cloth.buildGrid();
  cloth.buildClothMesh();

  // Initialize the ClothSimulator object
  app = new ClothSimulator(screen);
  app->loadCloth(&cloth);
  app->loadClothParameters(&cp);
  app->loadFlock(&flock);
  app->loadObstacles(obstacles);
  app->loadFood(food);
  app->loadCollisionObjects(&objects);
  app->loadInitialUnitCount(initial_unit_count);
  app->loadPlaygroundParams(playground_params);
  app->init();

  // Call this after all the widgets have been defined

  screen->setVisible(true);
  screen->performLayout();

  // Attach callbacks to the GLFW window

  setGLFWCallbacks();

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    app->drawContents();

    // Draw nanogui
    screen->drawContents();
    screen->drawWidgets();

    glfwSwapBuffers(window);

    if (!app->isAlive()) {
      glfwSetWindowShouldClose(window, 1);
    }
  }

  return 0;
}
