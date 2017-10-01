#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.

  Vector3D mass_to_origin_dir = (origin - pm.position).unit();
  double a = dot(mass_to_origin_dir, mass_to_origin_dir);
  double b = dot(2.0 * (pm.position - origin), mass_to_origin_dir);
  double c = dot(pm.position - origin, pm.position - origin) - radius2;

  double intersection = (b*b) - (4.0 * a * c);
  double t1 = (-b + sqrt(intersection)) / (2.0 * a);
  double t2 = (-b - sqrt(intersection)) / (2.0 * a);
  double t = min(t1, t2);

  if (t <= 0.0) {
    Vector3D tangentPt = pm.position + (t * mass_to_origin_dir);

    double dist_to_surface = (tangentPt - pm.last_position).norm();
    Vector3D unitDirection = (tangentPt - pm.last_position).unit();
    pm.position =  pm.last_position + ((1.0 - friction) * (unitDirection * dist_to_surface));
  }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  Misc::draw_sphere(shader, origin, radius * 0.92);
}
