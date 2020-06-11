#include "geom.hpp"


/*
[x] Vec3        Prism
[x] LineSegment Sphere
[x] Sphere      Sphere
[x] Sphere      Cylinder
[x] Cylinder    Sphere

[x] Prism Vec3
[x] Prism LineSegment
[x] Prism Prism
[x] Prism Sphere
[x] Prism Cylinder
*/


using namespace std;


/* Non-prism intersections */


bool intersect(Sphere x, Cylinder y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // first project sphere into cylinder coordinates
  auto center = rotate_point(x.center, y.center, inverse(y.orientation)) - y.center;
  auto xydist = sqrt((center.x*center.x) + (center.y*center.y));
  // now that we're in the frame of the cylinder, if they're close enough on the x-y plane then we check for collisions
  if (xydist + x.r <= y.r) {
    if (fabs(center.z) <= y.e) {
      // trivial collision
      DEBUG_COUT("Trivial collision.");
      DEBUG_LEAVE;
      return true;
    }
    else if (fabs(center.z) > (y.e + x.r)) {
      // trivial no collision
      DEBUG_COUT("Trivial no collision, sphere too distant along z.");
      DEBUG_LEAVE;
      return false;
    }
    // The complicated check: the sphere's center is above the top (or below the bottom)
    // of the cylinder but not far enough that we know there's no collision.
    // There will be a collision if (sphere's xy distance) - (cylinder radius) is leq  
    // the cosine of the angle between the centre of the sphere and the closest point on the
    // circle on top/bottom of the cylinder.
    const double th = asin((fabs(center.z) - y.e) / x.r);
    const bool col = fabs(xydist + x.r*cos(th)) <= y.r;
    if (col) {
      DEBUG_COUT("Did full check, collision found.");
    } else {
      DEBUG_COUT("Did full check, collision not found.");
    }
    DEBUG_LEAVE;
    return col;
  }
  else {
    // if the're not close enough on the xy-plane there can't be an intersection
    DEBUG_COUT("Trivial no collision: x-y plane separation too large.");
    DEBUG_LEAVE;
    return false;
  }
}


bool intersect(LineSegment x, Sphere y) {
  // DEBUG_ENTER(__PRETTY_FUNCTION__);

  Vec3
    v = x.b - x.a,
    w = y.center - x.a;

  double
    d0 = w * v,
    d1 = v * v;

  if (d0 <= 0) {  // before ls.a
    if (norm(x.a - y.center) < y.r) {
      // DEBUG_COUT("Found collision, line segment a is within sphere.");
      // DEBUG_LEAVE;
      return true;
    }
  }
  else if (d1 <= d0) { // after ls.b
    if (norm(x.b - y.center) < y.r) {
      // DEBUG_COUT("Found collision, line segment b is within sphere.");
      // DEBUG_LEAVE;
      return true;
    }
  }
  else {
    Vec3 closest = x.a + (d0 / d1) * v;
    if (norm(closest - y.center) < y.r) {
      // DEBUG_COUT("Found collision, line segment intersects sphere in the middle.");
      // DEBUG_LEAVE;
      return true;
    }
  }

  // DEBUG_COUT("Found no collision.");
  // DEBUG_LEAVE;
  return false;
}


bool intersect(Sphere x, Sphere y) {
  // DEBUG_ENTER(__PRETTY_FUNCTION__);
  // DEBUG_LEAVE;
  return norm(y.center - x.center) <= (x.r + y.r);
}


bool intersect(Cylinder y, Sphere x) {
  return intersect(x, y);
}


/* Prism intersections */




bool intersect(Prism x, Vec3 y) {
  return intersect(y, x);
}


bool intersect(Vec3 x, Prism y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // transform x into coordinate system of prism
  Vec3 x_ = rotate_point(x, y.center, inverse(y.orientation)) - y.center;
  DEBUG_LEAVE;
  return
    abs(x_.x) <= y.ex &&
    abs(x_.y) <= y.ey &&
    abs(x_.z) <= y.ez;
}


bool intersect(Prism x, LineSegment y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // see if either extrema of y is inside x
  if ( intersect(x, y.a) || intersect(x, y.b) ) {
    DEBUG_COUT("Collision found: endpoint of line segment is inside prism.");
    DEBUG_LEAVE;
    return true;
  }

  LineSegment transformed = {
    rotate_point(y.a, x.center, inverse(x.orientation)) - x.center,
    rotate_point(y.b, x.center, inverse(x.orientation)) - x.center
  };
  auto disp = transformed.b - transformed.a;
  double t, _x, _y, _z;

  // z+
  t = (x.ez - transformed.a.z) / disp.z;
  if (0 <= t && t <= 1) {
    // possible intersection with top
    _x = fabs(transformed.a.x + t * disp.x);
    _y = fabs(transformed.a.y + t * disp.y);
    if (_x <= x.ex && _y <= x.ey) {
      DEBUG_COUT("Solution found for segment colliding with +z.");
      DEBUG_LEAVE;
      return true;
    }
  }
  // z-
  t = (-x.ez - transformed.a.z) / disp.z;
  if (0 <= t && t <= 1) {
    _x = fabs(transformed.a.x + t * disp.x);
    _y = fabs(transformed.a.y + t * disp.y);
    if (_x <= x.ex && _y <= x.ey) {
      DEBUG_COUT("Solution found for segment colliding with -z.");
      DEBUG_LEAVE;
      return true;
    }
  }
  // y+
  t = (x.ey - transformed.a.y) / disp.y;
  if (0 <= t && t <= 1) {
    _x = fabs(transformed.a.x + t * disp.x);
    _z = fabs(transformed.a.z + t * disp.z);
    if (_x <= x.ex && _z <= x.ez) {
      DEBUG_COUT("Solution found for segment colliding with +y.");
      DEBUG_LEAVE;
      return true;
    }
  }
  // y-
  t = (-x.ey - transformed.a.y) / disp.y;
  if (0 <= t && t <= 1) {
    _x = fabs(transformed.a.x + t * disp.x);
    _z = fabs(transformed.a.z + t * disp.z);
    if (_x <= x.ex && _z <= x.ez) {
      DEBUG_COUT("Solution found for segment colliding with -y.");
      DEBUG_LEAVE;
      return true;
    }
  }
  // x+
  t = (x.ex - transformed.a.x) / disp.x;
  if (0 <= t && t <= 1) {
    _y = fabs(transformed.a.y + t * disp.y);
    _z = fabs(transformed.a.z + t * disp.z);
    if (_y <= x.ey && _z <= x.ez) {
      DEBUG_COUT("Solution found for segment colliding with +x.");
      DEBUG_LEAVE;
      return true;
    }
  }
  // x-
  t = (x.ex - transformed.a.x) / disp.x;
  if (0 <= t && t <= 1) {
    _y = fabs(transformed.a.y + t * disp.y);
    _z = fabs(transformed.a.z + t * disp.z);
    if (_y <= x.ey && _z <= x.ez) {
      DEBUG_COUT("Solution found for segment colliding with -x.");
      DEBUG_LEAVE;
      return true;
    }
  }
  DEBUG_COUT("No solution exists for any surface: no collision.");
  DEBUG_LEAVE;
  return false;
}


// this is a specialized version of the SAT algorithm used other places
// it uses the shape of the prisms to do fewer checks
bool intersect(Prism x, Prism y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // first, check if bounding spheres intersect

  // distance vector between centroids
  Vec3 c2c = x.center - y.center;
  // norm squared of distance between centroids, bounding sphere radii, squared sum of radii
  double
    c2c2   = (c2c.x*c2c.x) + (c2c.y*c2c.y) + (c2c.z*c2c.z),
    rx     = sqrt((x.ex*x.ex) + (x.ey*x.ey) + (x.ez*x.ez)),
    ry     = sqrt((y.ex*y.ex) + (y.ey*y.ey) + (y.ez*y.ez)),
    vdist2 = (rx + ry) * (rx + ry);

  // cout << endl << "Testing prisms..." << endl;

  if (c2c2 > vdist2) {
    // cout << "Bounding spheres don't intersect." << endl;
    DEBUG_COUT("Bounding spheres do not intersect.");
    DEBUG_LEAVE;
    return false; // bounding spheres do not intersect
  }

  DEBUG_COUT("Bounding spheres intersect. Running full test.");

  // now check if any points are inside of the other
  // TODO: benchmark if this improves performance

  auto xpts = x.vertexes();
  auto ypts = y.vertexes();


  // Now that we've done cheap tests, we have to use SAT
  // From geometrictools.com
  // We must find the projection of each point on a series of axes
  // The axes are the set of the normals of the two boxes union the set of cross products of the edges of the two boxes
  // This would be a total of 12 axes from the normals, plus 12^2 = 144 axes from the cross products of 12 edges each
  // However, there will only be 3^2 distinct cross products because we have rectangular prisms.

  array<Vec3, 12+9> axes;
  // generate axes from normals

  {
    auto x_normals = x.normals();
    memcpy(axes.data(), x_normals.data(), 6 * sizeof(Vec3));
    auto y_normals = y.normals();
    memcpy(axes.data() + 6, y_normals.data(), 6 * sizeof(Vec3));
  }

  // generate axes from line segments
  // this depends on the order we generate points
  {
    // Generate x and y axes separately
    // From the order of points generated the three axes are guaranteed to be linearly independent
    // Take a look at the `lineSegments` function. This is equivalent to the first, second, and fifth segments generated
    Vec3 xes[3], yes[3];
    xes[0] = xpts[4] - xpts[0];
    yes[0] = ypts[4] - ypts[0];
    xes[1] = xpts[1] - xpts[0];
    yes[1] = ypts[1] - ypts[0];
    xes[2] = xpts[2] - xpts[1];
    yes[2] = ypts[2] - ypts[1];

    for (int k = 0; k < 9; k++) {
      int i = k%3, j = k/3;
      axes[12+k] = cross(xes[i], yes[j]);
    }
  }

  array<double, 8> projected_x, projected_y;

  for (int i = 0; i < (12+9); i++) {
    for (int j = 0; j < 8; j++) {
      projected_x[j] = axes[i] * (xpts[j]);
      projected_y[j] = axes[i] * (ypts[j]);
    }

    auto x_extr = extrema(projected_x);
    auto y_extr = extrema(projected_y);

    // cout << "  Extrema x: " << xExtr.first << ", " << xExtr.second << endl;
    // cout << "  Extrema y: " << yExtr.first << ", " << yExtr.second << endl;
    if ((x_extr.second < y_extr.first) || (y_extr.second < x_extr.first)) {
      DEBUG_COUT("Separating axis found.");
      DEBUG_LEAVE;
      return false;
    }
  }

  DEBUG_COUT("No separating axis found, there must be a collision.");
  DEBUG_LEAVE;
  return true;
}


bool intersect(Prism x, Sphere y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // This check is quite simple:
  // Iterate over each line segment in the prism, and calculate the closest point from that line segment to the center
  //   of that sphere. If it's less than the radius, return true.
  // Closest distance between point&line algorithm from geomalgorithms.com

  auto ls = x.edges();

  for (int i = 0; i < 12; i++) {
    if (intersect(ls[i], y)) {
      DEBUG_COUT("Found intersection of sphere and prism edge " << i);
      DEBUG_LEAVE;
      return true;
    }
  }
  DEBUG_COUT("None of the prism edges intersect with the sphere.");
  DEBUG_LEAVE;
  return false;
}


// Checks for intersections between a prism and the **side** of a cylinder. Does not check ends of the cylinder.
// For each line segment in the prism, check if there is a solution to that and the circle
bool intersect(Prism x, Cylinder y) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // first, find points of prism in frame of cylinder
  auto ls = x.edges();

  {
    array<Vec3, 24> pts;
    for (int i = 0; i < 12; i++) {
      pts[2*i]   = ls[i].a;
      pts[2*i+1] = ls[i].b;
    }
    rotate_points_inplace(pts, y.center, y.orientation);
    for (int i = 0; i < 12; i++) {
      ls[i].a = pts[2*i];
      ls[i].b = pts[2*i+1];
    }
  }

  for (int i = 0; i < 12; i++) {
    LineSegment l = ls[i];

    Vec3 d = l.b - l.a;

    double denom = (d.x*d.x) + (d.y*d.y);
    if (abs(denom) < APPROX)
      continue;
    double det = ((y.r*y.r) * (d.x*d.x))
               + ((y.r*y.r) * (d.y*d.y))
               - ((d.x*d.x) * (l.a.y*l.a.y))
               - ((d.y*d.y) * (l.a.x*l.a.x))
               + (2*d.x*d.y*l.a.x*l.a.y);
    if (det - APPROX <= 0)
      continue;

    det = sqrt(det);

    double
      num = -(d.x*l.a.x) - (d.y*l.a.y),
      t1  = (num + det) / denom,
      t2  = (num - det) / denom;

    if (t1 >= 0 && t1 <= 1) {
      double z = l.a.z + (t1 * d.z);
      if (abs(z) < y.e) {
        DEBUG_COUT("t1 solution found.")
        DEBUG_LEAVE;
        return true;
      }
    }

    if (t2 >= 0 && t2 <= 1) {
      double z = l.a.z + (t2 * d.z);
      if (abs(z) < y.e) {
        DEBUG_COUT("t2 solution found.")
        DEBUG_LEAVE;
        return true;
      }
    }

  }
  
  DEBUG_COUT("No collision found.");
  DEBUG_LEAVE;
  return false;
}


struct static_intersect_visitor : public boost::static_visitor<bool> {
  static_intersect_visitor(Prism prism) : p(prism) {}
  Prism p;

  template<typename T>
  bool operator()(T t) const {
    return intersect(this->p, t);
  }

  // bool operator()(LineSegment& y) {
  //   return intersect(this->p, y);
  // }

  // bool operator()(Prism& y) {
  //   return intersect(this->p, y);
  // }

  // bool operator()(Sphere& y) {
  //   return intersect(this->p, y);
  // }

  // bool operator()(Cylinder& y) {
  //   return intersect(this->p, y);
  // }
};


bool intersect(Prism x, Intersectable y) {
  auto visitor = static_intersect_visitor(x);
  return boost::apply_visitor(visitor, y);
}


bool intersect(Prism x, vector<Intersectable> ys) {
  auto visitor = static_intersect_visitor(x);
  for (size_t i = 0; i < ys.size(); i++) {
    if (boost::apply_visitor(visitor, ys[i])) return true;
  }
  return false;
}
