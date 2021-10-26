#include "geom.hpp"
#include "sat.hpp"

/*
[x] Prism Vec3
[x] Prism LineSegment
[x] Prism Prism
[x] Prism Sphere
[x] Prism Cylinder
*/


using namespace std;


bool intersect(Prism x, Vec3 y, Vec3 disp) {
  LineSegment ls = { y, y - disp };
  return intersect(x, ls);
}


bool intersect(Prism x, LineSegment y, Vec3 disp) {
  DEBUG_ENTER(__PRETTY_FUNCTION__)
  // Uses SAT but is exact
  std::vector<Vec3> pts;
  pts.reserve(4);
  pts.push_back(y.a);
  pts.push_back(y.b);
  pts.push_back(y.b - disp);
  pts.push_back(y.a - disp);

  const ConvexPolygon p = { pts };

  DEBUG_COUT("Built polygon, n=" << p.vertexes.size());

  const auto res = intersect(polyhedron(x), p);
  DEBUG_LEAVE;
  return res;
}


// creates the polyhedron resulting from sweeping a prism
ConvexPolyhedron _sweep(const Prism p, Vec3 disp) {
  vector<IdxPair> edges;
  edges.reserve(24+8);
  auto pts = p.vertexes();
  const vector<Vec3> vertexes = {
    pts[0], pts[1], pts[2], pts[3], pts[4], pts[5], pts[6], pts[7],
    pts[0]+disp, pts[1]+disp, pts[2]+disp, pts[3]+disp, pts[4]+disp, pts[5]+disp, pts[6]+disp, pts[7]+disp
  };
  for (size_t i = 0; i < 8; i++) {
    edges.push_back(make_pair(i,i+8));
  }
  for (size_t i = 0; i < 8; i++) {
    edges.push_back(make_pair(i,   i+4));
    edges.push_back(make_pair(i,   (i+1)%4));
    edges.push_back(make_pair(i+4, ((i+1)%4)+4));
    edges.push_back(make_pair(i+4,   i+4+4));
    edges.push_back(make_pair(i+4,   (i+1)%4+4));
    edges.push_back(make_pair(i+4+4, ((i+1)%4)+4+4));
  }

  const vector<Vec3> normals = {
    rotate_point( Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_z(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_z(), Vec3::zero(), p.orientation)
  };

  return { vertexes, edges, normals };
}

// creates the polyhedron resulting from sweeping a prism
Prism _sweepPrism(const Prism p, Vec3 disp) {
  return { p.center+0.5*disp, p.ex+fabs(disp.x), p.ey+fabs(disp.y), p.ez+fabs(disp.z), p.orientation};
}

// todo: use shape of prisms to reduce checks needed
bool intersect(Prism x, Prism y, Vec3 disp) {
  return intersect(_sweep(x, disp), polyhedron(y));
}

bool intersect(Prism x, Cylinder y, Vec3 disp) {
  //auto p = _sweep(x, disp);
  // if (!intersect(bounding_cylinder(p, disp), bounding_sphere(y))) return false;
  //auto c = polyhedron(y);
  return intersect(_sweepPrism(x,disp), y);
}


/* Intersection with sphere */


static const unordered_set<int> INTERSECTION_REGIONS = {0,1,2,3,5,6,8,10};


inline boost::optional<Vec3> _point_of_collision(LineSegment x, Sphere y) {
  Vec3
    v = x.b - x.a,
    w = y.center - x.a;

  double
    d0 = w * v,
    d1 = v * v;

  if (d0 <= 0 || d1 <= d0) {  // before ls.a
    return boost::none;
  }
  else {
    return x.a + (d0 / d1) * v;
  }
}


inline boost::optional<Vec3> _point_of_collision(LineSegment x, Cylinder y) {
  LineSegment l = {
    rotate_point(x.a, y.center, inverse(y.orientation)),
    rotate_point(x.b, y.center, inverse(y.orientation))
  };
  Vec3 d = l.b - l.a;

  double denom = (d.x*d.x) + (d.y*d.y);

  if (abs(denom) < APPROX)
    return boost::none;

  double det = ((y.r*y.r) * (d.x*d.x))
             + ((y.r*y.r) * (d.y*d.y))
             - ((d.x*d.x) * (l.a.y*l.a.y))
             - ((d.y*d.y) * (l.a.x*l.a.x))
             + (2*d.x*d.y*l.a.x*l.a.y);

  if (det - APPROX <= 0)
    return boost::none;

  det = sqrt(det);

  double
    num = -(d.x*l.a.x) - (d.y*l.a.y),
    t1  = (num + det) / denom,
    t2  = (num - det) / denom;

  if (t1 >= 0 && t1 <= 1) {
    double z = l.a.z + (t1 * d.z);
    if (abs(z) < y.e)
      return t1 * Vec3(d.x, d.y, z) + x.a;
  }

  if (t2 >= 0 && t2 <= 1) {
    double z = l.a.z + (t2 * d.z);
    if (abs(z) < y.e)
      return t2 * Vec3(d.x, d.y, z) + x.a;
  }

  return boost::none;
}


// sphere, source, displacement, normal 1, normal 2
bool _sphere_intersect(Sphere s, Vec3 c, Vec3 disp, Vec3 n1, Vec3 n2, Vec3 n3) {
  // c.x + t*disp.x
  LineSegment ls = {c, c+disp};
  auto _p = _point_of_collision(ls, s);

  if (!_p) return false; // no collision

  // there is a collision, now check if in correct octant
  // should pass vectors along borders of octant
  auto
    p   = *_p,
    del = p - s.center;

  return (del * n1) >= 0 && (del * n2) >= 0 && (del * n3) >= 0;
}


// cylinder, source, displacement, normal 1, normal 2
bool _cyl_intersect(Cylinder cyl, Vec3 c, Vec3 disp, Vec3 n1, Vec3 n2) {
  // c.x + t*disp.x
  return true;
  LineSegment ls = {c, c+disp};
  auto _p = _point_of_collision(ls, cyl);

  if (!_p) return false;

  auto 
    p   = *_p,
    del = p - cyl.center;

  return (del * n1) >= 0 && (del * n2) >= 0;
}


enum IntersectOpinion {
  Intersects,
  NoIntersect,
  MaybeIntersect
};


// does the ray intersect with the box faces?
IntersectOpinion _superbox_intersect(Vec3 c, Vec3 disp, Vec3 extents, double r, uint8_t xn, uint8_t x1, uint8_t x2) {
  // collision with plane in normal direction
  double t = ((extents[xn] + r) - c[xn]) / disp[xn];

  // no intersection if outside disp range
  if (0 > t || t > 1) return NoIntersect;

  double
    i1 = c[x1] + (t * disp[x1]),
    i2 = c[x2] + (t * disp[x2]);

  // no intersection if i1 or i2 is out of the bounds of the super box
  if (fabs(i1) > extents[x1] + r || fabs(i2) > extents[x2] + r) return NoIntersect;

  // intersection on rounded box rectangular face
  if (fabs(i1) < extents[x1] && fabs(i2) < extents[x2]) return Intersects;

  return MaybeIntersect;
}


// does the ray intersect with the end spheres?
// note: we don't technically need to check every single sphere, however it's just easier to do this an will still be plenty fast.
//       
bool _sphereend_intersect(Vec3 c, Vec3 disp, Vec3 extents, double r) {
  const auto
    X = Vec3::basis_x(),
    Y = Vec3::basis_y(),
    Z = Vec3::basis_z();

  // spheres in first quadrant
  Sphere s = {Vec3(extents.x, extents.y, extents.z), r};
  if (_sphere_intersect(s, c, disp, Vec3::basis_x(), Vec3::basis_x(), Z)) return true;
  s.center.z *= -1;
  if (_sphere_intersect(s, c, disp, X, Y, -Z)) return true;

  // spheres in second quadrant
  s = {Vec3(extents.x, -extents.y, extents.z), r};
  if (_sphere_intersect(s, c, disp, X, -Y, Z)) return true;
  s.center.z *= -1;
  if (_sphere_intersect(s, c, disp, X, -Y, -Z)) return true;

  // spheres in third quadrant
  s = {Vec3(-extents.x, extents.y, extents.z), r};
  if (_sphere_intersect(s, c, disp, -X, Y, Z)) return true;
  s.center.z *= -1;
  if (_sphere_intersect(s, c, disp, -X, Y, -Z)) return true;

  // sphere in fourth quadrant
  s = {Vec3(-extents.x, -extents.y, extents.z), r};
  if (_sphere_intersect(s, c, disp, -X, -Y, Z)) return true;
  s.center.z *= -1;
  if (_sphere_intersect(s, c, disp, -X, -Y, -Z)) return true;

  return false;
}


// checks if we've collided with side cylinders
bool _cylinder_intersect(Vec3 c, Vec3 disp, Vec3 extents, double r) {
  const auto
    X = Vec3::basis_x(),
    Y = Vec3::basis_y(),
    Z = Vec3::basis_z();

  Cylinder cyl;

  // first -> second quadrant
  cyl = {Vec3(extents.x, 0, extents.z), r, extents.y, Quaternion::from_axis_angle(X, PI/2)};
  if (_cyl_intersect(cyl, c, disp, X,  Z)) return true;
  cyl.center.z *= -1;
  if (_cyl_intersect(cyl, c, disp, X, -Z)) return true;

  // second -> third quadrant
  cyl = {Vec3(0, extents.y, extents.z), r, extents.x, Quaternion::from_axis_angle(Y, PI/2)};
  if (_cyl_intersect(cyl, c, disp, Y,  Z)) return true;
  cyl.center.z *= -1;
  if (_cyl_intersect(cyl, c, disp, Y, -Z)) return true;

  // third -> fourth quadrant
  cyl = {Vec3(-extents.x, 0, extents.z), r, extents.y, Quaternion::from_axis_angle(X, PI/2)};
  if (_cyl_intersect(cyl, c, disp, -X,  Z)) return true;
  cyl.center.z *= -1;
  if (_cyl_intersect(cyl, c, disp, -X, -Z)) return true;

  // fourth -> first quadrant
  cyl = {Vec3(0, -extents.y, extents.z), r, extents.x, Quaternion::from_axis_angle(Y, PI/2)};
  if (_cyl_intersect(cyl, c, disp, -Y,  Z)) return true;
  cyl.center.z *= -1;
  if (_cyl_intersect(cyl, c, disp, -Y, -Z)) return true;

  return false;
}


bool intersect(Prism x, Sphere y, Vec3 disp) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);

  // Taken from geometrictools.com
  // This is a bit of a monster function. It follows the algorithm at https://www.geometrictools.com/Documentation/IntersectionMovingSphereBox.pdf.
  //   There are a couple of changes:
  //       we're using a displacement instead of a velocity
  //       we only report if there is or isn't an intersection, we don't care about when

  // First, transform sphere to frame where prism is centred at origin and oriented along global dimensions
  y.center = rotate_point(y.center - x.center, Vec3::zero(), inverse(x.orientation));
  // disp is prism displacement, so to find sphere displacement 
  disp = -disp;

  // Now, transform so y.center is in first octant
  if (y.center.x < 0) {
    y.center.x *= -1;
    disp.x     *= -1;
  }
  if (y.center.y < 0) {
    y.center.y *= -1;
    disp.y     *= -1;
  }
  if (y.center.z < 0) {
    y.center.z *= -1;
    disp.z     *= -1;
  }

  // Now we have to find which region the sphere's center falls into
  Vec3 vert = {x.ex, x.ey, x.ez};
  Vec3 delt = y.center - vert;

  // Could be split up into a tree but this is honestly more readable
  uint8_t region = 255;

  const bool
    dx_le_0 = delt.x <= 0,
    dy_le_0 = delt.y <= 0,
    dz_le_0 = delt.z <= 0,
    dx_le_r = delt.x <= y.r,
    dy_le_r = delt.y <= y.r,
    dz_le_r = delt.z <= y.r,
    dx_gt_0_le_r = !dx_le_0 && dx_le_r,
    dy_gt_0_le_r = !dy_le_0 && dy_le_r,
    dz_gt_0_le_r = !dz_le_0 && dz_le_r,
    dx2_dy2_le_r2 = (delt.x * delt.x) + (delt.y * delt.y) <= (y.r * y.r),
    dy2_dz2_le_r2 = (delt.y * delt.y) + (delt.z * delt.z) <= (y.r * y.r),
    dx2_dz2_le_r2 = (delt.x * delt.x) + (delt.z * delt.z) <= (y.r * y.r),
    dx2_dy2_dz2_le_r2 = (delt.x * delt.x) + (delt.y * delt.y) + (delt.z * delt.z) <= (y.r * y.r);

  if (dx_le_0  &&  dy_le_0  &&  dz_le_0)
    region = 0;
  else if (dx_gt_0_le_r  &&  dy_le_0  &&  dz_le_0)
    region = 1;
  else if (dx_le_0  &&  dy_gt_0_le_r  &&  dz_le_0)
    region = 2;
  else if (dx_gt_0_le_r  &&  dy_gt_0_le_r  &&  dx2_dy2_le_r2)
    region = 3;
  else if (dx_gt_0_le_r  &&  dy_gt_0_le_r  &&  !dx2_dy2_le_r2)
    region = 4;
  else if (dx_le_0  &&  dy_le_0  &&  dz_gt_0_le_r)
    region = 5;
  else if (dx_gt_0_le_r  &&  dy_le_0  &&  dz_gt_0_le_r  &&   dx2_dz2_le_r2)
    region = 6;
  else if (dx_gt_0_le_r  &&  dy_le_0  &&  dz_gt_0_le_r  &&  !dx2_dz2_le_r2)
    region = 7;
  else if (dx_le_0  &&  dy_gt_0_le_r  &&  dz_gt_0_le_r  &&   dy2_dz2_le_r2)
    region = 8;
  else if (dx_le_0  &&  dy_gt_0_le_r  &&  dz_gt_0_le_r  &&  !dy2_dz2_le_r2)
    region = 9;
  else if (dx_gt_0_le_r  &&  dy_gt_0_le_r  &&  dz_gt_0_le_r  &&   dx2_dy2_dz2_le_r2)
    region = 10;
  else if (dx_gt_0_le_r  &&  dy_gt_0_le_r  &&  dz_gt_0_le_r  &&  !dx2_dy2_dz2_le_r2)
    region = 11;
  else if (!dx_le_r  &&  dy_le_r  &&  dz_le_r)
    region = 12;
  else if ( dx_le_r  && !dy_le_r  &&  dz_le_r)
    region = 13;
  else if (!dx_le_r  && !dy_le_r  &&  dz_le_r)
    region = 14;
  else if ( dx_le_r  &&  dy_le_r  && !dz_le_r)
    region = 15;
  else if (!dx_le_r  &&  dy_le_r  && !dz_le_r)
    region = 16;
  else if ( dx_le_r  && !dy_le_r  && !dz_le_r)
    region = 17;
  else if (!dx_le_r  && !dy_le_r  && !dz_le_r)
    region = 18;
  else {
    DEBUG_COUT(C_BR_RED << "Error: incorrect region. Reporting collision." << C_RESET);
    cerr << C_BR_RED << "Error: incorrect region. Reporting collision." << C_RESET;
    DEBUG_LEAVE;
    return true;
  }

  DEBUG_COUT("Found region " << ((int)region));

  if (INTERSECTION_REGIONS.count(region)) {
    // in a region with immediate collision
    DEBUG_COUT("Immediate collision.");
    DEBUG_LEAVE;
    return true;
  } else if (disp == Vec3::zero()) {
    // no immediate intersection, and no velocity, so there can be no collision
    DEBUG_COUT("No immediate collision and displacement is zero.");
    DEBUG_LEAVE;
    return false;
  }
  // no immediate intersection, but we must check if one is possible in the future

  if (region == 11) {
    DEBUG_COUT("This is a vertex-separated region.");
    // vertex-separated

    double
      a0 = y.r * y.r,
      a1 = 2 * (disp * delt),
      a2 = norm2(disp);

    if (fabs(a2) < 1e-9) {
      DEBUG_COUT(C_BR_RED << "Error: displacement is zero. Probably no collision, I guess, but this shoudln't happen." << C_RESET);
      DEBUG_LEAVE;
      return false;
    }

    double t = (-a1 - sqrt(a1*a1 - a2*a0)) / a2;
    DEBUG_COUT("Found t=" << t);
    DEBUG_LEAVE;
    return (t >= 0 && t <= 1); 
  }

  else if (region == 4 || region == 7 || region == 9) {
    DEBUG_COUT("Edge-separated region.");
    // edge separated
    // R4 => xy
    // R7 => xz
    // R9 => yz
    uint8_t
      i0 = region == 4 ? 0 : region == 7 ? 2 : 1,
      i1 = region == 4 ? 1 : region == 7 ? 0 : 2;
      //i2 = region == 4 ? 2 : region == 7 ? 1 : 0;
    
    double
      b0 = (disp[i0]*disp[i0]) + (disp[i1]*disp[i1]),
      b1 = 2 * ((disp[i0]*delt[i0]) + (disp[i1]*delt[i1])),
      b2 = (delt[i0]*delt[i0]) + (delt[i1]*delt[i1]) - (y.r*y.r);
    
    if (fabs(b2) < 1e-9) {
      DEBUG_LEAVE;
      return true;
    }

    double t = (-b1 - sqrt(b1*b1 - b2*b0)) / b2;
    DEBUG_COUT("Found t=" << t);
    DEBUG_LEAVE;
    return (t >= 0 && t <= 1); 
  }

  else {
    DEBUG_COUT("Unbound region.");
    // unbounded
    // test for intersection on three relevant faces of "super box"
    // c = y.center
    const static array< tuple<uint8_t, uint8_t, uint8_t>, 3> permutations = {{
      make_tuple((uint8_t)0, (uint8_t)1, (uint8_t)2),
      make_tuple((uint8_t)2, (uint8_t)0, (uint8_t)1),
      make_tuple((uint8_t)1, (uint8_t)2, (uint8_t)0)
    }};

    bool all_nointersect = true;

    for (size_t i = 0; i < 3; i++) {
      const auto perm = permutations[i];

      switch(_superbox_intersect(y.center, disp, vert, y.r, get<0>(perm), get<1>(perm), get<2>(perm))) {
        case Intersects:
          DEBUG_COUT("Superbox intersected.");
          DEBUG_LEAVE;
          return true;
        case NoIntersect:
          continue;
        default:  // might intersect
          all_nointersect = false;
          continue;
      }
    }

    if (all_nointersect) {
      DEBUG_COUT("All permutations reported no collision.");
      DEBUG_LEAVE;
      return false;
    }
    // one of the sides of the superbox reported "maybe", so we have to check end spheres and cylinders.
    else {
      if (_sphereend_intersect(y.center, disp, vert, y.r)) {
        DEBUG_COUT("Found intersection with a sphere octant.");
        DEBUG_LEAVE;
        return true;
      }
      else if (_cylinder_intersect(y.center, disp, vert, y.r)) {
        DEBUG_COUT("Found intersection with a quarter cylinder.");
        DEBUG_LEAVE;
        return true;
      }
    }

  }

  DEBUG_COUT("No intersections found.");
  DEBUG_LEAVE;
  return false;
}


struct disp_intersect_visitor : public boost::static_visitor<bool> {
  disp_intersect_visitor(Prism prism, Vec3 disp) : p(prism), d(disp) {}
  Prism p;
  Vec3  d;

  template<typename T>
  bool operator()(T t) const {
    return intersect(p, t, d);
  } 
};


bool intersect(Prism x, Intersectable y, Vec3 disp) {
  auto visitor = disp_intersect_visitor(x, disp);
  return boost::apply_visitor(visitor, y);
}


bool intersect(Prism x, vector<Intersectable> ys, Vec3 disp) {
  auto visitor = disp_intersect_visitor(x, disp);
  for (size_t i = 0; i < ys.size(); i++) {
    if (boost::apply_visitor(visitor, ys[i])) return true;
  }
  return false;
}

bool intersect(std::array<Prism, 3> x, vector<Intersectable> ys, Vec3 disp) {
  for (size_t i = 0; i < 3; i++) {
    intersect(x[i],ys,disp);
  }
  return false;
}