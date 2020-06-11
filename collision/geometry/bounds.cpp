#include "geom.hpp"


Sphere bounding_sphere(Vec3 x) {
  return Sphere {
    x, 1e-9
  };
}

Sphere bounding_sphere(LineSegment x) {
  return Sphere {
    x.a + 0.5 * (x.b - x.a),
    norm(x.b - x.a)
  };
}

Sphere bounding_sphere(Prism x) {
  return Sphere {
    x.center,
    norm({x.ex, x.ey, x.ez})
  };
}

Sphere bounding_sphere(Sphere x) {
  return x;
}

Sphere bounding_sphere(Cylinder x) {
  return Sphere {
    x.center,
    sqrt((x.e * x.e) + (x.r * x.r))
  };
}

Sphere bounding_sphere(const vector<Vec3>& vertexes) {
  auto c = centroid(vertexes);
  double max_dist = -1;
  for (size_t i = 0; i < vertexes.size(); i++) {
    double dist = norm(vertexes[i] - c);
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  return Sphere {
    c, max_dist
  };
}

struct bounding_visitor : public boost::static_visitor<Sphere> {
  template<typename T>
  Sphere operator()(T t) const {
    return bounding_sphere(t);
  }
};

Sphere bounding_sphere(Intersectable x) {
  return boost::apply_visitor(bounding_visitor(), x);
}
