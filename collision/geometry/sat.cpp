#include "sat.hpp"
#include "col.hpp"
#include <algorithm>

#ifdef DEBUG
#include "serialization.hpp"
#endif

using namespace std;


ConvexPolyhedron to_polyhedron(const ConvexPolygon& p) {
  vector<IdxPair> edges;
  edges.reserve(p.vertexes.size());
  for (size_t i = 0; i <= p.vertexes.size(); i++) {
    edges.push_back(make_pair(i, i + 1 % p.vertexes.size()));
  }
  ConvexPolyhedron ret = {
    p.vertexes,
    edges,
    { normal(p) }
  };
  return ret;
}


Vec3 centroid(const ConvexPolygon& poly) {
  return centroid(poly.vertexes);
}

Vec3 centroid(const ConvexPolyhedron& poly) {
  return centroid(poly.vertexes);
}


void scale_around(ConvexPolygon& poly, Vec3 about, double factor) {
  for (size_t i = 0; i < poly.vertexes.size(); i++) {
    poly.vertexes[i] = about + factor * (poly.vertexes[i] - about);
  }
}


void scale_around(ConvexPolyhedron& poly, Vec3 about, double factor) {
  for (size_t i = 0; i < poly.vertexes.size(); i++) {
    poly.vertexes[i] = about + factor * (poly.vertexes[i] - about);
  }
}


void scale(ConvexPolygon& poly, double factor) {
  scale_around(poly, centroid(poly), factor);
}


void scale(ConvexPolyhedron& poly, double factor) {
  scale_around(poly, centroid(poly), factor);
}


ConvexPolyhedron polyhedron(const Prism p) {
  const auto pts = p.vertexes();

  const vector<Vec3> vertexes = {
    pts[0], pts[1], pts[2], pts[3], pts[4], pts[5], pts[6], pts[7]
  };

  vector<IdxPair> edges;
  edges.reserve(12);

  for (size_t i = 0; i < 4; i++) {
    edges.push_back(make_pair(i,   i+4));
    edges.push_back(make_pair(i,   (i+1)%4));
    edges.push_back(make_pair(i+4, ((i+1)%4)+4));
  }

  const vector<Vec3> normals = {
    rotate_point( Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_z(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_z(), Vec3::zero(), p.orientation)
  };

  const ConvexPolyhedron ret = { vertexes, edges, normals };

  return ret;
}

void push_edge(vector<IdxPair> &edges, IdxPair pair){
  if(std::find(edges.begin(), edges.end(), pair) == edges.end()) {
    edges.push_back(pair);
  }
}

ConvexPolyhedron polyhedron(const ColStlMesh mesh){
  vector<Vec3> normals;
  vector<Vec3> vertexes;
  vector<IdxPair> edges;

  /*const vector<Vec3> normals = {
    rotate_point( Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point( Vec3::basis_z(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_x(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_y(), Vec3::zero(), p.orientation),
    rotate_point(-Vec3::basis_z(), Vec3::zero(), p.orientation)
  };*/
  for(size_t i = 0; i < mesh.num_vrts(); i++){
    const float* v = mesh.vrt_coords(i);
    vertexes.push_back(Vec3(v[0],v[1],v[2]));
  }

  for(size_t i = 0; i < mesh.num_tris(); i++) {
      const unsigned int* indexes =  mesh.tri_corner_inds(i);
      push_edge(edges, make_pair(indexes[0],   indexes[1]));
      push_edge(edges, make_pair(indexes[0],   indexes[2]));
      push_edge(edges, make_pair(indexes[1],   indexes[2]));
      const float* n = mesh.tri_normal (i);
      normals.push_back(Vec3(n[0],n[1],n[2]));
  }

  const ConvexPolyhedron ret = { vertexes, edges, normals };

  return ret;
}

ConvexPolyhedron translate(const ConvexPolyhedron mesh,Vec3 disp){
  const vector<Vec3> pts = mesh.vertexes;

  vector<Vec3> vertices;

  for(Vec3 v:pts){
    vertices.push_back(v+disp);
  }

  const ConvexPolyhedron ret = { vertices, mesh.edges, mesh.normals };
  return ret;
}

ConvexPolyhedron rotate(const ConvexPolyhedron mesh,Quaternion orientation,Vec3 rotation_point){
  const vector<Vec3> pts = mesh.vertexes;
  const vector<Vec3> norms = mesh.normals;

  vector<Vec3> normals;
  vector<Vec3> vertices;

  for(Vec3 v:pts){
    vertices.push_back(rotate_point( v, rotation_point, orientation));
  }
  for(Vec3 n:norms)
    normals.push_back(rotate_point( n, Vec3::zero(), orientation));//should this rotate the opposite way?

  const ConvexPolyhedron ret = { vertices, mesh.edges, normals };
  return ret;
  
}

ConvexPolyhedron scale(const ConvexPolyhedron mesh,Vec3 scale_vec,Vec3 scale_point){
  const vector<Vec3> pts = mesh.vertexes;

  vector<Vec3> vertices;

  for(Vec3 v:pts){
    vertices.push_back( scale(v-scale_point,scale_vec) + scale_point);
  }

  const ConvexPolyhedron ret = { vertices, mesh.edges, mesh.normals };
  return ret;
}

ConvexPolyhedron polyhedron(const Cylinder c) {
  vector<Vec3> vertexes;
  vertexes.reserve(2 * NUM_NORMALS_FOR_CYLINDER);
  vector<IdxPair> edges;
  edges.reserve(2 * NUM_NORMALS_FOR_CYLINDER);
  vector<Vec3> normals;
  normals.reserve(NUM_NORMALS_FOR_CYLINDER);

  const double dtheta = 2 * PI / NUM_NORMALS_FOR_CYLINDER;

  for (size_t i = 0; i < NUM_NORMALS_FOR_CYLINDER; i++) {
    vertexes.push_back(rotate_point(
      {c.r*cos(i*dtheta), c.r*sin(i*dtheta), c.e},
      Vec3::zero(), c.orientation
    ) + c.center);
    vertexes.push_back(rotate_point(
      {c.r*cos(i*dtheta), c.r*sin(i*dtheta), -c.e},
      Vec3::zero(), c.orientation
    ) + c.center);
    edges.push_back(make_pair(2*i, 2*i+1));
    if(i<NUM_NORMALS_FOR_CYLINDER-1){
      edges.push_back(make_pair(2*i, 2*(i+1)));
      edges.push_back(make_pair(2*i+1, 2*(i+1)+1));
    }else{
      edges.push_back(make_pair(2*i+0, 0));
      edges.push_back(make_pair(2*i+1, 1));
    }
    normals.push_back(rotate_point(
      { cos(i*dtheta), sin(i*dtheta), 0.0 },
      Vec3::zero(),
      c.orientation
    ));
    auto v = rotate_point(
      {c.r*cos(i*dtheta), c.r*sin(i*dtheta), c.e},
      Vec3::zero(), c.orientation
    ) + c.center;
  }
  
  return { vertexes, edges, normals };
}


ConvexPolyhedron sweep(const ConvexPolygon& p, const Vec3 disp) {
  const auto size = p.vertexes.size();

  ConvexPolygon p_ = { p.vertexes };
  vector<IdxPair> edges;
  edges.reserve(3 * size);
  vector<Vec3> normals;
  normals.reserve(size + 2);
  normals.push_back(normal(p));
  normals.push_back(-normals[0]);

  for (size_t i = 0; i < size; i++) {
    p_.vertexes[i] = p_.vertexes[i] + disp;
  }

  vector<Vec3> vertexes;
  vertexes.reserve(2 * size);
  vertexes.insert(vertexes.end(), p.vertexes.begin(), p.vertexes.end());
  vertexes.insert(vertexes.end(), p_.vertexes.begin(), p_.vertexes.end());

  for (size_t i = 0; i <= 2 * size; i++) {
    size_t i_  = i + size;
    size_t i__ = ((i + 1) % size) + size;
    edges.push_back(make_pair(i, i + 1 % size));
    edges.push_back(make_pair(i_, i__));
  }

  for (size_t i = 0; i < p.vertexes.size(); i++) {
    edges.push_back(make_pair(i, i + size));
  }

  const ConvexPolyhedron ph = {
    vertexes,
    edges,
    normals
  };

  return ph;
}


// creates something that works like a polyhedron but isn't technically one
// has overlapping faces, etc
// It is fast to generate, though
// ConvexPolyhedron sweep(const ConvexPolyhedron& p, const Vec3 disp) {
//   vector<Vec3> vertexes;
//   vertexes.reserve(2 * p.vertexes.size());
//   for (size_t i = 0; i < p.vertexes.size(); i++) {
//     vertexes.push_back(p.vertexes[i]);
//   }
//   for (size_t i = 0; i < p.vertexes.size(); i++) {
//     vertexes.push_back(p.vertexes[i] + disp);
//   }
//   vector<IdxPair> edges;
//   edges.reserve(2 * p.edges.size() + )
// }


Vec3 normal(const ConvexPolygon& p) {
  Vec3 cross_sum = Vec3::zero();
  for (size_t i = 0; i <= p.vertexes.size(); i++) {
    cross_sum = cross_sum + cross(p.vertexes[i + 1 % p.vertexes.size()], p.vertexes[i]);
  }
  return normalized(cross_sum);
}


bool coplanar(const ConvexPolygon& p1, const ConvexPolygon& p2, double tolerance) {
  Vec3
    norm1 = normal(p1),
    norm2 = normal(p2);
  // if normals are different, not coplanar
  // note that if norm1 = -norm2 they would be coplanar, so we 
  if (fabs(dot(norm1, norm2)) > tolerance) return false;

  // if they're separated along either normal they're not coplanar
  // we'll give a bit of tolerance
  vector<double> proj1, proj2;
  proj1.reserve(p1.vertexes.size());
  proj2.reserve(p2.vertexes.size());

  project(norm1, p1.vertexes, proj1);
  project(norm1, p2.vertexes, proj2);

  auto
    extrema1 = extrema(proj1),
    extrema2 = extrema(proj2);

  // add a bit of tolerance, for perfectly coplanar (for example all at z = 1.0)

  return (extrema1.first + tolerance) > extrema2.second || extrema1.second < (extrema2.first + tolerance);
}


double project(const Vec3& direction, const Vec3& to_project) {
  return dot(to_project, normalized(direction));
}


void project(const Vec3& direction, const vector<Vec3>& to_project, vector<double>& dst) {
  dst.clear();
  dst.reserve(to_project.size());

  for (size_t i = 0; i < to_project.size(); i++) {
    dst[i] = project(direction, to_project[i]);
  }
}


bool separated(const vector<Vec3>& points1, const vector<Vec3>& points2, const Vec3& direction) {
  vector<double> proj1, proj2;
  proj1.reserve(points1.size());
  proj2.reserve(points2.size());

  project(direction, points1, proj1);
  project(direction, points2, proj2);

  auto
    extrema1 = extrema(proj1),
    extrema2 = extrema(proj2);
  //              min              max                max               min
  return extrema1.first > extrema2.second || extrema1.second < extrema2.first;
}


bool _intersect_polypoly_coplanar(const ConvexPolygon& poly1, const ConvexPolygon& poly2, Vec3 normal1, Vec3 normal2) {
  for (size_t i = 0; i <= poly1.vertexes.size(); i++) {
    const auto prod = cross(
      normal2,
      poly1.vertexes[i + 1 % poly1.vertexes.size()] - poly1.vertexes[i]
    );
    if (separated(poly1.vertexes, poly2.vertexes, prod)) return false;
  }
  for (size_t i = 0; i <= poly2.vertexes.size(); i++) {
    const auto prod = cross(
      normal1,
      poly2.vertexes[i + 1 % poly2.vertexes.size()] - poly2.vertexes[i]
    );
    if (separated(poly1.vertexes, poly2.vertexes, prod)) return false;
  }
  return true;
}


bool _intersect_polypoly_noncoplanar(const ConvexPolygon& poly1, const ConvexPolygon& poly2) {
  for (size_t i_p1 = 0; i_p1 <= poly1.vertexes.size(); i_p1++) {
    for (size_t i_p2 = 0; i_p2 <= poly2.vertexes.size(); i_p2++) {
      const Vec3 prod = cross(
        poly1.vertexes[i_p1 + 1 % poly1.vertexes.size()] - poly1.vertexes[i_p1],
        poly2.vertexes[i_p2 + 1 % poly2.vertexes.size()] - poly2.vertexes[i_p2]
      );

      if (separated(poly1.vertexes, poly2.vertexes, prod)) return false;
    }
  }
  return true;
}


bool intersect(const ConvexPolygon& poly1, const ConvexPolygon& poly2) {
  // first we check the normals
  auto
    normal1 = normal(poly1),
    normal2 = normal(poly2);

  if (separated(poly1.vertexes, poly2.vertexes, normal1)) return false;
  if (separated(poly1.vertexes, poly2.vertexes, normal2)) return false;

  if (coplanar(poly1, poly2)) {
    return _intersect_polypoly_coplanar(poly1, poly2, normal1, normal2);
  } else {
    return _intersect_polypoly_noncoplanar(poly1, poly2);
  }
}


bool intersect(const ConvexPolyhedron& polyh1, const ConvexPolyhedron& polyh2) {
  const auto num_axes = polyh1.normals.size() + polyh2.normals.size() + polyh1.edges.size()*polyh2.edges.size();
  if (num_axes >= NUM_AXES_FOR_BOUNDS_CHECK && !intersect(bounding_sphere(polyh1), bounding_sphere(polyh2))) {
    return false;
  }
  else if (num_axes >= NUM_AXES_FOR_PAIRWISE && (//TODO: wtf if one sphere is lager than the other then this breaks 
    !intersect(bounding_sphere(polyh1), polyh2)
    && !intersect(bounding_sphere(polyh2), polyh1)
  )) {
    return false;
  }
  
  for (size_t i = 0; i < polyh1.normals.size(); i++) {
    auto normal = polyh1.normals[i];
    if (separated(polyh1.vertexes, polyh2.vertexes, normal)){
      //std::cout << "not seapterated\n";
       return false;
    }
  }
  for (size_t i = 0; i < polyh2.normals.size(); i++) {
    auto normal = polyh2.normals[i];
    if (separated(polyh1.vertexes, polyh2.vertexes, normal)) return false;
  }
  for (size_t i = 0; i < polyh1.edges.size(); i++) {
    auto edge1 = polyh1.edges[i];
    Vec3 dir1  = (polyh1.vertexes[edge1.second] - polyh1.vertexes[edge1.first]);

    for (size_t j = 0; j < polyh2.edges.size(); j++) {
      auto edge2 = polyh2.edges[i];
      Vec3 dir2  = (polyh2.vertexes[edge2.second] - polyh2.vertexes[edge2.first]);
      
      auto dir = cross(dir1, dir2);

      if (separated(polyh1.vertexes, polyh2.vertexes, dir)) return false;
    }
  }
  return true;
}

double SignedVolume(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
  return (1.0/6.0)*dot(cross(b-a,c-a),d-a);
}

//modified version of this https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm,
bool lineIntersectTriangle(Vec3 l0,Vec3 l1,Vec3 v0,Vec3 v1,Vec3 v2,Vec3 &outIntersectionPoint)
{
  const double EPSILON = 0.0000001;
  outIntersectionPoint = Vec3(0, 0, 0);

  Vec3 rayOrigin = l0;
  Vec3 rayVector = normalized(l1-l0);

  Vec3 edge1 = v1-v0;
  Vec3 edge2 = v2-v0;
  Vec3 h =cross(rayVector, edge2);

  //crossing condidtion
  Vec3 normal = cross(edge1, edge2);
  if( (dot(l0-v0,normal)<0) == (dot(l1-v0,normal)<0) ){
    //std::cout << "Same side condidtion\n";
    return false;
  }

  double a = dot(edge1, h);
  if (a > -EPSILON && a < EPSILON)
  {
    return false;    // This ray is parallel to this triangle.
  }

  double f = 1.0 / a;      
  Vec3 s = rayOrigin-v0;

  double u = f * dot(s, h);
  if (u < 0.0 || u > 1.0)
  {
    return false;
  }
  Vec3 q = cross(s, edge1);
  double v = f * dot(rayVector, q);
  if (v < 0.0 || u + v > 1.0)
  {
    return false;
  }
  // At this stage we can compute t to find out where the intersection point is on the line.
  double t = f * dot(edge2, q);
  if (t > EPSILON && t < sqrt(dot(rayVector, rayVector))) // ray intersection
  {
    outIntersectionPoint = rayOrigin + rayVector * t;
    return true;
  }
  else // This means that there is a line intersection but not a ray intersection.
  {
    return false;
  }
}

bool intersect2(const ConvexPolyhedron& polyh1, const ConvexPolyhedron& polyh2) {
  
  
  for (size_t i = 0; i < polyh1.edges.size(); i++) {
     for (size_t j = 0; j < polyh1.edges.size(); j++) {
       Vec3 a,b,c;
       if(i==j)
        continue;
      
       a=polyh1.vertexes[std::get<0>(polyh1.edges[i])];
       b=polyh1.vertexes[std::get<1>(polyh1.edges[i])];

       if(std::get<0>(polyh1.edges[i])==std::get<0>(polyh1.edges[j]) || 
          std::get<1>(polyh1.edges[i])==std::get<0>(polyh1.edges[j])){//TODO: does this need to be like this?
         c=polyh1.vertexes[std::get<1>(polyh1.edges[j])];
       }else if(std::get<1>(polyh1.edges[i])==std::get<1>(polyh1.edges[j]) ||
                std::get<0>(polyh1.edges[i])==std::get<1>(polyh1.edges[j])){
         c=polyh1.vertexes[std::get<0>(polyh1.edges[j])];
       }else{
         continue;
       }
       
       for (size_t k = 0; k < polyh2.edges.size(); k++) {
          Vec3 d,e;
          d = polyh2.vertexes[get<0>(polyh2.edges[k])];
          e = polyh2.vertexes[get<1>(polyh2.edges[k])];
          /*bool b1 = SignedVolume(a,b,d,e) < 0;
          bool b2 = SignedVolume(b,c,d,e) < 0;
          bool b3 = SignedVolume(c,a,d,e) < 0;
          if(b1 == b2 && b2 == b3){
            return true;*/
          Vec3 out;
          if(lineIntersectTriangle(d,e,a,b,c,out))
            return true;
          
       }
     }
  }
  return false;
}


bool intersect(const ConvexPolygon& polygon, const ConvexPolyhedron& polyhedron) {
  auto n = normal(polygon);

  // first check normals

  if (separated(polyhedron.vertexes, polygon.vertexes, n)) return false;

  for (size_t i = 0; i < polyhedron.normals.size(); i++) {
    auto normal = polyhedron.normals[i];
    if (separated(polyhedron.vertexes, polygon.vertexes, normal)) return false;
  }

  // now check normals cross edges

  for (size_t i = 0; i < polyhedron.edges.size(); i++) {
    auto edge = polyhedron.edges[i];
    auto disp = polyhedron.vertexes[edge.second] - polyhedron.vertexes[edge.first];
    auto vec  = cross(n, disp);
    if (separated(polyhedron.vertexes, polygon.vertexes, vec)) return false;
  }

  for (size_t i = 0; i < polyhedron.normals.size(); i++) {
    auto normal = polyhedron.normals[i];

    for (size_t j = 0; j <= polygon.vertexes.size(); j++) {
      auto edge = polygon.vertexes[j + 1 % polygon.vertexes.size()] - polygon.vertexes[j];
      auto vec  = cross(normal, edge);
      if (separated(polyhedron.vertexes, polygon.vertexes, vec)) return false;
    }
  }

  // finally, edge cross edge
  for (size_t i = 0; i < polyhedron.edges.size(); i++) {
    auto h_edge = polyhedron.edges[i];
    auto h_disp = polyhedron.vertexes[h_edge.second] - polyhedron.vertexes[h_edge.first];
    
    for (size_t j = 0; j <= polygon.vertexes.size(); j++) {
      auto g_disp = polygon.vertexes[j + 1 % polygon.vertexes.size()] - polygon.vertexes[j];
      auto vec    = cross(h_disp, g_disp);
      if (separated(polyhedron.vertexes, polygon.vertexes, vec)) return false;
    }

  }

  return true;
}


bool intersect(const ConvexPolyhedron& polyhedron, const ConvexPolygon& polygon) {
  return intersect(polygon, polyhedron);
}



bool intersect(const Sphere& s, const ConvexPolygon& p) {
  for (size_t i = 0; i < p.vertexes.size(); i++) {
    if (norm(p.vertexes[i] - s.center) <= s.r) return true;
  }
  for (size_t i = 0; i <= p.vertexes.size(); i++) {
    LineSegment ls = {
      p.vertexes[i + 1 % p.vertexes.size()],
      p.vertexes[i]
    };
    if (intersect(ls, s)) return true;
  }

  return false;
}


bool intersect(const Sphere& s, const ConvexPolyhedron& p) {
  for (size_t i = 0; i < p.vertexes.size(); i++) {
    if (norm(p.vertexes[i] - s.center) <= s.r) return true;
  }
  for (size_t i = 0; i < p.edges.size(); i++) {
    auto edge = p.edges[i];
    LineSegment ls = {
      p.vertexes[edge.second],
      p.vertexes[edge.first]
    };
    if (intersect(ls, s)) return true;
  }

  return false;
}


bool intersect(const Sphere& s, const ConvexPolygon& p, const Vec3& disp) {
  // for the polygon, all we have to do is "sweep" it into a polyhedron along with displacement vector
  // if (3 * p.vertexes.size() >= NUM_AXES_FOR_DISP_BOUNDS) {
  //   if (!intersect(s, bounding_cylinder(p, disp))) {
  //     return false;
  //   }
  // }
  auto ph = sweep(p, disp);
  return intersect(s, ph);
}


bool intersect(const Sphere& s, const ConvexPolyhedron& p, const Vec3& disp) {
  // for this, we'll check the two "endpoint" polyhedrons for collision, then
  // iterate over the polygons created by sweeping the polyhedron's vertexes
  // I think (but am not sure) that this is faster than trying to create a convex hull
  // for the swept polyhedron 
  // We generate the swept polygons by examining each edge on the original and swept polyhedra.
  // Since the polyhedron is supposed to be convex, this (I think) will result in redundant checks but no false negatives.
  
  if (intersect(s, p)) return true;

  ConvexPolyhedron p_ = p;
  for (size_t i = 0; i < p_.vertexes.size(); i++) {
    p_.vertexes[i] = p_.vertexes[i] + disp;
  }

  if (intersect(s, p_)) return true;

  for (size_t i = 0; i < p_.edges.size(); i++) {
    auto edge = p_.edges[i];
    ConvexPolygon pg = {{
      p.vertexes[edge.first],
      p.vertexes[edge.second],
      p_.vertexes[edge.second],
      p_.vertexes[edge.first]
    }};

    if (intersect(s, pg)) return true;
  }

  return false;
}


Sphere bounding_sphere(const ConvexPolyhedron& p) {
  return bounding_sphere(p.vertexes);
}


// currently probably produces a cylinder way too big
// Cylinder bounding_cylinder(const ConvexPolygon& p, const Vec3 disp) {
//   const auto n = normal(p);
//   const auto
//     c1 = centroid(p),
//     c2 = c1 + disp;
  
//   vector<Vec3> displaced;
//   displaced.reserve(p.vertexes.size());
//   for (size_t i = 0; i < p.vertexes.size(); i++) {
//     displaced.push_back(p.vertexes[i] + disp);
//   }

//   vector<double> proj;
//   project(disp, p.vertexes, proj);
//   auto e = extrema(proj);
//   double ez = e.second;
//   project(disp, displaced, proj);
//   e = extrema(proj);

//   if (e.second > ez) ez = e.second;


// }


// Cylinder bounding_cylinder(const ConvexPolyhedron& p, const Vec3 disp);


