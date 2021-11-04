#include "serialization.hpp"
#include "serialization_internal.hpp"
#include "has.hpp"
#include "sat.hpp"


namespace Serialization {


/*
 * Error handling
 */


string error_message(ErrorType e) {
  switch (e) {
    case NoError:
      return "";
    
    case SyntaxError:
      return "Syntax error";

    case InvalidName:
      return "Unknown geometry type";
    
    case MissingProperty:
      return "Required property missing";

    case ExtraProperty:
      return "Extraneous property";
    
    case DuplicateProperty:
      return "Duplicate property";

    case NoParser:
      return "No parser exists for that datatype";
    
    default:
    case UnknownError:
      return "Unknown error";
  }
}


bool is_error(GeomResult& r) {
  if (has<ErrorType>(r)) {
    return true;
  } else {
    return false;
  }
}


GeomResult deserialize(const string& s) {
  // Remove whitespace and make lowercase
  static const regex ws("\\s+");
  auto reduced = regex_replace(s, ws, "");

  // check if it's commented out
  if (s.size() && (s[0] == '#' || s[0] == '!')) return Commented;

  transform(reduced.begin(), reduced.end(), reduced.begin(), ::tolower);

  GeomType geom_type;

  auto res = Internal::find_geom_type(reduced);

  if (res.second == NoError) {
    geom_type = res.first;
  } else {
    return GeomResult { res.second };
  }

  // remove the type name now
  // reduced = reduced.substr(idx);

  Vec3 v;
  LineSegment l;
  Quaternion a;
  Prism p;
  Sphere sp;
  Cylinder c;
  ConvexPolyhedron poly;

  // todo: is there a better way of doing this?
  ErrorType err;
  switch (geom_type) {
    case GeomTypes::Vec3:
      err = deserialize(reduced, v);
      if (err != NoError) return GeomResult { err };
      return GeomResult { v };

    case GeomTypes::LineSegment:
      err = deserialize(reduced, l);
      if (err != NoError) return GeomResult { err };
      return GeomResult { l };

    case GeomTypes::Quaternion:
      err = deserialize(reduced, a);
      if (err != NoError) return GeomResult { err };
      return GeomResult { a };

    case GeomTypes::Prism:
      err = deserialize(reduced, p);
      if (err != NoError) return GeomResult { err };
      return GeomResult { p };

    case GeomTypes::Sphere:
      err = deserialize(reduced, sp);
      if (err != NoError) return GeomResult { err };
      return GeomResult { sp };

    case GeomTypes::Cylinder:
      err = deserialize(reduced, c);
      if (err != NoError) return GeomResult { err };
      return GeomResult { c };

    case GeomTypes::ConvexPolyhedron:
      err = deserialize(reduced, poly);
      if (err != NoError) return GeomResult { err };
      return GeomResult { poly };

    default:
      return GeomResult { NoParser };
  }

}


// Vec3

string serialize(const Vec3& v) {
  ostringstream ret;
  ret << "Vec3[x: " << fixed << setprecision(SERIALIZE_PREC) << v.x << ", y: " << v.y << ", z: " << v.z << "]";
  return ret.str();
}


ErrorType deserialize(const string& s, Vec3& v) {
  static const unordered_map<string, GeomType> required_props({
    {"x", GeomTypes::Scalar},
    {"y", GeomTypes::Scalar},
    {"z", GeomTypes::Scalar}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }
  
  v.x = get<double>(res["x"]);
  v.y = get<double>(res["y"]);
  v.z = get<double>(res["z"]);
  
  return ErrorType::NoError;
}


// LineSegment

string serialize(const LineSegment& ls) {
  auto a = serialize(ls.a),
       b = serialize(ls.b);

  ostringstream ret;
  ret << "LineSegment[a: " << a << ", b: " << b << "]";
  return ret.str();
}


ErrorType deserialize(const string& s, LineSegment& ls) {
  static const unordered_map<string, GeomType> required_props({
    {"a", GeomTypes::Vec3},
    {"b", GeomTypes::Vec3}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  ls.a = get<Vec3>(res["a"]);
  ls.b = get<Vec3>(res["b"]);

  return ErrorType::NoError;
}


// Quaternion

string serialize(const Quaternion& q) {
  ostringstream ret;
  ret << "Quaternion[w: " << fixed << setprecision(SERIALIZE_PREC) << q.w
      << ", x: " << q.x
      << ", y: " << q.y
      << ", z: " << q.z << "]";
  return ret.str(); 
}


ErrorType deserialize(const string& s, Quaternion& a) {
  static const unordered_map<string, GeomType> required_props({
    {"w", GeomTypes::Scalar},
    {"x", GeomTypes::Scalar},
    {"y", GeomTypes::Scalar},
    {"z", GeomTypes::Scalar}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  a.w = get<double>(res["w"]);
  a.x = get<double>(res["x"]);
  a.y = get<double>(res["y"]);
  a.z = get<double>(res["z"]);

  return ErrorType::NoError;
}


// Prism

string serialize(const Prism& p) {
  auto vec = serialize(p.center),
       q   = serialize(p.orientation);

  ostringstream ret;
  ret << "Prism[center: " << vec << ", ";
  ret << "ex: " << fixed << setprecision(SERIALIZE_PREC) << p.ex << ", ey: " << p.ey << ", ez: " << p.ez << ", ";
  ret << "orientation: " << q << "]";

  return ret.str();
}


ErrorType deserialize(const string& s, Prism& p) {
  static const unordered_map<string, GeomType> required_props({
    {"center", GeomTypes::Vec3},
    {"ex",     GeomTypes::Scalar},
    {"ey",     GeomTypes::Scalar},
    {"ez",     GeomTypes::Scalar},
    {"orientation", GeomTypes::Quaternion}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  p.center = get<Vec3>(res["center"]);
  p.ex     = get<double>(res["ex"]);
  p.ey     = get<double>(res["ey"]);
  p.ez     = get<double>(res["ez"]);

  p.orientation = get<Quaternion>(res["orientation"]);

  return ErrorType::NoError;
}


// Sphere

string serialize(const Sphere& sp) {
  auto vec = serialize(sp.center);

  ostringstream ret;
  ret << "Sphere[center: " << vec << ", r: " << setprecision(SERIALIZE_PREC) << sp.r << "]";

  return ret.str();
}


ErrorType deserialize(const string& s, Sphere& sp) {
  static const unordered_map<string, GeomType> required_props({
    {"center", GeomTypes::Vec3},
    {"r",      GeomTypes::Scalar}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  sp.center = get<Vec3>(res["center"]);
  sp.r      = get<double>(res["r"]);

  return ErrorType::NoError;
}


// Cylinder

string serialize(const Cylinder& c) {
  auto vec = serialize(c.center),
       q   = serialize(c.orientation);
  
  ostringstream ret;
  ret << "Cylinder[center: " << fixed << setprecision(SERIALIZE_PREC) << vec << ", r: " << c.r << ", e: " << c.e << ", orientation: " << q << "]";

  return ret.str();
}


ErrorType deserialize(const string& s, Cylinder& c) {
  static const unordered_map<string, GeomType> required_props({
    {"center", GeomTypes::Vec3},
    {"r",      GeomTypes::Scalar},
    {"e",      GeomTypes::Scalar},
    {"orientation", GeomTypes::Quaternion}
  });

  auto res = Internal::parse_geometry(s, required_props);

  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  c.center = get<Vec3>(res["center"]);
  c.r      = get<double>(res["r"]);
  c.e      = get<double>(res["e"]);

  c.orientation = get<Quaternion>(res["orientation"]);

  return ErrorType::NoError;
}

ErrorType deserialize(const string& s, ConvexPolyhedron& p) {
  static const unordered_map<string, GeomType> required_props({
    {"file", GeomTypes::String},
    {"center", GeomTypes::Vec3},
    {"sx",      GeomTypes::Scalar},
    {"sy",      GeomTypes::Scalar},
    {"sz",      GeomTypes::Scalar},
    {"orientation", GeomTypes::Quaternion}
  });
  
  auto res = Internal::parse_geometry(s, required_props);
  
  if (res.find("*") != res.end()) {
    return get<ErrorType>(res["*"]);
  }

  string file_name = get<string>(res["file"]);
  std::cout << file_name <<" here\n";
  Vec3 scale_vec(get<double>(res["sx"]),get<double>(res["sy"]),get<double>(res["sz"]));
  Vec3 center = get<Vec3>(res["center"]);
  Quaternion orientation = get<Quaternion>(res["orientation"]);

  ColStlMesh mesh(file_name);
  ConvexPolyhedron p1 = polyhedron(mesh);
  p = translate( rotate( scale(p1,scale_vec), orientation ) , center);

  return ErrorType::NoError;
}

}
