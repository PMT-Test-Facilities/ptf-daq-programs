#ifndef __SERIALIZE__
#define __SERIALIZE__


#include "geom.hpp"
// #include <regex>
#include <boost/regex.hpp>
#include <algorithm>
#include <utility>
#include <boost/variant.hpp>
#include <string>
#include <iostream>
#include <ostream>
#include <sstream>
#include <iomanip>
#include "has.hpp"


using namespace std;
using namespace boost;


// number of digits to use when serializing objects
#define SERIALIZE_PREC 5


namespace Serialization {


enum ErrorType {
  NoError,
  Commented,
  SyntaxError,
  InvalidName,
  MissingProperty,
  ExtraProperty,
  DuplicateProperty,
  UnknownError,
  NoParser
};


typedef ::GeomTypes::GeomType GeomType;


// result of parsing geometry text
typedef boost::variant<Vec3, LineSegment, Quaternion, Prism, Sphere, Cylinder, ConvexPolyhedron, string, double, ErrorType> GeomResult;


// Serialization format:
// GeomType[prop:val, prop:val, ...]
// It is not whitespace or case sensitive
// Val may be another datatype
// For example, a serialized Vec3 could be:
// "Vec3[1.0, 2.0, 3.0]"
// A line segment could be:
// "LineSegment[a: Vec3[x: 1, y: 2, z: 3], b:vec3[x:0,y:0,z:0.0]]"

// To serialize a geometry object, simply call `serialize` on it.

/* To deserialize a stored string, call `deserialize` on the string.
   It will return a `GeomResult` object, which is a Boost tagged union (boost::variant) of the following types:
   - Vec3
   - LineSegment
   - Quaternion
   - Prism
   - Sphere
   - Cylinder
   - double (this is used internally and should not be returned)
   - ErrorType (in case of a parse error)
 */



GeomResult deserialize(const string& s);


string serialize(const Vec3& v);
string serialize(const LineSegment& ls);
string serialize(const Quaternion& a);
string serialize(const Prism& p);
string serialize(const Sphere& sp);
string serialize(const Cylinder& c);


// Returns an error message for the error type
string error_message(ErrorType e);

// Is this GeomResult an error type?
bool is_error(GeomResult r);


// note that these do _not_ check that the geometry type name matches
// the output type
ErrorType deserialize(const string& s, Vec3& v);
ErrorType deserialize(const string& s, LineSegment& ls);
ErrorType deserialize(const string& s, Quaternion& a);
ErrorType deserialize(const string& s, Prism& p);
ErrorType deserialize(const string& s, Sphere& sp);
ErrorType deserialize(const string& s, Cylinder& c);
ErrorType deserialize(const string& s, ConvexPolyhedron& p);


}


#endif // __SERIALIZE__
