#include "serialization.hpp"
#include "serialization_internal.hpp"


namespace Serialization { namespace Internal {


pair<int,int> find_data_bounds(const string& outer, uint32_t start) {
  int end = -1,
      paren_count = 1;

  while (start < outer.length() && outer[start] != '[') {
    if (!isalnum(outer[start])) {
      // cerr << "Not alnum at " << start << endl;
      return make_pair(-1, -1);
    }
    start++;
  }
  if (start == outer.length()) {
    // cerr << "String doesn't have opening bracket" << endl;
    return make_pair(-1, -1);
  }

  for (uint32_t i = start+1; i < outer.length(); i++) {
    if (outer[i] == '[')
      paren_count++;
    else if (outer[i] == ']')
      paren_count--;
    
    if (paren_count == 0) {
      end = i;
      break;
    }
  }
  
  if (end == -1) {
    // cerr << "Never reached paren 0 count" << endl;
    return make_pair(-1, -1);
  } else {
    return make_pair(start, end);
  }
}


pair<GeomType, ErrorType> find_geom_type(const string& s, uint32_t idx) {
  uint32_t start = idx;

  // cerr << "Searching string '" << s << "' for geom type." << endl;

  while (s[idx] != '[' && idx < s.length()) idx++;
  if (idx == s.length()) {
    // cerr << "Reched end of string, none found" << endl;
    return make_pair(GeomTypes::Invalid, ErrorType::SyntaxError);
  }

  auto geom_name = s.substr(start, idx-start);
  auto res = GeomTypes::GEOM_MAP.find(geom_name);

  if (res == GeomTypes::GEOM_MAP.end()) {
    // cerr << "Name '" << geomName << "' not found in map." << endl;
    return make_pair(GeomTypes::Invalid, ErrorType::InvalidName);
  }
  else {
    // cerr << "Found type " << geomName << endl;
    return make_pair(res->second, ErrorType::NoError);
  }
}


pair<int, string> find_next_pair(const string& s, uint32_t start, uint32_t max) {
  string kv;
  uint32_t i = start;
  if (s[start] == ']') {
    return make_pair(-2, "");
  }
  while (i < s.length() && i < max && s[i] != ':') {
    kv.push_back(s[i]);
    i++;
    if (i == ']') {
      return make_pair(-1, "");
    }
  }
  if (i == max) {
    return make_pair(-2, "");
  }
  else if (i == s.length()) {
    return make_pair(-1, "");
  }
  kv.push_back(':');
  i++; // i now points to the start of the second item
  
  int paren_count = 0;
  while (i < s.length() && i < max) {
    if (s[i] == '[')
      paren_count++;
    else if (s[i] == ']')
      paren_count--;

    if (paren_count == -1) // we've reached the end of the parent data string
      break;
    else if (paren_count == 0 && s[i] == ',')
      break;

    kv.push_back(s[i]);
    i++;
  }

  if (i == max || i == s.length()) {
    return make_pair(-1, "");
  }

  return make_pair(static_cast<int>(i), kv);
}


pair<unordered_map<string, string>, bool> find_pairs(const string& s) {
  unordered_map<string, string> map;

  auto bounds = find_data_bounds(s);
  if (bounds.first == -1) return make_pair(map, true);
  
  uint32_t i = bounds.first + 1;
  while (i < s.length()) {
    auto pair = find_next_pair(s, i, s.length());

    // cerr << "Got pair <" << pair.first << ", " << pair.second << ">" << endl;

    if (pair.first == -1) {
      // error!
      return make_pair(map, true);
    }
    else if (pair.first == -2) {
      break;
    }

    auto str = pair.second;
    auto idx = str.find(':');

    if (idx < 0) {
      return make_pair(map, true);
    }
    
    auto str1 = str.substr(0, idx), str2 = str.substr(idx+1);
    // cerr << "The strings are: '" << str1 << "', '" << str2 << "'." << endl;
    map[str1] = str2;

    i = pair.first + 1;
    if (i >= s.length()) {
      break;
    }
  }

  return make_pair(map, false);
}


pair<int, double> parse_double(string& s, uint32_t start) {
  bool seen_decimal = false;
  uint32_t i = start;
  if (s[i] == '-') i++;
  while (i < s.length()) {
    if (s[i] == '.') {
      if (seen_decimal) {
        return make_pair(-1, 0);
      } else {
        seen_decimal = true;
        i++;
        continue;
      }
    }
    else if (s[i] == '-' && i > start) {
      return make_pair(-1, 0);
    }
    else if (!isdigit(s[i])) { // float has ended
      // i++;
      break;
    }

    i++;
  }
  // if (i > 0) i--;
  auto numString = s.substr(start, i-start);
  try {
    return make_pair(i-1, stod(numString));
  } catch (const invalid_argument& arg) {
    return make_pair(-1, 0);
  }
}


unordered_map<string, GeomResult> parse_geometry(const string& s, const unordered_map<string, GeomType>& required_props) {
  unordered_map<string, GeomResult> ret;
  auto bounds = find_data_bounds(s);

  if (bounds.first == -1) {
    ret["*"] = SyntaxError;
    return ret;
  }

  auto prop_res = find_pairs(s.substr(bounds.first, bounds.second - bounds.first + 1));

  unordered_map<string, bool> prop_seen;

  for (auto it = required_props.begin(); it != required_props.end(); ++it) {
    prop_seen[it->first] = false;
  }

  if (prop_res.second) {
    ret["*"] = UnknownError;
    return ret;
  }

  auto props = prop_res.first;

  for (auto prop = props.begin(); prop != props.end(); ++prop) {
    auto prop_name = prop->first;
    if (required_props.find(prop_name) != required_props.end()) {
      if (prop_seen[prop_name]) {
        // we've already seen this property!
        ret["*"] = DuplicateProperty;
        return ret;
      } else {
        prop_seen[prop_name] = true;
      }
    } else {
      // property is not required
      ret["*"] = ExtraProperty;
      return ret;
    }
  }

  for (auto prop = required_props.begin(); prop != required_props.end(); ++prop) {
    string prop_name = prop->first;

    if (!prop_seen[prop_name]) {
      cerr << "Missing property " << prop_name << endl;
      ret["*"] = MissingProperty;
      return ret;
    }

    auto geom_type = prop->second;
    auto str = props[prop->first];

    ErrorType err;
    pair<int, double> res;

    Vec3 v;
    LineSegment l;
    Quaternion q;
    Prism p;
    Sphere s;
    Cylinder c;

    switch (geom_type) {
      case GeomTypes::Vec3:
        err = deserialize(str, v);
        if (err == NoError) {
          ret[prop_name] = v;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::LineSegment:
        err = deserialize(str, l);
        if (err == NoError) {
          ret[prop_name] = l;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::Quaternion:
        err = deserialize(str, q);
        if (err == NoError) {
          ret[prop_name] = q;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::Prism:
        err = deserialize(str, p);
        if (err == NoError) {
          ret[prop_name] = p;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::Sphere:
        err = deserialize(str, s);
        if (err == NoError) {
          ret[prop_name] = s;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::Cylinder:
        err = deserialize(str, c);
        if (err == NoError) {
          ret[prop_name] = c;
          break;
        } else {
          ret["*"] = err;
          return ret;
        }

      case GeomTypes::Scalar:
        res = parse_double(str, 0);
        if (res.first != -1) {
          ret[prop_name] = res.second;
          break;
        } else {
          ret["*"] = SyntaxError;
          return ret;
        }

      default:
        ret["*"] = NoParser;
        return ret;
    }
  }

  return ret;
}


} } // end namespace Internal, Serialization