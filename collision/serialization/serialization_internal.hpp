#ifndef __SERIALIZE_INTERNAL__
#define __SERIALIZE_INTERNAL__


#include "serialization.hpp"


// Functions used internally, not intended to be part of the public interface
namespace Serialization { namespace Internal {
  // Finds the bounds of data (the information in the next pair of square braces).
  // Returns the pair <start, end> on success, <-1, -1> on failure
  pair<int,int> find_data_bounds(const string& outer, uint32_t start = 0);

  // Finds the next geom type in the string, if one exists
  pair<GeomType, ErrorType> find_geom_type(const string& s, uint32_t idx = 0);

  // Finds the next key:value pair in the string.
  // Returns <last character index of pair, key:value>
  // On error, returns <-1, "">
  // If no next pair exists but no syntax error is present, returns <-2, "">
  pair<int, string> find_next_pair(const string& s, uint32_t start = 0, uint32_t max = UINT32_MAX);

  // Finds all property:value pairs inside a data string
  // Data string should be of the format "[prop:val, prop:val, ...]"
  // Will only match valid strings using the regex, so no warnings or errors are provided in the case that there are incorrect k:v pairs
  // Returns pair of map and was_error
  pair<unordered_map<string, string>, bool> find_pairs(const string& s);

  // Parse a string to the next valid float
  // Returns pair <index of last char in float, returned value> if it works,
  // <-1, {undefined}> if not
  pair<int, double> parse_double(string& s, uint32_t start = 0);

  // Parses geometry items
  // The required_props argument should be a map from the name of the k:v pair to the desired geometry type 
  // If no error, will return all of the requested properties with the requested type, so it's ok to assume they're the right type for casting etc
  // In case of error, returns the string "*" mapped to GeomResult containing error
  // todo: refactor this error handling into something better
  unordered_map<string, GeomResult> parse_geometry(const string& s, const unordered_map<string, GeomType>& required_props);
} }


#endif //__SERIALIZE_INTERNAL__
