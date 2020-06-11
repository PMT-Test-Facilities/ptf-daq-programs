#ifndef __PATHGEN_INTERNAL
#define __PATHGEN_INTERNAL


#include "pathgen.hpp"


namespace PathGeneration {


// Represents the order in which a gantry travels a path
typedef struct DimensionOrder {
  Dimension first;
  Dimension second;
  Dimension third;
  Dimension fourth;
  Dimension fifth;

  template<typename T>
  Dimension operator[](T idx) const {
    switch (idx) {
      case 0: return this->first;
      case 1: return this->second;
      case 2: return this->third;
      case 3: return this->fourth;
      case 4: return this->fifth;
      default:
        throw new runtime_error("Out of range.");
    }
  }

  // Ensure that what the order is valid (contains each dimension once and only once)
  static boost::optional<DimensionOrder> build(Dimension first, Dimension second, Dimension third, Dimension fourth, Dimension fifth);

  // Gets every order of dimensions
  static std::vector<DimensionOrder> all_orders();
} DimensionOrder;
// variant<vector<MovePath>, flatten


ostream& operator<<(ostream& l, const DimensionOrder& r);
ostream& operator<<(ostream& l, const Point& r);
bool operator==(const Point& l, const Point& r);


Prism point_to_optical_box(const Point& p, bool gantry1);



}

#endif