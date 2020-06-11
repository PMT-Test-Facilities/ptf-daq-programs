#ifndef __HAS_H
#define __HAS_H

#include <boost/variant.hpp>

template <typename T, typename Ts>
bool has(Ts _variant) {
  if (boost::get<T>(&_variant)) {
    return true;
  } else {
    return false;
  }
}

#endif
