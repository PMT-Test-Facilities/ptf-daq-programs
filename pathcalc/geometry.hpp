// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2011 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2011 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2011 Mateusz Loskot, London, UK.

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_GEOMETRY_HPP
#define BOOST_GEOMETRY_GEOMETRY_HPP

// Shortcut to include all header files


#include </home1/midptf/boost_1_47_0/boost/geometry/core/cs.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/tag.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/tag_cast.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/tags.hpp>

// Core algorithms
#include </home1/midptf/boost_1_47_0/boost/geometry/core/access.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/exterior_ring.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/interior_rings.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/radian_access.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/core/topological_dimension.hpp>


#include </home1/midptf/boost_1_47_0/boost/geometry/arithmetic/arithmetic.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/arithmetic/dot_product.hpp>

#include </home1/midptf/boost_1_47_0/boost/geometry/strategies/strategies.hpp>

#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/append.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/area.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/assign.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/buffer.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/centroid.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/clear.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/convex_hull.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/correct.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/comparable_distance.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/difference.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/distance.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/envelope.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/for_each.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/intersection.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/intersects.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/length.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/make.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/num_geometries.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/num_interior_rings.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/num_points.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/overlaps.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/perimeter.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/reverse.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/simplify.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/sym_difference.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/transform.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/union.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/unique.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/algorithms/within.hpp>

// Include multi a.o. because it can give weird effects
// if you don't (e.g. area=0 of a multipolygon)
#include </home1/midptf/boost_1_47_0/boost/geometry/multi/multi.hpp>

// check includes all concepts
#include </home1/midptf/boost_1_47_0/boost/geometry/geometries/concepts/check.hpp>

#include </home1/midptf/boost_1_47_0/boost/geometry/util/for_each_coordinate.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/util/math.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/util/select_most_precise.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/util/select_coordinate_type.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/util/write_dsv.hpp>

#include </home1/midptf/boost_1_47_0/boost/geometry/views/box_view.hpp>
#include </home1/midptf/boost_1_47_0/boost/geometry/views/segment_view.hpp>

#include </home1/midptf/boost_1_47_0/boost/geometry/domains/gis/io/wkt/wkt.hpp>


#endif // BOOST_GEOMETRY_GEOMETRY_HPP
