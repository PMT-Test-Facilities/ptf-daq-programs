#include <cstdio>
#include <string>
#include <iostream>
#include <ostream>
#include <cstdint>
#include <cstring>
#include <random>
#include "boost/variant.hpp"

#define BOOST_TEST_MODULE Geometry Tests
// #define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/included/unit_test.hpp>

#include "geom.hpp"
#include "serialization.hpp"
#include "serialization_internal.hpp"
#include "has.hpp"
#include "pathgen.hpp"


namespace PG = PathGeneration;
namespace SD = Serialization;


#define _TOL *ut::tolerance(1e-9)


namespace ut = boost::unit_test;

// entry point
// int main(int argc, char* argv[], char* envp[]) {
//   return boost::unit_test::unit_test_main( &init_unit_test, argc, argv );
// }


/*
 * Geometry Tests
 */


BOOST_AUTO_TEST_SUITE(geometry);


BOOST_AUTO_TEST_CASE(testRotatePoint) {
  Vec3  centre   = {0.0, 0.0, 0.0};
  Vec3  toRotate = {1.0, 0.0, 0.0};
  Vec3  shouldBe = {0.0, 0.0, 1.0};
  Vec3  done;

  Quaternion angle = Quaternion::from_spherical_angle(0.0, -PI/2);

  done = rotate_point(toRotate, centre, angle);

  BOOST_TEST(done.x == shouldBe.x);
  BOOST_TEST(done.y == shouldBe.y);
  BOOST_TEST(done.z == shouldBe.z);

  centre   = {0.0, 0.0, -1.0};
  shouldBe = {0.7071067811865476, 0.7071067811865476, 1};
  toRotate = {1.0, 0.0, 1.0};
  angle    = Quaternion::from_spherical_angle(PI/4, 0.0);

  done = rotate_point(toRotate, centre, angle);

  BOOST_TEST(done.x == shouldBe.x);
  BOOST_TEST(done.y == shouldBe.y);
  BOOST_TEST(done.z == shouldBe.z);

  centre   = { 0.0,  0.0,  0.0};
  shouldBe = {-0.5, -0.5, -0.5};
  toRotate = {-0.5, -0.5, -0.5};
  angle    = Quaternion::from_spherical_angle(0, 0);

  done = rotate_point(toRotate, centre, angle);

  BOOST_TEST(done.x == shouldBe.x);
  BOOST_TEST(done.y == shouldBe.y);
  BOOST_TEST(done.z == shouldBe.z);
}


BOOST_AUTO_TEST_CASE(testRotationDoesNotChangeMagnitude) {
  static const Vec3 centre = {0,0,0};

  Vec3 points[5] = {
    {1,2,3},
    {-1,2,-3},
    {-1,-1,-1},
    {PI,-PI/2,2*PI},
    {0, 0, 0}
  };

  Quaternion angles[5] = {
    Quaternion::from_spherical_angle(0, 0),
    Quaternion::from_spherical_angle(PI, PI),
    Quaternion::from_spherical_angle(PI/2,-PI/2),
    Quaternion::from_spherical_angle(-PI/4,-PI/4),
    Quaternion::from_spherical_angle(-3*PI/4,5*PI/6)
  };

  Vec3 rotated[5][5];

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      rotated[i][j] = rotate_point(points[i], centre, angles[j]);
      BOOST_TEST(norm(rotated[i][j]) == norm(points[i]));
    }
  }
}


BOOST_AUTO_TEST_CASE(testAntiRot) {
  static const int
    nPoints = 1,
    nTests  = 1;

  Vec3
    centre,
    toRotate[nPoints],
    rotated[nPoints];

  Quaternion q;
  std::mt19937 engine;

  engine.seed(12345);

  auto gen_vec = [&]() -> Vec3 {
    return (Vec3) {
      ((double) engine()) / ((double) UINT32_MAX) * 20 - 10,
      ((double) engine()) / ((double) UINT32_MAX) * 20 - 10,
      ((double) engine()) / ((double) UINT32_MAX) * 20 - 10
    };
  };

  auto gen_quaternion = [&]() -> Quaternion {
    return Quaternion::from_spherical_angle(
					    ((double) engine()) / ((double) UINT32_MAX) * 2*PI - PI,
					    ((double) engine()) / ((double) UINT32_MAX) * PI
					    );
  };

  for (int i = 0; i < nTests; i++) {
    for (int j = 0; j < nPoints; j++)
      toRotate[i] = gen_vec();

    centre = gen_vec();
    q      = gen_quaternion();
    // printf("Generated test case %i:\n", i);
    // printf("  rotate about {%2.3f, %2.3f, %2.3f}\n", centre.x, centre.y, centre.z);
    // printf("  by angle {%1.3f, %1.3f}\n", angle.theta, angle.phi);

    for (int j = 0; j < nPoints; j++) {
      rotated[j] = rotate_point(toRotate[j], centre, q);
      rotated[j] = rotate_point(rotated[j],  centre, inverse(q));
    }

    for (int j = 0; j < nPoints; j++) {
      BOOST_TEST(toRotate[j].x == rotated[j].x);
      BOOST_TEST(toRotate[j].y == rotated[j].y);
      BOOST_TEST(toRotate[j].z == rotated[j].z);
    }
  }
}


BOOST_AUTO_TEST_CASE(testPrismPoints) {
  Prism p = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    Quaternion::from_spherical_angle(0.0, 0.0)
  };

  Vec3 exp[8] = {
    (Vec3) { 0.5,  0.5,  0.5},
    (Vec3) {-0.5,  0.5,  0.5},
    (Vec3) {-0.5, -0.5,  0.5},
    (Vec3) { 0.5, -0.5,  0.5},
    (Vec3) { 0.5,  0.5, -0.5},
    (Vec3) {-0.5,  0.5, -0.5},
    (Vec3) {-0.5, -0.5, -0.5},
    (Vec3) { 0.5, -0.5, -0.5},
  };

  auto pts = p.vertexes();

  for (int i = 0; i < 8; i++) {
    BOOST_TEST(pts[i].x == exp[i].x);
    BOOST_TEST(pts[i].y == exp[i].y);
    BOOST_TEST(pts[i].z == exp[i].z);
  }

  exp[0] = Vec3(1+0.5, 1+0.5, 1+0.5);
  exp[1] = Vec3(1-0.5, 1+0.5, 1+0.5);
  exp[2] = Vec3(1-0.5, 1-0.5, 1+0.5);
  exp[3] = Vec3(1+0.5, 1-0.5, 1+0.5);
  exp[4] = Vec3(1+0.5, 1+0.5, 1-0.5);
  exp[5] = Vec3(1-0.5, 1+0.5, 1-0.5);
  exp[6] = Vec3(1-0.5, 1-0.5, 1-0.5);
  exp[7] = Vec3(1+0.5, 1-0.5, 1-0.5);

  p = {
    Vec3(1.0, 1.0, 1.0),
    0.5, 0.5, 0.5,
    Quaternion::from_spherical_angle(0.0, 0.0)
  };

  pts = p.vertexes();

  for (int i = 0; i < 8; i++) {
    BOOST_TEST(pts[i].x == exp[i].x);
    BOOST_TEST(pts[i].y == exp[i].y);
    BOOST_TEST(pts[i].z == exp[i].z);
  }
}


// BOOST_AUTO_TEST_CASE(testPrismLineSegments) {
//   // Prism p = {
//     // Vec3(0.0, 0.0, 0.0),
//     // 0.5, 0.5, 0.5,
//     // Quaternion::from_spherical_angle(0.0, 0.0)
//   // };
  
//   // auto ls = p.edges();

// //  for (int i = 0; i < 12; i++) {
// //    printf("% .1f, % .1f, % .1f, % .1f, % .1f, % .1f\n", ls[i].a.x, ls[i].a.y, ls[i].a.z, ls[i].b.x, ls[i].b.y, ls[i].b.z);
// //  }

//   BOOST_TEST(1 == 1);
// }


BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE(intersections);


BOOST_AUTO_TEST_CASE(testPrismNormals) {
  Prism p = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    Quaternion::from_spherical_angle(0.0, 0.0)
  };
  
  auto ns = p.normals();

  Vec3 shouldBe[6] = {
    Vec3::basis_x(),
    Vec3::basis_y(),
    Vec3::basis_z(),
    -Vec3::basis_x(),
    -Vec3::basis_y(),
    -Vec3::basis_z()
  };

  for (int i = 0; i < 6; i++) {
    BOOST_TEST(shouldBe[i].x == ns[i].x);
    BOOST_TEST(shouldBe[i].y == ns[i].y);
    BOOST_TEST(shouldBe[i].z == ns[i].z);
  }
}


/*
 *  Point + Prism intersection
 */


BOOST_AUTO_TEST_CASE(testPointPrismIntersection) {
  Vec3 a = {0.0, 0.0, 0.0};
  Prism b = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  BOOST_TEST(intersect(a, b));
}


BOOST_AUTO_TEST_CASE(testPointPrismNoIntersection) {
  Vec3 a = {0.0, 0.0, 1.0};
  Prism b = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  BOOST_TEST(!intersect(a, b));
}


BOOST_AUTO_TEST_CASE(testPointPrismRotIntersection) {
  Vec3 a = {4.0, 0.0, 0.0};
  Prism b = {
    Vec3(0.0, 0.0, 0.0),
    0.1, 10.0, 0.5,
    PI/2, 0.0
  };
  BOOST_TEST(intersect(a, b));
}


/*
 * Prism + Sphere intersection
 */


BOOST_AUTO_TEST_CASE(testPrismSphereSurfaceIntersection) {
  Prism p = {
    Vec3(1.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };

  Sphere s = {
    {0.0, 0.0, 0.0},
    1.0,
  };

  BOOST_TEST(intersect(p, s));
}


BOOST_AUTO_TEST_CASE(testPrismSphereNoIntersection) {
  Prism p = {
    Vec3(10.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };

  Sphere s = {
    {0.0, 0.0, 0.0},
    1.0,
  };

  BOOST_TEST(!intersect(p, s));
}


BOOST_AUTO_TEST_CASE(testPrismSphereInternal) {
  Prism p = {
    Vec3(0.0, 0.0, 0.0),
    0.1, 0.1, 0.1,
    0.0, 0.0
  };

  Sphere s = {
    {0.0, 0.0, 0.0},
    1.0,
  };

  BOOST_TEST(intersect(p, s));
}


/*
 * Prism + Sphere intersection
 */


BOOST_AUTO_TEST_CASE(testPrismPrismNoIntersection) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Prism y = {
    Vec3(1.1, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };

  BOOST_TEST(!intersect(x, y));
}


BOOST_AUTO_TEST_CASE(testPrismPrismNoBoundingIntersection) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Prism y = {
    Vec3(5.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };

  BOOST_TEST(!intersect(x, y));
}


BOOST_AUTO_TEST_CASE(testPrismPrismSimpleIntersection) {
  // bounding spheres will intersect
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Prism y = {
    Vec3(0.5, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };

  BOOST_TEST(intersect(x, y));
}


BOOST_AUTO_TEST_CASE(testPrismPrismInternalIntersection) {
  // bounding spheres will _not_ intersect
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Prism y = {
    Vec3(0.0, 0.0, 0.0),
    0.1, 0.1, 0.1,
    0.0, PI/2
  };

  BOOST_TEST(intersect(x, y));
}


BOOST_AUTO_TEST_CASE(testPrismPrismRotatedIntersection) {
  // bounding spheres will intersect
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    -PI/3, 0.0
  };
  Prism y = {
    Vec3(0.25, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, PI/6
  };

  BOOST_TEST(intersect(x, y));
}


BOOST_AUTO_TEST_CASE(testPrismPrismRotatedNoIntersection) {
  // bounding spheres will intersect
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    PI/3, 0.0
  };
  Prism y = {
    Vec3(1.5, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, -PI/6
  };

  BOOST_TEST(!intersect(x, y));
}


/*
 *  Moving prism + sphere
 */


BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionRegion0) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Sphere s = { Vec3(0.0, 0.0, 0.0), 1 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(intersect(x, s, disp));
}


BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionRegion1) {
  Prism x = {
    Vec3(1.0, 0.0, 0.0),
    0.01, 0.01, 0.01,
    0.0, 0.0
  };
  Sphere s = { Vec3(0.0, 0.0, 0.0), 1 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(intersect(x, s, disp));
}


BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionRegion2) {
  Prism x = {
    Vec3(0.0, 1.0, 0.0),
    0.01, 0.01, 0.01,
    0.0, 0.0
  };
  Sphere s = { Vec3(0.0, 0.0, 0.0), 1 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(intersect(x, s, disp));
}

// regions 3, 4, 14: XY Edge
BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionXYEdgeVelXY) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1.02, 1.02, 0), .02 };
  Vec3 disp = 0.01 * Vec3(1, 1, 0);

  BOOST_TEST(intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionXYEdgeVelZ) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1, 1, 2), .5 };
  Vec3 disp = 2 * Vec3::basis_z();

  BOOST_TEST(intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereNoIntersectionXYEdgeVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1.02, 1.02, 0), .01 };
  Vec3 disp = 0.01 * Vec3(-1, -1, 0);

  BOOST_TEST(!intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionXYEdgeNoVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1.02, 1.02, 0), .02 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(!intersect(x, s, disp));
}

// regions 5, 15: Z face
BOOST_AUTO_TEST_CASE(testMovingPrismSphereNoIntersectionZFaceNoVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(0, 0, 2), .4 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(!intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionZFaceNoVelSinglePoint) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(0, 0, 2), 1 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereIntersectionZFaceVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(0, 0, 2), 1 };
  Vec3 disp = 3 * Vec3::basis_z();

  BOOST_TEST(intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereNoIntersectionZFaceVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(0, 0, 2), 0.9 };
  Vec3 disp = -Vec3::basis_z();

  BOOST_TEST(!intersect(x, s, disp));
}

// regions 6, 7, 16: XZ edge
BOOST_AUTO_TEST_CASE(testMovingPrismSphereXZEdgeOverlap) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1.125, 0, 1.125), 0.25 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereXZEdgeSeparatedNoIntersectionNoVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1, 2, 1), 0.25 };
  Vec3 disp = Vec3::zero();

  BOOST_TEST(!intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereXZEdgeSeparatedNoIntersectionVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1, 2, 1), 0.25 };
  Vec3 disp = -Vec3::basis_y();

  BOOST_TEST(!intersect(x, s, disp));
}
BOOST_AUTO_TEST_CASE(testMovingPrismSphereXZEdgeSeparatedIntersectionVel) {
  Prism x = {
    Vec3(0.0, 0.0, 0.0),
    1, 1, 1,
    0.0, 0.0
  };
  Sphere s = { Vec3(1, 2, 1), 0.25 };
  Vec3 disp = 2 * Vec3::basis_y();

  BOOST_TEST(intersect(x, s, disp));
}


/*
 * Rotating prism + sphere intersection
 */


BOOST_AUTO_TEST_CASE(testRotatingPrismSphereNoIntersect) {
  Prism x = {
    Vec3(2, 2, 2),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.25 };
  Quaternion rot = Quaternion::from_spherical_angle(PI/2, PI/4);

  BOOST_TEST(!intersect(x, s, rot, x.center));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereBoundsIntersect) {
  Prism x = {
    Vec3(0.5, 0.5, 0.5),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.25 };
  Quaternion rot = Quaternion::from_spherical_angle(PI/2, PI/4);

  BOOST_TEST(intersect(x, s, rot, Vec3(1, 1, 1)));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereInitialIntersect) {
  Prism x = {
    Vec3(0.5, 0.5, 0.5),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI/2, PI/4);

  BOOST_TEST(intersect(x, s, rot, Vec3(1, 1, 1)));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationNoIntersect) {
  Prism x = {
    Vec3(0.0, 0.0, 1),
    1, 1, 0.25,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI, 0);
  Vec3 about = Vec3(0, 0, 1);

  BOOST_TEST(!intersect(x, s, rot, about));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationIntersectNoInitial) {
  Prism x = {
    Vec3(0.0, 0.0, 1),
    1, 1, 0.25,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(0, PI);
  Vec3 about = Vec3(0, 0, 1);

  BOOST_TEST(intersect(x, s, rot, about));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationIntersectSwingNoInitial) {
  Prism x = {
    Vec3(0, 0.7071067811865476, 0),
    0.25, 0.25, 0.25,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI, 0);
  Vec3 about = Vec3(2, 0, 0);

  BOOST_TEST(intersect(x, s, rot, about));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationIntersectSpinNoInitial) {
  Prism x = {
    Vec3(0, 0.7071067811865476, 0),
    0.25, 0.25, 0.25,
    0.0, 0.0
  };
  Sphere s = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI, 0)
    * Quaternion::from_spherical_angle(PI, 0)
    * Quaternion::from_spherical_angle(PI, 0);
  Vec3 about = Vec3(2, 0, 0);

  BOOST_TEST(intersect(x, s, rot, about));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationNoIntersectRotAbout) {
  Prism x = {
    Vec3(1.1, 0, 0),
    0.5, 2, 2,
    0.0, 0.0
  };
  Sphere s       = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI, 0);
  Vec3 about     = Vec3::zero();

  BOOST_TEST(!intersect(x, s, rot, about));
}
BOOST_AUTO_TEST_CASE(testRotatingPrismSphereRotationBarelyIntersect) {
  Prism x = {
    Vec3(1.1, 0, 0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Sphere s       = { Vec3::zero(), 0.5 };
  Quaternion rot = Quaternion::from_spherical_angle(PI/4, 0);
  Vec3 about     = x.center;

  BOOST_TEST(intersect(x, s, rot, about));
}


/*
 * Prism + Cylinder intersection
 */


BOOST_AUTO_TEST_CASE(testPrismCylinderNoIntersection) {
  Prism p = {
    Vec3(2.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    0.0, 0.0
  };
  Cylinder c = {
    Vec3(0.0, 0.0, 0.0),
    1.0, 2.0,
    Quaternion::identity()
  };

  BOOST_TEST(!intersect(p, c));
}


BOOST_AUTO_TEST_CASE(testPrismCylinderNoIntersectionRot) {
  Prism p = {
    Vec3(0.0, 2.0, 0.0),
    0.5, 0.5, 0.5,
    Quaternion::identity()
  };
  Cylinder c = {
    {0.0, 0.0, 0.0},
    1.0, 2.0,
    Quaternion::from_spherical_angle(0.0, PI/2),
  };

  BOOST_TEST(!intersect(p, c));
}


BOOST_AUTO_TEST_CASE(testPrismCylinderNoIntersectionInternal) {
  Prism p = {
    Vec3(0.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    Quaternion::identity()
  };
  Cylinder c = {
    {0.0, 0.0, 0.0},
    1.0, 2.0,
    Quaternion::identity()
  };

  BOOST_TEST(!intersect(p, c));
}


BOOST_AUTO_TEST_CASE(testPrismCylinderNoIntersectionExceedExtent) {
  Prism p = {
    Vec3(0.0, 0.0, 3.0),
    1.5, 1.5, 1.5,
    Quaternion::identity()
  };
  Cylinder c = {
    Vec3(0.0, 0.0, 0.0),
    1.0, 2.0,
    Quaternion::identity()
  };

  BOOST_TEST(!intersect(p, c));
}


BOOST_AUTO_TEST_CASE(testPrismCylinderIntersectionSimple) {
  Prism p = {
    Vec3(1.0, 0.0, 0.0),
    0.5, 0.5, 0.5,
    Quaternion::identity()
  };
  Cylinder c = {
    {0.0, 0.0, 0.0},
    1.0, 2.0,
    Quaternion::identity()
  };

  BOOST_TEST(intersect(p, c));
}


// This test will fail. If full collision detection between cylinders and prisms is implemented, uncomment this test.
// BOOST_AUTO_TEST_CASE(testPrismCylinderIntersectionLarge) {
//   Prism p = {
//     Vec3(0.0, 0.0, 0.0),
//     1.5, 1.5, 1.5,
//     Quaternion::identity()
//   };
//   Cylinder c = {
//     {0.0, 0.0, 0.0},
//     1.0, 2.0,
//     Quaternion::identity()
//   };
//
//   BOOST_TEST(intersect(p, c));
// }


BOOST_AUTO_TEST_CASE(testPrismCylinderIntersectionRot) {
  Prism p = {
    Vec3(0.0, 0.0, 1.0),
    0.5, 0.5, 0.5,
    Quaternion::identity()
  };
  Cylinder c = {
    {0.0, 0.0, 0.0},
    1.0, 2.0,
    Quaternion::from_spherical_angle(0.0, PI/2)
  };

  BOOST_TEST(intersect(p, c));
}


/*
***********************
* Serialization Tests *
***********************
*/


BOOST_AUTO_TEST_SUITE_END();
BOOST_AUTO_TEST_SUITE(serialization);


// Find data bounds

BOOST_AUTO_TEST_SUITE(find_data_bounds);

BOOST_AUTO_TEST_CASE(find_data_boundsFailure) {
  string str = "NoDataExistsHere";
  auto res = SD::Internal::find_data_bounds(str);

  BOOST_TEST(res.first == -1);
  BOOST_TEST(res.second == -1);
}


BOOST_AUTO_TEST_CASE(find_data_boundsFailureCloseBracketNoOpen) {
  string str = "NoDataExistsHere]";
  auto res = SD::Internal::find_data_bounds(str);

  BOOST_TEST(res.first == -1);
  BOOST_TEST(res.second == -1);
}


BOOST_AUTO_TEST_CASE(find_data_boundsFailureCloseBracketOpen) {
  string str = "NoDataExistsHere][";
  auto res = SD::Internal::find_data_bounds(str);

  BOOST_TEST(res.first == -1);
  BOOST_TEST(res.second == -1);
}


BOOST_AUTO_TEST_CASE(find_data_boundsSimple) {
  string str = "Vec3[x:1,y:2,z:3]";
  auto res = SD::Internal::find_data_bounds(str);

  BOOST_TEST(res.first == 4);
  BOOST_TEST(res.second == 16);
}


BOOST_AUTO_TEST_CASE(find_data_boundsComplex) {
  string str = "Sphere[r:1,center:Vec3[1,2,3]]";
  auto res = SD::Internal::find_data_bounds(str);

  BOOST_TEST(res.first == 6);
  BOOST_TEST(res.second == 29);
}


BOOST_AUTO_TEST_CASE(find_data_boundsNested) {
  string str = "Sphere[r:1,center:Vec3[1,2,3]]";
  auto res = SD::Internal::find_data_bounds(str, 18);

  BOOST_TEST(res.first == 22);
  BOOST_TEST(res.second == 28);
}

BOOST_AUTO_TEST_SUITE_END();


// Find geometry type

BOOST_AUTO_TEST_SUITE(find_geom_type);

BOOST_AUTO_TEST_CASE(find_geom_typeNoneExists) {
  string str = "[]";
  auto res = SD::Internal::find_geom_type(str);
  
  BOOST_TEST(res.first == SD::GeomType::Invalid);
}


BOOST_AUTO_TEST_CASE(find_geom_typeUnknownType) {
  string str = "awefji1o23awejfk5lsd[]";
  auto res = SD::Internal::find_geom_type(str);
  
  BOOST_TEST(res.first == SD::GeomType::Invalid);
  BOOST_TEST(res.second == SD::ErrorType::InvalidName);
}


BOOST_AUTO_TEST_CASE(find_geom_typeVec3) {
  string str = "Vec3[x:1,y:2,z:3]";
  auto res = SD::Internal::find_geom_type(str);
  
  BOOST_TEST(res.first == SD::GeomType::Vec3);
  BOOST_TEST(res.second == SD::ErrorType::NoError);
}

BOOST_AUTO_TEST_SUITE_END();


// Find next pair

BOOST_AUTO_TEST_SUITE(find_next_pair);

BOOST_AUTO_TEST_CASE(find_next_pairSyntaxError) {
  string str = ":";
  auto res = SD::Internal::find_next_pair(str,0,1);

  BOOST_TEST(res.first == -1);
}


BOOST_AUTO_TEST_CASE(find_next_pairEndOfString) {
  string str = "]";
  auto res = SD::Internal::find_next_pair(str);

  BOOST_TEST(res.first == -2);
}


BOOST_AUTO_TEST_CASE(find_next_pairMiddle) {
  string str = "key:value,";
  auto res = SD::Internal::find_next_pair(str);

  BOOST_TEST(res.first == 9);
  BOOST_TEST(res.second == "key:value");
}

BOOST_AUTO_TEST_CASE(find_next_pairEnd) {
  string str = "key:value]";
  auto res = SD::Internal::find_next_pair(str);

  BOOST_TEST(res.first == 9);
  BOOST_TEST(res.second == "key:value");
}


BOOST_AUTO_TEST_SUITE_END();


// Find all pairs

BOOST_AUTO_TEST_SUITE(find_pairs);

BOOST_AUTO_TEST_CASE(find_pairsNormal) {
  string str = "Vec3[x:1,y:2,z:3]";
  auto res = SD::Internal::find_pairs(str);
  auto map = res.first;
  
  BOOST_TEST(!res.second);
  if (res.second) return;
  bool isEnd = map.find("x") == map.end();
  BOOST_TEST(!isEnd);
  if (isEnd) return;
  BOOST_TEST(map["x"] == "1");
  BOOST_TEST((uint64_t)map.size() == (uint64_t)3);
}


BOOST_AUTO_TEST_CASE(find_pairsNone) {
  string str = "Spoofy[]";
  auto res = SD::Internal::find_pairs(str);
  auto map = res.first;
  
  BOOST_TEST(!res.second);
  if (res.second) return;
  BOOST_TEST(map.empty());
}


BOOST_AUTO_TEST_CASE(find_pairsErr) {
  string str = "]]]]]";
  auto res = SD::Internal::find_pairs(str);
  
  BOOST_TEST(res.second);
}


BOOST_AUTO_TEST_CASE(find_pairsNested) {
  string str = "Foo[x: Vec3[x: 1, y: 2, z: 3], y: 2]";

  auto res = SD::Internal::find_pairs(str);

  BOOST_TEST(!res.second);
  BOOST_TEST((uint64_t)res.first.size() == (uint64_t)2);
}


BOOST_AUTO_TEST_SUITE_END();


// Parse double

BOOST_AUTO_TEST_SUITE(parse_double);


BOOST_AUTO_TEST_CASE(parse_doubleNormal) {
  string str = "12345";
  auto res = SD::Internal::parse_double(str);
  BOOST_TEST(res.first == 4);
  BOOST_TEST(res.second == 12345.0);
}

BOOST_AUTO_TEST_CASE(parse_doubleEnded) {
  string str = "12.345,";
  auto res = SD::Internal::parse_double(str);
  BOOST_TEST(res.first == 5);
  BOOST_TEST(res.second == 12.345);
}

BOOST_AUTO_TEST_CASE(parse_doubleTwoDecimal) {
  string str = "12.345.123";
  auto res = SD::Internal::parse_double(str);
  BOOST_TEST(res.first == -1);
}

BOOST_AUTO_TEST_CASE(parse_doubleNegative) {
  string str = "-12345";
  auto res = SD::Internal::parse_double(str);
  BOOST_TEST(res.first == 5);
  BOOST_TEST(res.second == -12345.0);
}

BOOST_AUTO_TEST_CASE(parse_doubleHyphenMiddle) {
  string str = "12345-123";
  auto res = SD::Internal::parse_double(str);
  BOOST_TEST(res.first == -1);
}

BOOST_AUTO_TEST_SUITE_END();


// (De-)serialization tests

BOOST_AUTO_TEST_SUITE(reSerialization);


BOOST_AUTO_TEST_CASE(testVec3Reserialize) {
  Vec3 test = {1.0, 2.0, 3.0};
  auto serialized   = SD::serialize(test);
  auto deserialized = SD::deserialize(serialized);

  BOOST_TEST(has<Vec3>(deserialized));

  if (has<SD::ErrorType>(deserialized)) {
    auto e = get<SD::ErrorType>(deserialized);
    if (!e) {
      cerr << "No error message" << endl;
    }
    cerr << SD::error_message(e) << endl;
    return;
  } else if (!has<Vec3>(deserialized)) {
    cerr << "Deserialized holds wrong type!" << endl;
    return;
  }

  auto v = get<Vec3>(deserialized);

  BOOST_TEST(v.x == test.x);
  BOOST_TEST(v.y == test.y);
  BOOST_TEST(v.z == test.z);
}


BOOST_AUTO_TEST_CASE(testLineSegmentReserialize) {
  Vec3 testv1 = {1.5, -2.0, 3.5};
  Vec3 testv2 = {0.0, 0.0, 12345};
  LineSegment test = { testv1, testv2 };
  auto serialized   = SD::serialize(test);
  auto deserialized = SD::deserialize(serialized);

  BOOST_TEST(has<LineSegment>(deserialized));

  if (has<SD::ErrorType>(deserialized)) {
    auto e = get<SD::ErrorType>(deserialized);
    if (!e) {
      cerr << "No error message" << endl;
    }
    cerr << SD::error_message(e) << endl;
    return;
  }
  else if (!has<LineSegment>(deserialized)) {
    cerr << "Deserialized holds wrong type!" << endl;
    return;
  };

  auto ls = get<LineSegment>(deserialized);

  BOOST_TEST(ls.a.x == testv1.x);
  BOOST_TEST(ls.a.y == testv1.y);
  BOOST_TEST(ls.a.z == testv1.z);
  BOOST_TEST(ls.b.x == testv2.x);
  BOOST_TEST(ls.b.y == testv2.y);
  BOOST_TEST(ls.b.z == testv2.z);
}


BOOST_AUTO_TEST_CASE(testAngleReserialize) {
  Quaternion test   = Quaternion::from_spherical_angle(PI, PI/2);
  auto serialized   = SD::serialize(test);
  auto deserialized = SD::deserialize(serialized);

  BOOST_TEST(has<Quaternion>(deserialized));

  if (has<SD::ErrorType>(deserialized)) {
    auto e = get<SD::ErrorType>(deserialized);
    if (!e) {
      cerr << "No error message" << endl;
    }
    cerr << SD::error_message(e) << endl;
    return;
  } else if (!has<Quaternion>(deserialized)) {
    cerr << "Deserialized holds wrong type!" << endl;
    return;
  };

  auto q = get<Quaternion>(deserialized);

  BOOST_TEST(q.w == test.w);
  BOOST_TEST(q.x == test.x);
  BOOST_TEST(q.y == test.y);
  BOOST_TEST(q.z == test.z);
}


BOOST_AUTO_TEST_CASE(testPrismReserialize) {
  Prism test = { Vec3(1, 2, 3), 4, 5, 6, Quaternion::from_spherical_angle(-PI, PI/2)};
  auto serialized   = SD::serialize(test);
  auto deserialized = SD::deserialize(serialized);

  BOOST_TEST(has<Prism>(deserialized));

  if (has<SD::ErrorType>(deserialized)) {
    auto e = get<SD::ErrorType>(deserialized);
    if (!e) {
      cerr << "No error message" << endl;
    }
    cerr << SD::error_message(e) << endl;
    return;
  }
  else if (!has<Prism>(deserialized)) {
    cerr << "Deserialized holds wrong type!" << endl;
    return;
  };

  auto p = get<Prism>(deserialized);

  BOOST_TEST(p.center.x == test.center.x);
  BOOST_TEST(p.center.y == test.center.y);
  BOOST_TEST(p.center.z == test.center.z);
  BOOST_TEST(p.orientation.w == test.orientation.w);
  BOOST_TEST(p.orientation.x == test.orientation.x);
  BOOST_TEST(p.orientation.y == test.orientation.y);
  BOOST_TEST(p.orientation.z == test.orientation.z);
  BOOST_TEST(p.ex == test.ex);
  BOOST_TEST(p.ey == test.ey);
  BOOST_TEST(p.ez == test.ez);
}


BOOST_AUTO_TEST_CASE(testSphereReserialize) {
  Sphere test = {{9, 9, 9}, 9.25};
  auto serialized   = SD::serialize(test);
  auto deserialized = SD::deserialize(serialized);

  BOOST_TEST(has<Sphere>(deserialized));

  if (has<SD::ErrorType>(deserialized)) {
    auto e = get<SD::ErrorType>(deserialized);
    if (!e) {
      cerr << "No error message" << endl;
    }
    cerr << SD::error_message(e) << endl;
    return;
  }
  else if (!has<Sphere>(deserialized)) {
    cerr << "Deserialized holds wrong type!" << endl;
    return;
  }

  auto s = get<Sphere>(deserialized);

  BOOST_TEST(s.r == test.r);
  BOOST_TEST(s.center.x == test.center.x);
  BOOST_TEST(s.center.y == test.center.y);
  BOOST_TEST(s.center.z == test.center.z);
}


BOOST_AUTO_TEST_SUITE_END();
BOOST_AUTO_TEST_SUITE_END();


BOOST_AUTO_TEST_SUITE(Pathgen);


/*
BOOST_AUTO_TEST_CASE(testPathExistsTrivial) {
  const PG::MovePoint from = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.4,0.4,0.4},{0,0}},
  },
  to = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.4,0.4,0.4},{0,0}},
  };

  vector<Intersectable> geom;

  auto p = PG::single_move(from, to, geom);
  auto is_err = has<PG::ErrorType>(p);

  BOOST_TEST(!is_err);

  if (is_err) {
  cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
  }
}


BOOST_AUTO_TEST_CASE(testPathExistsSimpleTrans) {
  const PG::MovePoint from = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.4,0.4,0.4},{0,0}},
  },
  to = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.5,0.5,0.5},{0,0}},
  };

  vector<Intersectable> geom;

  auto p = PG::single_move(from, to, geom);
  auto is_err = has<PG::ErrorType>(p);

  BOOST_TEST(!is_err);

  if (is_err) {
  cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
  }
}


BOOST_AUTO_TEST_CASE(testPathExistsSimpleRot) {
  const PG::MovePoint from = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.4,0.4,0.4},{0,0}},
  },
  to = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.4,0.4,0.4},{PI/4,PI/4}},
  };

  vector<Intersectable> geom;

  auto p = PG::single_move(from, to, geom);
  auto is_err = has<PG::ErrorType>(p);

  BOOST_TEST(!is_err);

  if (is_err) {
  cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
  }
}


BOOST_AUTO_TEST_CASE(testPathExistsSingleObstacle) {
  const PG::MovePoint from = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.35,0.35,0.35},{0,0}},
  },
  to = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.6,0.6,0.35},{0,0}},
  };

  vector<Intersectable> geom = {
    (Sphere){Vec3(0.35,0.5,0.3),0.01},
  };

  auto p = PG::single_move(from, to, geom);
  auto is_err = has<PG::ErrorType>(p);

  BOOST_TEST(!is_err);

  if (is_err) {
  cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
  }
}
*/


BOOST_AUTO_TEST_CASE(testPathExistsMultiObstacle) {
  const PG::MovePoint from = {
    {{0.1,0.1,0.1},{0,0}},
    {{0.35,0.35,0.35},{0,0}},
  },
    to = {
      {{0.05,0.05,0.05},{0,0}},
      {{0.35,0.35,0.35},{0,0}},
    };

    vector<Intersectable> geom = {
      (Sphere){Vec3(0.5,0.5,0.5),0.1},
      (Sphere){Vec3(0.55,0.55,0.55),0.1},
      Prism(Vec3(-0.3, -0.3, -0.3), 0.05, 0.05, 0.05, Quaternion::identity()),
    };

    auto p = PG::single_move(from, to, geom);
    auto is_err = has<PG::ErrorType>(p);

    BOOST_TEST(!is_err);

    if (is_err) {
      cerr << C_BR_RED << "Pathgen error: " << PG::error_message(get<PG::ErrorType>(p)) << C_RED << "\n" << std::flush;
    }
}


BOOST_AUTO_TEST_SUITE_END();
