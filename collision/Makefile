# CXX = /usr/local/bin/g++-8

CFLAGS = -Wall -Igeometry -Iserialization -Ipathgen -I. -I.. -std=gnu++0x -march=native -mtune=native -fexceptions
VPATH  = geometry:pathgen:serialization

ifeq ($(RELEASE),TRUE)
	DEBUG_O :=
	CFLAGS += -g -O3 -Werror=implicit-function-declaration -D_FORTIFY_SOURCE=1
else
	DEBUG_O := debug.o
	CFLAGS += -g3 -D_FORTIFY_SOURCE=2 -DDEBUG -rdynamic
endif

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
	CFLAGS += -I/home1/midptf/packages/boost/include/**
	CFLAGS += -lboost_unit_test_framework
else 
	# this is useful for debugging on your own computer if you aren't running Linux
	CXX = /usr/local/bin/g++-8
	CFLAGS += -I/Users/Mia/packages/boost-src/ -L/Users/Mia/packages/boost-src/stage/lib
	# CFLAGS += -I/usr/local/include/
	CFLAGS += -lboost_unit_test_framework
endif

ifeq ($(PRINT),TRUE)
	CFLAGS += -DDEBUG_PRINT
endif

ifeq ($(RELEASE)$(UNAME_S),Darwin)
	# very useful flag on newer GCC versions for debugging
	# turns on all optimizations that don't interfere with debuggers and are fast to compile
	CFLAGS += -Og
endif

CFLAGS += -lboost_regex -lboost_regex-mt

CXXFLAGS = $(CFLAGS)

GEOM_OBJECTS := vec3.o rotations.o quaternion.o prism.o
INTERSECT_OBJECTS := intersection_static.o intersection_displacement.o intersection_rotation.o sat.o bounds.o
PATHGEN_OBJECTS := pathgen.o rect.o cyl.o

tests: tests.o geom.o serialization_internal.o serialization.o $(GEOM_OBJECTS) $(INTERSECT_OBJECTS) $(PATHGEN_OBJECTS) $(DEBUG_O)
	$(CXX) -o $@ $(CXXFLAGS) $^

debugme: debugme.cxx geom.o serialization_internal.o serialization.o $(GEOM_OBJECTS) $(INTERSECT_OBJECTS) $(PATHGEN_OBJECTS) $(DEBUG_O)
	$(CXX) -o $@ $(CXXFLAGS) $^

test: tests
	./tests

all: tests

notest: geom.o serialization_internal.o serialization.o $(GEOM_OBJECTS) $(INTERSECT_OBJECTS) $(DEBUG_O)

serialization_internal.o: serialization_internal.cpp $(DEBUG_O)
	$(CXX) -c $(CXXFLAGS) $< -o $@

serialization.o: serialization.cpp $(DEBUG_O)
	$(CXX) -c $(CXXFLAGS) $< -o $@ 

tests.o: tests.cpp $(DEBUG_O)
	$(CXX) -o $@ -c $< $(CXXFLAGS) -Wno-format-overflow

%.o: %.cpp %.hpp $(DEBUG_O)
	$(CXX) -o $@ -c $< $(CXXFLAGS)

col-clean:
	rm -rf *.o *.gch *.dSYM tests
