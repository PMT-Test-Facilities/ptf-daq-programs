# Makefile for PMT test facility
#
# $Id: Makefile 1131 2008-05-22 21:35:42Z olchansk $
# 
# $Log$
#

VPATH = degauss:motor:scan:pathcalc:collision:move:collision/geometry:collision/pathgen:collision/serialization

CFLAGS  = -std=gnu++0x -Dextname -g -Wall -Wuninitialized -march=native -mtune=native
CFLAGS += -Ipathcalc -Iscan -Imove -Imotor -Idegauss -Icollision -Icollision/geometry -Icollision/pathgen -Icollision/serialization -I.
# GCC 4.4 seems to have a bug with strict aliasing warnings
# There's one other spot where aliasing is technically violated but it's right before one of the values is dropped
CFLAGS += -Wno-strict-aliasing

ifeq ($(DEBUG),TRUE)
	DEBUG_O := debug.o
	CFLAGS += -g -D_FORTIFY_SOURCE=2 -DDEBUG -rdynamic
else
	DEBUG_O :=
	CFLAGS += -O2 -finline-functions -fpredictive-commoning -ftree-vectorize -fexceptions -D_FORTIFY_SOURCE=1
endif

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
	# packages for new collision avoidance
	CFLAGS += -I/home1/midptf/packages/boost/include/** -L/home1/midptf/packages/boost/lib
	# packages for old collision avoidance
	CFLAGS += -I/home1/midptf/boost_1_47_0/
	CFLAGS += -DOS_LINUX
else
	# Mia: since I'm working on a Mac I'm using this to select different packages
	CXX = /usr/local/bin/g++-8
	CFLAGS += -I/Users/Mia/packages/boost-src/ -L/Users/Mia/packages/boost-src/stage/lib
endif

ifeq ($(PRINT),TRUE)
	CFLAGS += -DDEBUG_PRINT
endif

# very useful flag on newer GCC versions for debugging
# turns on all optimizations that don't interfere with debuggers and are fast to compile
ifeq ($(DEBUG)$(UNAME_S),TRUEDarwin)
	CFLAGS += -Og
endif


# MIDAS location

MIDASSYS=$(HOME)/packages/midas

DRV_DIR = $(MIDASSYS)/drivers/bus

MFE       = $(MIDASSYS)/linux/lib/mfe.o
MIDASLIBS = $(MIDASSYS)/linux/lib/libmidas.a
CFLAGS   += -I$(MIDASSYS)/include
CFLAGS   += -I$(MIDASSYS)/drivers/vme
# CFLAGS   += -I$(HOME)/packages/root
CFLAGS   += -I$(DRV_DIR)


# ROOT library

ROOTLIBS  = $(shell $(ROOTSYS)/bin/root-config --libs) -lThread -Wl,-rpath,$(ROOTSYS)/lib
ROOTGLIBS = $(shell $(ROOTSYS)/bin/root-config --glibs) -lThread -Wl,-rpath,$(ROOTSYS)/lib


# VME interface library

VMICHOME     = $(HOME)/packages/vmisft-7433-3.5-KO2/vme_universe
VMICLIBS     = $(VMICHOME)/lib/libvme.so.3.4 -Wl,-rpath,$(VMICHOME)/lib
#VMICLIBS     = -lvme
CFLAGS      += -I$(VMICHOME)/include


#USE_VMICVME=1
USE_GEFVME=1

ifdef USE_VMICVME
VMICHOME     = $(HOME)/packages/vmisft-7433-3.5-KO2/vme_universe
VMICLIBS     = $(VMICHOME)/lib/libvme.so.3.4 -Wl,-rpath,$(VMICHOME)/lib
#VMICLIBS     = -lvme
CFLAGS      += -I$(VMICHOME)/include
VMEDRV      = vmicvme.o
VMELIBS     = -lvme
endif

ifdef USE_GEFVME
VMEDRV      = gefvme.o
VMELIBS     =
endif


# support libraries

LIBS = -lm -lz -lutil -lnsl -lpthread -lrt -lboost_unit_test_framework -lboost_regex -lboost_regex-mt

GEOM_OBJECTS          := geom.o vec3.o rotations.o quaternion.o prism.o
INTERSECT_OBJECTS     := intersection_static.o intersection_displacement.o intersection_rotation.o sat.o bounds.o
PATHGEN_OBJECTS       := pathgen.o rect.o cyl.o
SERIALIZATION_OBJECTS := serialization_internal.o serialization.o

# include collision/Makefile


all: feMotor feMove feMove_prev feScan fedvm fePhidget feDegauss fesimdaq.exe feptfwiener.exe $(GEOM_OBJECTS) $(PATHGEN_OBJECTS) $(INTERSECT_OBJECTS) $(SERIALIZATION_OBJECTS)


gefvme.o: %.o: $(MIDASSYS)/drivers/vme/vmic/%.c
	$(CC) -c -o $@ $(CFLAGS)  $<

feMotor: $(MIDASLIBS) $(MFE) feMotor.o $(DRV_DIR)/tcpip.o cd_Galil.o 
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

feMove: feMove.cxx feMove.hxx $(MIDASLIBS) $(MFE) $(GEOM_OBJECTS) $(PATHGEN_OBJECTS) $(INTERSECT_OBJECTS) $(SERIALIZATION_OBJECTS)
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS)

feMove_prev: $(MIDASLIBS) $(MFE) feMove_prev.o TPathCalculator.o TRotationCalculator.o TGantryConfigCalculator.o
		$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

feScan: $(MIDASLIBS) $(MFE) feScan.o  ScanSequence.o
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

fedvm: $(MIDASLIBS) $(MFE) fedvm.o 
	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

fePhidget: $(MIDASLIBS) $(MFE) fePhidget.o 
	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS) -lphidget21

fesimdaq.exe: $(MFE) fesimdaq.o 
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS)  $(MIDASLIBS) $(LIBS)

feptfwiener.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"feptfwiener\" -DEQ_NAME=\"PtfWiener\" -DEQ_EVID=EVID_PTFWIENER

feDegauss: $(MIDASLIBS) $(MFE) degauss.o feDegauss.o
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS) $(VMELIBS) -DDEBUG

serialization_internal.o: serialization_internal.cpp $(DEBUG_O)
	$(CXX) -c $(CFLAGS) $< -o $@

serialization.o: serialization.cpp $(DEBUG_O)
	$(CXX) -c $(CFLAGS) $< -o $@ 

tests.o: tests.cpp $(DEBUG_O)
	$(CXX) -o $@ -c $< $(CFLAGS) -Wno-format-overflow

tests: tests.o $(GEOM_OBJECTS) $(INTERSECT_OBJECTS) $(PATHGEN_OBJECTS) $(DEBUG_O)
	$(CXX) -o $2 $^ $(CFLAGS)


collision: $(GEOM_OBJECTS) $(PATHGEN_OBJECTS) $(INTERSECT_OBJECTS) $(SERIALIZATION_OBJECTS) $(DEBUG_O)

%.o: $(MIDASSYS)/drivers/vme/%.c
	$(CC) -o $@ -c $< $(CFLAGS)

%.o: $(MIDASSYS)/drivers/vme/%.c
	$(CC) -o $@ -c $< $(CFLAGS)

%.o: $(MIDASSYS)/src/%.c
	$(CC) -o $@ -c $< $(CFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

%.o: %.cxx %.hxx
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.cpp %.hpp
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.cxx
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $< $(CFLAGS)


clean:
	rm -f *.o */*.o *.a *.gch *.dSYM feMotor feMove feMove_prev feScan testVI feDegauss fedvm fePhidget fesimdaq.exe feptfwiener.exe

# end
