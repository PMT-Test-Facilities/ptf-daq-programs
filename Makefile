# Makefile for PMT test facility
#
# $Id: Makefile 1131 2008-05-22 21:35:42Z olchansk $
# 
# $Log$
#

VPATH = degauss:motor:scan:pathcalc:move

CFLAGS   = -DOS_LINUX -Dextname -g -O2 -Wall -Wuninitialized -I/home1/midptf/boost_1_47_0/ -std=gnu++0x -Ipathcalc -Iscan -Imove -Imotor -Idegauss -I. -DDEBUG
CXXFLAGS = $(CFLAGS)

# MIDAS location

MIDASSYS=$(HOME)/packages/midas

DRV_DIR = $(MIDASSYS)/drivers/bus

MFE       = $(MIDASSYS)/lib/mfe.o
MIDASLIBS = $(MIDASSYS)/lib/libmidas.a
CFLAGS   += -I$(MIDASSYS)/include
CFLAGS   += -I$(MIDASSYS)/drivers/vme
# CFLAGS   += -I$(HOME)/packages/root
CFLAGS   += -I$(DRV_DIR)
CXX = g++
# ROOT library

ROOTLIBS  = $(shell $(ROOTSYS)/bin/root-config --libs) -lThread -Wl,-rpath,$(ROOTSYS)/lib
ROOTGLIBS = $(shell $(ROOTSYS)/bin/root-config --glibs) -lThread -Wl,-rpath,$(ROOTSYS)/lib
#CXXFLAGS += -DHAVE_ROOT -DUSE_ROOT -I$(ROOTSYS)/include

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

LIBS = -lm -lz -lutil -lnsl -lpthread -lrt

# all: default target for "make"

all:feMotor feMove feScan fedvm fePhidget fesimdaq.exe feptfwiener.exe testVI feDegauss


gefvme.o: %.o: $(MIDASSYS)/drivers/vme/vmic/%.c
	$(CXX) -c -o $@ $(CFLAGS)  $<


feMotor: $(MIDASLIBS) $(MFE) feMotor.o $(DRV_DIR)/tcpip.o cd_Galil.o 
	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

feMove: $(MIDASLIBS) $(MFE) feMove.o TPathCalculator.o TRotationCalculator.o TGantryConfigCalculator.o
	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

#feMoveNew: $(MIDASLIBS) $(MFE) feMove.o TPathCalculator.o TRotationCalculator.o TGantryConfigCalculator.o
#	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

#Rika (18Apr2017): Added feMoveOld for debugging gantry position errors during a large area scan
#feMoveOld: $(MIDASLIBS) $(MFE) feMove.o TPathCalculator.o TRotationCalculator.o
#	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

feScan: $(MIDASLIBS) $(MFE) feScan.o  ScanSequence.o
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

# DEPRECATED (from test phase)
#feScanNew: $(MIDASLIBS) $(MFE) feScan_new.o 
#	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)

# DEPRECATED (from test phase)
#feScanNewNew: $(MIDASLIBS) $(MFE) feScan_newnew.o ScanSequence.o
#	$(CXX) -o $@ $(CFLAGS)  $^ $(MIDASLIBS) $(LIBS) $(VMELIBS)
testVI: $(MIDASLIBS) coilVoltageCurrent.o
	$(CXX) -o $@ $(CFLAGS) $^ $(MIDASLIBS) $(LIBS) $(VMELIBS) # coilVoltageCurrent.cpp # -nostartfiles

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


%.o: $(MIDASSYS)/drivers/vme/%.c
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: $(MIDASSYS)/src/%.c
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.c
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.cxx
	$(CXX) -o $@ -c $< $(CXXFLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $< $(CXXFLAGS)

%.o: $(MIDASSYS)/drivers/vme/%.c
	$(CXX) -o $@ -c $< $(CFLAGS)


clean:
	rm -f *.o *.gch *.dSYM feMotor feMove feMoveNew feMoveOld feScan testVI fedvm fePhidget fesimdaq.exe feptfwiener.exe

# end
