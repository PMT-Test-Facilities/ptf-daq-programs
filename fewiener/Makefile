# Makefile
#
# $Id$
#

OSFLAGS  = -DOS_LINUX -Dextname
CFLAGS   =  -ggdb -fPIC -Wall -Wuninitialized -I. -I$(MIDASSYS)/include -I.. -I./ -std=c++11
CXXFLAGS = $(CFLAGS)

LIBS = -lm -lz -lutil -lnsl -lpthread -lrt

# MIDAS library
MIDASLIBS = $(MIDASSYS)/lib/libmidas.a
MFE = $(MIDASSYS)/lib/mfe.o

# location of the Wiener SNMP MIB file
CFLAGS += -DWIENER_MIB_DIR=\"$(PWD)\"

all:: feptfwiener.exe

iris: feirisvme1.exe feirisvme2.exe
deap: fedeapvme01.exe fedeapvme02.exe fedeapvme03.exe fedeapmpod.exe
ptf: feptfwiener.exe

dox: ; doxygen ../Doxyfile 

feptfwiener.exe:  %.exe: fewiener.cxx  $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"feptfwiener\" -DEQ_NAME=\"PtfWiener\"


mxml.o: $(MIDASSYS)/../mxml/mxml.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<

strlcpy.o: $(MIDASSYS)/../mxml/strlcpy.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<

%.o: %.c
	$(CXX) -o $@ -c $< $(CFLAGS)

%.o: %.cxx
	$(CXX) -o $@ -c $< $(CXXFLAGS)

%.o: %.cpp
	$(CXX) -o $@ -c $< $(CXXFLAGS)

depend:

-include *.d

clean::
	-rm -f *.o *.exe
	-rm -f *.d

# end
