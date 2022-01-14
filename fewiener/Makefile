# Makefile
#
# $Id$
#

OSFLAGS  = -DOS_LINUX -Dextname
CFLAGS   = -g -O2 -fPIC -Wall -Wuninitialized -I. -I$(MIDASSYS)/include -I.. -std=c++11
CXXFLAGS = $(CFLAGS)

LIBS = -lm -lz -lutil -lnsl -lpthread -lrt

# MIDAS library
MIDASLIBS = $(MIDASSYS)/lib/libmidas.a
MFE = $(MIDASSYS)/lib/mfe.o

# location of the Wiener SNMP MIB file
CFLAGS += -DWIENER_MIB_DIR=\"$(PWD)\"

all:: dox
all:: feptfwiener.exe
#all:: fefgdwiener01.exe
#all:: fefgdwiener02.exe
#all:: fedeapvme01.exe
#all:: fedeapvme02.exe
#all:: fedeapvme03.exe
#all:: fedeapmpod.exe
#all:: feirisvme1.exe
#all:: feirisvme2.exe

iris: feirisvme1.exe feirisvme2.exe
deap: fedeapvme01.exe fedeapvme02.exe fedeapvme03.exe fedeapmpod.exe
ptf: feptfwiener.exe

dox: ; doxygen ../Doxyfile 

fewiener.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS)

fefgdwiener01.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fefgdwiener01\" -DEQ_NAME=\"FgdWiener01\"

fefgdwiener02.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fefgdwiener02\" -DEQ_NAME=\"FgdWiener02\"

fedeapvme01.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fedeapvme01\" -DEQ_NAME=\"deapvme01\" -DEQ_EVID=1

fedeapvme02.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fedeapvme02\" -DEQ_NAME=\"deapvme02\" -DEQ_EVID=1

fedeapvme03.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fedeapvme03\" -DEQ_NAME=\"deapvme03\" -DEQ_EVID=1

fedeapmpod.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"fedeapmpod\" -DEQ_NAME=\"deapmpod\" -DEQ_EVID=1

feirisvme1.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"feirisvme1\" -DEQ_NAME=\"VmeCrate1\"

feirisvme2.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"feirisvme2\" -DEQ_NAME=\"VmeCrate2\"

feptfwiener.exe: %.exe: fewiener.cxx $(MIDASLIBS) $(MFE)
	$(CXX) -o $@ $(CFLAGS) $(OSFLAGS) $^ $(MIDASLIBS) $(LIBS) -DFE_NAME=\"feptfwiener\" -DEQ_NAME=\"PtfWiener\"

mxml.o: $(MIDASSYS)/../mxml/mxml.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<

strlcpy.o: $(MIDASSYS)/../mxml/strlcpy.c
	$(CC) $(CFLAGS) $(OSFLAGS) -c -o $@ -c $<

%.o: %.c
	$(CXX) $(CFLAGS) $(OSFLAGS) -c $<

%.o: %.cxx
	$(CXX) $(CXXFLAGS) $(OSFLAGS) -c $<

#%.cxx.d: %.cxx
#	$(CXX) -MF $<.d -MT $< -MM $(CXXFLAGS) $(OSFLAGS) -c $<
#
#%.c.d: %.c
#	$(CC) -MF $<.d -MT $< -MM $(CFLAGS) $(OSFLAGS) -c $<

depend:

-include *.d

clean::
	-rm -f *.o *.exe
	-rm -f *.d

# end
