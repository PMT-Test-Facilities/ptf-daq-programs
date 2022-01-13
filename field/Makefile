UNAME_S := $(shell uname -s)

CFLAGS = -lm -lz -lutil -lpthread -march=native -mtune=native -fexceptions -fexceptions

ifeq ($(UNAME_S),Darwin)
	CFLAGS += -std=c++11 
	CFLAGS += -I/Users/Mia/packages/midas/include
	CXX     = /usr/local/bin/g++-8
	CFLAGS += -I/Users/Mia/packages/boost-src -L/Users/Mia/packages/boost-src/stage/lib
	MIDAS  := 
endif
ifeq ($(UNAME_S),Linux)
	CFLAGS += -std=gnu++0x
	CFLAGS += -I/home1/midptf/packages/boost/include/**
	CFLAGS += -I$(HOME)/packages/midas/include -I$(HOME)/packages/midas/drivers/vme -I$(HOME)/packages/midas/drivers/bus
	CFLAGS += -L$(HOME)/packages/midas/lib/
	CFLAGS += -DOS_LINUX -Dextname -lnsl -lrt
	MIDAS  := $(HOME)/packages/midas/lib/libmidas.a
endif
# CFLAGS += -I$(ROOTSYS)/include/root -L$(ROOTSYS)/lib/root
# CFLAGS += -lCore -lHist -lRIO -lTree -lGpad

ifeq ($(RELEASE),TRUE)
	CFLAGS += -g3 -O2 -D_FORTIFY_SOURCE=1 -Werror=implicit-function-declaration
else
	CFLAGS += -g3 -D_FORTIFY_SOURCE=2 -DDEBUG
endif

%.o: %.cxx %.hxx
	$(CXX) -c $(CFLAGS) $< -o $@

optimize: optimize.cxx degauss.o odblayer.o util.o
	$(CXX) $(CFLAGS) $^ $(MIDAS) -o $@

single: single.cxx degauss.o odblayer.o util.o
	$(CXX) $(CFLAGS) $^ $(MIDAS) -o $@

set: set.cxx degauss.o odblayer.o util.o
	$(CXX) $(CFLAGS) $^ $(MIDAS) -o $@

clean:
	rm -rf *.o *.gch *.dSYM optimize
