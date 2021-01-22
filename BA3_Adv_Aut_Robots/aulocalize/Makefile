# makefile $Id: Makefile 535 2014-08-13 12:27:35Z jcan $
#
# libraryname (for shared library?)
libname = aulocalize.so
#
# preprocessor flags like -I(include dir)
CPPFLAGS = -I../../include -fopenmp
# CPPFLAGS += `pkg-config gstreamer-0.10 --cflags`
#
# linker flags like e.g. -lpthread and -L/usr/local/lib
LDFLAGS = -g2 -shared -Wl,-soname,$(libname) -L../../lib -liau_mat -liau_lel -lutils -lgomp
#
# extra preprocessor defines (mainly for module testcode)
DEFINES = -D LIBRARY_OPEN_NEEDED
#
# C++ compiler flags
CXXFLAGS = -g2 -O0 -Wall  -Wno-enum-compare -Wno-unused-local-typedefs -fPIC $(DEFINES)
# CXXFLAGS += -Wno-unused-but-set-variable
include ../../include/opencv.mk
include ../../include/opencv_flags.mk
#
# Object files to produce before link
objects = templateInstantiations.o ufunclocalize.o

# shared library file name (version 0)
shlib = $(libname).0

# compile all - all objects depend on all other (objects)
all: $(objects)
	c++ -o $(shlib) $(objects) $(LDFLAGS)

.PHONY : clean install
clean :
	rm -f $(shlib) $(objects)
	-rm -f *~


install:
	cp $(shlib) ../../build/lib
include ../../include/dependencies4objects.mk
