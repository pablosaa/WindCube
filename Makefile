# --------------------------------------------------------------------------
# MAKEFILE to compile the WindCubMEX library files:
# By default it compiles the dinamic library alone : ../lib/libwindcube.so
# Optionally it creates the MATLAB or the OCTAVE MEX Funcions
#
# USAGE:
# * to create the only the library
# > make
# * to create the MATLAB MEX function
# > make matlab
# * to create the OCTAVE MEX function
# > make octave
#
# --
# Part of the '' repository
# SEE LICENCE.TXT
# -------------------------------------------------------------------------

# Base directory where repository is located:
HOME_PATH:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
##HOME_PATH = $(CURDIR)
SRC_PATH = $(HOME_PATH)/src
LIB_PATH = $(HOME_PATH)/lib
BIN_PATH = $(HOME_PATH)/bin
FOO := $(firstword $(shell which matlab))

# Definition of Compilers
MEX = $(subst matlab,mex,$(shell readlink -f $(FOO)))
OCT = /usr/bin/mkoctfile      # Octave compiler
GCC = g++

# General Options to pass to the compiler defined in CFLAGS
COMMON_CXX =  -O2 -Wall -Wextra -std=gnu++11
MYLIBS = -lwindcube

# Matlab Options:
MYFLAGS = CXXFLAGS='$$CXXFLAGS $$COMMON_CXX' #-Wall -O2 -std=gnu++11'

# Compilation commands starts here:
$(BIN_PATH)/read_V2lidar: $(SRC_PATH)/read_V2Lidar.cpp $(LIB_PATH)/libwindcube.so
	$(GCC) -std=gnu++11 -L$(LIB_PATH) $^ -lwindcube -o $@

$(LIB_PATH)/libwindcube.so: $(SRC_PATH)/windcube.o
	$(GCC) -shared -std=gnu++11 $^ -o $@

$(SRC_PATH)/windcube.o: $(SRC_PATH)/windcube.cpp
	$(GCC) -c -fPIC -std=gnu++11 $^ -o $@

matlab: $(SRC_PATH)/windcube.o 
	$(MEX) $(MYFLAGS) $(SRC_PATH)/Read_WindCube.cpp $(SRC_PATH)/windcube.o -outdir $(BIN_PATH)

octave: $(SRC_PATH)/windcube.o
	$(OCT) $(COMMON_CXX) --mex $(SRC_PATH)/Read_WindCube.cpp $(SRC_PATH)/windcube.o -o $(BIN_PATH)/Read_WindCube.mex
 
clean:
	rm $(SRC_PATH)/*.o

