MATLABROOT = /usr/local/MATLAB/R2018a/bin

HOME_PATH = $(CURDIR)
SRC_PATH = $(HOME_PATH)/src
LIB_PATH = $(HOME_PATH)/lib
BIN_PATH = $(HOME_PATH)/bin

# Definition of Compilers
MEX = $(MATLABROOT)/mex
OCT = /bin/mkoctfile
GCC = g++

# General Options to pass to the compiler defined in CFLAGS
COMMON_CXX =  -O2 -Wall -Wextra -std=gnu++11
MYLIBS = -lwindcube

# Matlab Options:
MYFLAGS = CXXFLAGS='$$CXXFLAGS $$COMMON_CXX' #-Wall -O2 -std=gnu++11'
CC = $(MATLABROOT)/bin/mex # Matlab compiler


$(BIN_PATH)/read_V2lidar: $(SRC_PATH)/read_V2Lidar.cpp $(LIB_PATH)/libwindcube.so
	$(GCC) -std=gnu++11 -L$(LIB_PATH) $^ -lwindcube -o $@

$(LIB_PATH)/libwindcube.so: $(SRC_PATH)/windcube.o
	$(GCC) -shared -std=gnu++11 $^ -o $@

$(SRC_PATH)/windcube.o: $(SRC_PATH)/windcube.cpp
	$(GCC) -c -fPIC -std=gnu++11 $^ -o $@


clean:
	rm $(SRC_PATH)/*.o

