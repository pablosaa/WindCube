GCC = g++

HOME_PATH = $(CURDIR)
SRC_PATH = $(HOME_PATH)/src
LIB_PATH = $(HOME_PATH)/lib
BIN_PATH = $(HOME_PATH)/bin

$(BIN_PATH)/read_V2lidar: $(SRC_PATH)/read_V2Lidar.cpp $(LIB_PATH)/libwindcube.so
	$(GCC) -std=gnu++11 -L$(LIB_PATH) $^ -lwindcube -o $@

$(LIB_PATH)/libwindcube.so: $(SRC_PATH)/windcube.o
	$(GCC) -shared -std=gnu++11 $^ -o $@

$(SRC_PATH)/windcube.o: $(SRC_PATH)/windcube.cpp
	$(GCC) -c -fPIC -std=gnu++11 $^ -o $@


clean:
	rm $(SRC_PATH)/*.o

