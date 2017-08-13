INC = -I include/ -I /usr/local/include/eigen3
LIB = -lwiringPi -lncurses -pthread
SRC_OBJ := $(patsubst src/%.cpp, .obj/src/%.o, $(wildcard src/*.cpp))

CC = g++
L_FLAGS = -g -o
C_FLAGS = -g -c -std=c++11 -o
BIN_DIR = mkdir -p bin
OBJ_DIR = mkdir -p .obj/src .obj/test
LINK = $(BIN_DIR); $(CC) $(L_FLAGS) $@ $^ $(LIB)
COMPILE = $(OBJ_DIR); $(CC) $(C_FLAGS) $@ $< $(INC)

all: launch.exe tests
tests: pidtest kalmantest bmptest serialtest filtertest ctrltest chronotest iostest skipstest

chronotest: bin/chronotest
iostest: bin/iostest
pidtest: bin/pidtest
timetest: bin/timetest
kalmantest: bin/kalmantest
bmptest: bin/bmptest
serialtest: bin/serialtest
filetest: bin/filetest
filtertest: bin/filtertest
ctrltest: bin/ctrltest
skipstest: bin/skipstest

# Target executable #

launch.exe: $(SRC_OBJ)
	$(LINK)

# Test executables #

bin/pidtest: .obj/test/pidtest.o .obj/src/pid.o
	$(LINK)

bin/timetest: .obj/test/timetest.o .obj/src/timeutil.o
	$(LINK)

bin/kalmantest: .obj/test/kalmantest.o .obj/src/kalman.o 
	$(LINK)

bin/bmptest: .obj/test/bmptest.o .obj/src/bmp.o .obj/src/i2c.o .obj/src/smem.o
	$(LINK)

bin/serialtest: .obj/test/serialtest.o .obj/src/ardimu.o .obj/src/smem.o
	$(LINK)

bin/filetest: .obj/test/filetest.o .obj/src/timeutil.o .obj/src/filebuffer.o .obj/src/smem.o
	$(LINK)

bin/filtertest: .obj/test/filtertest.o .obj/src/filters.o
	$(LINK)

bin/ctrltest: .obj/test/ctrltest.o .obj/src/control.o .obj/src/filters.o .obj/src/pid.o .obj/src/smem.o .obj/src/ardimu.o .obj/src/bmp.o .obj/src/i2c.o
	$(LINK)

bin/chronotest: .obj/test/chronotest.o
	$(LINK)

bin/iostest: .obj/test/iostest.o
	$(LINK)

bin/skipstest: .obj/test/skipstest.o
	$(LINK)

# Special dependencies #

.obj/src/launch.o: src/launch.cpp
	$(COMPILE)

# Pattern object recipes #

.obj/src/%.o: src/%.cpp include/%.h
	$(COMPILE)

.obj/test/%.o: test/%.cpp
	$(COMPILE)

clean:
	-@rm -r -f .obj 2>/dev/null || true
	-@rm -r -f bin 2>/dev/null || true
	-@rm launch.exe 2>/dev/null || true
