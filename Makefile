$(shell mkdir -p .build/src .build/test bin 2>/dev/null)

SRC = $(wildcard src/*.cpp)
TST = $(wildcard test/*.cpp)
SOBJ = $(patsubst %.cpp, .build/%.o, $(SRC))
TOBJ = $(patsubst %.cpp, .build/%.o, $(TST))
DEP = $(patsubst %.o, %.d, $(SOBJ) $(TOBJ))

CC = g++
CF = -g -std=c++11 -Wall -Wpedantic
LF = -g
LIB = -lncurses -lwiringPi
INC = -I include/
LINK = $(CC) $(LF) $^ -o $@ $(LIB)

all: launch.exe tests
tests: chronotest iostest pidtest bmptest serialtest \
	filtertest ctrltest skipstest monitortest

install:
	yes | sudo apt-get install libncurses5-dev wiringpi

# Target executable #

launch.exe: $(SOBJ)
	$(LINK)

chronotest: bin/chronotest
iostest: bin/iostest
pidtest: bin/pidtest
timetest: bin/timetest
bmptest: bin/bmptest
serialtest: bin/serialtest
filetest: bin/filetest
filtertest: bin/filtertest
ctrltest: bin/ctrltest
skipstest: bin/skipstest
monitortest: bin/monitortest

### Test executables ###

bin/pidtest: .build/test/pidtest.o .build/src/pid.o
	$(LINK)

bin/bmptest: .build/test/bmptest.o .build/src/bmp.o \
	.build/src/i2c.o .build/src/smem.o
	$(LINK)

bin/serialtest: .build/test/serialtest.o .build/src/ardimu.o .build/src/smem.o
	$(LINK)

bin/filtertest: .build/test/filtertest.o .build/src/filters.o
	$(LINK)

bin/ctrltest: .build/test/ctrltest.o .build/src/control.o \
	.build/src/filters.o .build/src/pid.o .build/src/smem.o \
	.build/src/ardimu.o .build/src/bmp.o .build/src/i2c.o \
	.build/src/monitor.o
	$(LINK)

bin/chronotest: .build/test/chronotest.o
	$(LINK)

bin/iostest: .build/test/iostest.o
	$(LINK)

bin/skipstest: .build/test/skipstest.o
	$(LINK)

bin/monitortest: .build/test/monitortest.o .build/src/monitor.o
	$(LINK)

### Pattern recipes ###

%.o: %.d
	$(CC) $(CF) -c $(patsubst .build/%.o, %.cpp, $@) $(INC) -o $@
	@touch $@

.build/src/%.d: src/%.cpp
	$(CC) -std=c++11 -MM $< -MT $@ -MP -MF $@ $(INC)

.build/test/%.d: test/%.cpp
	$(CC) -std=c++11 -MM $< -MT $@ -MP -MF $@ $(INC)

### Clean ###

clean:
	-@rm -r -f .build 2>/dev/null || true
	-@rm -r -f bin 2>/dev/null || true
	-@rm launch.exe 2>/dev/null || true

-include $(DEP)
