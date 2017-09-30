$(shell mkdir -p .build/src .build/test bin 2>/dev/null)

SRC = $(wildcard src/*.cpp)
TST = $(wildcard test/*.cpp)
SOBJ = $(patsubst %.cpp, .build/%.o, $(SRC))
TOBJ = $(patsubst %.cpp, .build/%.o, $(TST))
DEP = $(patsubst %.o, %.d, $(SOBJ) $(TOBJ))

CC = g++
CF = -g -std=c++14 -Wall -Wpedantic
LF = -g
LIB = -lncurses -lwiringPi -pthread
INC = -I include/
LINK = $(CC) $(LF) $^ -o $@ $(LIB)

all: launch utilities
utilities: pidtest bmptest serialtest gpstest pwmtest skips bin2txt fbtest

install:
	yes | sudo apt-get install libncurses5-dev wiringpi

# Target executable #

launch: $(SOBJ)
	$(LINK)

pidtest: bin/pidtest
bmptest: bin/bmptest
serialtest: bin/serialtest
gpstest: bin/gpstest
pwmtest: bin/pwmtest
skips: bin/skips
bin2txt: bin/bin2txt
fbtest: bin/fbtest

### Test executables ###

bin/pidtest: .build/test/pidtest.o .build/src/pid.o
	$(LINK)

bin/bmptest: .build/test/bmptest.o .build/src/bmp.o .build/src/i2c.o
	$(LINK)

bin/serialtest: .build/test/serialtest.o .build/src/ardimu.o
	$(LINK)

bin/gpstest: .build/test/gpstest.o .build/src/gps.o
	$(LINK)

bin/pwmtest: .build/test/pwmtest.o .build/src/pwm.o .build/src/i2c.o \
	.build/src/motor.o
	$(LINK)

bin/skips: .build/test/skips.o
	$(LINK)

bin/bin2txt: .build/test/bin2txt.o .build/src/uavcore.o
	$(LINK)

bin/fbtest: .build/test/fbtest.o .build/src/freebody.o
	$(LINK)

### Pattern recipes ###

%.o: %.d
	$(CC) $(CF) -c $(patsubst .build/%.o, %.cpp, $@) $(INC) -o $@
	@touch $@

.build/src/%.d: src/%.cpp
	$(CC) -std=c++14 -MM $< -MT $@ -MP -MF $@ $(INC)

.build/test/%.d: test/%.cpp
	$(CC) -std=c++14 -MM $< -MT $@ -MP -MF $@ $(INC)

### Clean ###

clean:
	-@rm -r -f .build 2>/dev/null || true
	-@rm -r -f bin 2>/dev/null || true
	-@rm launch.exe 2>/dev/null || true

ifneq ($(MAKECMDGOALS), clean)
-include $(DEP)
endif
